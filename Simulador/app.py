#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
App de controle dos sensores CAN (Arduino Nano + MCP2515).

Os sliders trabalham em VALOR DE DIAGNOSTICO. O app converte para o valor
bruto (raw) de 16 bits usando uma calibracao linear de 2 pontos (editavel
na tela) e envia os 8 bytes prontos ao Arduino, que apenas retransmite na
CAN (ID 0x18F00F52, DLC 8, 250 kbps, 50 Hz).

Calibracao inicial (medida no diagnostico):
  Sensor 1: diag 0 -> raw 4000 ; diag 100 -> raw 6000   | slider 0..400
  Sensor 2: diag 0 -> raw 23500; diag 9.5 -> raw 42000   | slider 0..23

Conversao:  raw = raw0 + (rawRef - raw0) / diagRef * diag
O raw e' limitado a 0..65535 (2 bytes). Se estourar, o app avisa.

Bytes livres DATA[4..7] (padrao 55 9F 1F FF) podem ser editados
individualmente em hex e aplicados ("single shot") ao stream.

Requisitos:  pip install pyserial
Uso:         python app.py
"""

import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUD = 115200
CAN_ID = 0x18F00F52
RAW_MAX = 0xFFFF  # limite fisico de 2 bytes

# VID (fabricante USB) de chips seriais tipicos de Arduino / clones.
ARDUINO_VIDS = {
    0x2341,  # Arduino LLC
    0x2A03,  # Arduino SA (.org)
    0x1B4F,  # SparkFun
    0x239A,  # Adafruit
    0x1A86,  # WCH CH340 / CH341 (Nano clones)
    0x0403,  # FTDI
    0x10C4,  # Silicon Labs CP210x
}
# Palavras-chave na descricao/fabricante da porta.
ARDUINO_KEYWORDS = (
    "arduino", "ch340", "ch341", "usb-serial", "usb serial",
    "cp210", "ftdi", "wch", "silicon labs",
)


def parse_float(s, default=0.0):
    try:
        return float(str(s).replace(",", "."))
    except (ValueError, TypeError):
        return default


class Sensor:
    """Um sensor: slider em diagnostico + calibracao linear editavel."""

    def __init__(self, parent, app, title, diag_max, resolution, cal):
        self.app = app
        self.diag_max = diag_max
        self.clamped = False

        box = ttk.LabelFrame(parent, text=f"{title}  (diagnostico 0 .. {diag_max})")
        box.pack(fill="x", padx=10, pady=6)

        # slider em valor de diagnostico
        self.scale = tk.Scale(box, from_=0, to=diag_max, resolution=resolution,
                              orient="horizontal", length=430,
                              command=lambda e: self.app.on_change())
        self.scale.pack(fill="x", padx=10, pady=(6, 0))

        # calibracao editavel: raw@0 | diag ref | raw ref
        calf = ttk.Frame(box)
        calf.pack(fill="x", padx=10, pady=(2, 2))
        ttk.Label(calf, text="Calib:  raw@diag0=").pack(side="left")
        self.e_raw0 = self._entry(calf, cal["raw0"])
        ttk.Label(calf, text=" | diag ref=").pack(side="left")
        self.e_dref = self._entry(calf, cal["diag_ref"])
        ttk.Label(calf, text=" -> raw=").pack(side="left")
        self.e_rref = self._entry(calf, cal["raw_ref"])

        # leitura resultante
        self.lbl = ttk.Label(box, text="", font=("Consolas", 10))
        self.lbl.pack(anchor="w", padx=10, pady=(0, 6))

    def _entry(self, parent, value):
        e = ttk.Entry(parent, width=8)
        e.insert(0, str(value))
        e.bind("<KeyRelease>", lambda ev: self.app.on_change())
        e.pack(side="left")
        return e

    def raw_value(self):
        """Converte o valor de diagnostico atual em raw 16 bits (clampado)."""
        diag = self.scale.get()
        raw0 = parse_float(self.e_raw0.get())
        dref = parse_float(self.e_dref.get(), 1.0)
        rref = parse_float(self.e_rref.get())
        if dref == 0:
            dref = 1.0
        raw = raw0 + (rref - raw0) / dref * diag
        raw = int(round(raw))
        clamped = max(0, min(RAW_MAX, raw))
        self.clamped = (clamped != raw)
        return clamped

    def bytes(self):
        raw = self.raw_value()
        return raw & 0xFF, (raw >> 8) & 0xFF

    def update_readout(self):
        diag = self.scale.get()
        raw = self.raw_value()
        lo, hi = raw & 0xFF, (raw >> 8) & 0xFF
        txt = (f"diag {diag:>7}   ->   raw {raw:>5}  (0x{raw:04X})   "
               f"bytes {lo:02X} {hi:02X}")
        if self.clamped:
            txt += "   ! LIMITADO a 65535 (FF FF) - fora da faixa de 2 bytes"
            self.lbl.config(text=txt, foreground="#b00")
        else:
            self.lbl.config(text=txt, foreground="")


class App:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.last_sent = None
        root.title("Controle Sensores CAN - 18F00F52")
        root.geometry("620x520")
        root.resizable(False, False)

        pad = {"padx": 10, "pady": 6}

        # ---- conexao ----
        top = ttk.Frame(root)
        top.pack(fill="x", **pad)
        self.connect_btn = ttk.Button(top, text="Conectar (auto)",
                                       command=self.toggle_connect)
        self.connect_btn.pack(side="left")
        self.status = ttk.Label(top, text="desconectado", foreground="#b00")
        self.status.pack(side="left", padx=10)

        # ---- sensores ----
        self.s1 = Sensor(root, self, "Sensor 1", diag_max=400, resolution=1,
                         cal={"raw0": 4000, "diag_ref": 100, "raw_ref": 6000})
        self.s2 = Sensor(root, self, "Sensor 2", diag_max=23, resolution=0.1,
                         cal={"raw0": 23500, "diag_ref": 9.5, "raw_ref": 42000})

        # ---- bytes livres DATA[4..7] ----
        bfree = ttk.LabelFrame(root, text="Bytes livres DATA[4..7] (hex) - single shot")
        bfree.pack(fill="x", **pad)
        row = ttk.Frame(bfree)
        row.pack(fill="x", padx=10, pady=8)
        self.byte_entries = []
        defaults = ["55", "9F", "1F", "FF"]
        for i, d in enumerate(defaults):
            ttk.Label(row, text=f"D{i + 4}:").pack(side="left")
            e = ttk.Entry(row, width=4)
            e.insert(0, d)
            e.pack(side="left", padx=(2, 10))
            self.byte_entries.append(e)
        ttk.Button(row, text="Aplicar", command=self.on_change).pack(side="left")

        # ---- previa do frame ----
        fbox = ttk.LabelFrame(root, text="Frame CAN")
        fbox.pack(fill="x", **pad)
        self.frame_lbl = ttk.Label(fbox, text="", font=("Consolas", 11))
        self.frame_lbl.pack(anchor="w", padx=10, pady=8)

        self.on_change()

    # ---- montagem/envio ----
    def free_bytes(self):
        out = []
        for e in self.byte_entries:
            try:
                v = int(e.get(), 16) & 0xFF
            except ValueError:
                v = 0
            out.append(v)
        return out

    def build_frame(self):
        s1_lo, s1_hi = self.s1.bytes()
        s2_lo, s2_hi = self.s2.bytes()
        return [s1_lo, s1_hi, s2_lo, s2_hi] + self.free_bytes()

    def on_change(self):
        self.s1.update_readout()
        self.s2.update_readout()
        data = self.build_frame()
        data_str = " ".join(f"{b:02X}" for b in data)
        self.frame_lbl.config(text=f"ID {CAN_ID:08X}   DLC 8   DATA {data_str}")
        self.send(data)

    def send(self, data):
        if not (self.ser and self.ser.is_open):
            return
        if data == self.last_sent:
            return
        msg = "F" + "".join(f"{b:02X}" for b in data) + "\n"
        try:
            self.ser.write(msg.encode("ascii"))
            self.last_sent = data
        except serial.SerialException:
            self.disconnect()

    # ---- serial ----
    def detect_ports(self):
        """Ordena as portas seriais, das mais provaveis de ser o Arduino
        para as menos provaveis. Pontua por VID USB conhecido e por
        palavra-chave na descricao/fabricante."""
        scored = []
        for p in serial.tools.list_ports.comports():
            score = 0
            if p.vid in ARDUINO_VIDS:
                score += 10
            text = f"{p.description or ''} {p.manufacturer or ''}".lower()
            if any(k in text for k in ARDUINO_KEYWORDS):
                score += 5
            scored.append((score, p.device))
        # pontuados primeiro (maior score); demais portas como ultimo recurso
        scored.sort(key=lambda t: t[0], reverse=True)
        return [dev for _, dev in scored]

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        candidates = self.detect_ports()
        if not candidates:
            messagebox.showwarning(
                "Arduino", "Nenhuma porta serial encontrada.\n"
                "Verifique se o Arduino esta conectado ao USB.")
            return

        last_err = None
        for port in candidates:
            try:
                self.ser = serial.Serial(port, BAUD, timeout=0.1)
            except serial.SerialException as e:
                last_err = e
                self.ser = None
                continue
            self.status.config(text=f"conectado ({port})", foreground="#080")
            self.connect_btn.config(text="Desconectar")
            self.last_sent = None
            self.on_change()
            return

        messagebox.showerror(
            "Erro de conexao",
            "Arduino detectado, mas nao foi possivel abrir a porta.\n"
            "A porta pode estar em uso por outro programa.\n\n"
            f"{last_err}" if last_err else "Nenhuma porta pode ser aberta.")

    def disconnect(self):
        try:
            if self.ser:
                self.ser.close()
        finally:
            self.ser = None
        self.status.config(text="desconectado", foreground="#b00")
        self.connect_btn.config(text="Conectar")


def main():
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
