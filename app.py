#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Monitor/configurador do BRIDGE CAN (Arduino Nano + 2x MCP2515).

O Arduino fica no meio do barramento, repassa tudo e corrige o frame
0x18F00F52. A correcao usa um GANHO PROPORCIONAL UNICO: aumenta o O2 em
+X% e diminui o NOx em -X%. So atua quando o O2 fica ABAIXO de um limite
(trigger) e entra/sai com uma rampa suave (ms). Um segundo limite (piso)
reforca a correcao, oscilando, para o O2 corrigido nao baixar de um valor.
O pino D3 (negativo) serve de override por hardware (bypass ou corrigindo).
Tudo fica na EEPROM. A calibracao raw->valor exibido e' fixa.

Requisitos:  pip install pyserial
Uso:         python app.py
"""

import datetime
import os
import sys
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

BAUD = 115200

# Calibracao FIXA raw -> valor exibido (raw0, dref, rref) - igual ao firmware
CAL_NOX = (4000.0, 100.0, 6000.0)     # Sensor 1
CAL_O2 = (23500.0, 9.5, 42000.0)      # Sensor 2


def to_float(s, default=0.0):
    try:
        return float(str(s).replace(",", "."))
    except (ValueError, TypeError):
        return default


def raw_to_diag(raw, cal):
    raw0, dref, rref = cal
    den = (rref - raw0) or 1.0
    return (raw - raw0) * dref / den


def app_dir():
    # Pasta base para logs: ao lado do .exe quando empacotado (PyInstaller),
    # senao ao lado do app.py.
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


class App:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.rxbuf = ""

        # log em CSV
        self.logf = None
        self.log_t0 = 0.0
        self.log_n = 0
        self.log_path = ""

        root.title("Bridge CAN 18F00F52 - corretor de sensores")
        root.geometry("560x630")
        root.resizable(False, False)

        pad = {"padx": 10, "pady": 6}

        # ---- conexao ----
        top = ttk.Frame(root)
        top.pack(fill="x", **pad)
        ttk.Label(top, text="Porta:").pack(side="left")
        self.port_cb = ttk.Combobox(top, width=14, state="readonly")
        self.port_cb.pack(side="left", padx=6)
        ttk.Button(top, text="Atualizar", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(top, text="Conectar", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=6)
        self.status = ttk.Label(top, text="desconectado", foreground="#b00")
        self.status.pack(side="left", padx=6)

        # ---- gravacao de log ----
        logbar = ttk.Frame(root)
        logbar.pack(fill="x", padx=10)
        self.log_btn = ttk.Button(logbar, text="Gravar log", command=self.toggle_log)
        self.log_btn.pack(side="left")
        self.log_lbl = ttk.Label(logbar, text="log parado", foreground="#777")
        self.log_lbl.pack(side="left", padx=8)

        # ---- status bypass/corrigindo ----
        self.state_lbl = tk.Label(root, text="AGUARDANDO", font=("Segoe UI", 16, "bold"),
                                  fg="#555", bg="#eee", pady=8)
        self.state_lbl.pack(fill="x", padx=10, pady=(2, 8))

        # ---- leituras ao vivo ----
        live = ttk.LabelFrame(root, text="Leitura (bancada)")
        live.pack(fill="x", padx=10, pady=4)

        self.o2_read = self._reading(live, "Oxigenio (O2)   Lido:")
        self.o2_corr = self._reading(live, "                Corrigido:", "#060")
        ttk.Separator(live).pack(fill="x", padx=10, pady=4)
        self.nox_read = self._reading(live, "NOx (ppm)       Lido:")
        self.nox_corr = self._reading(live, "                Corrigido:", "#060")
        ttk.Frame(live, height=4).pack()

        # ---- configuracoes ----
        cfg = ttk.LabelFrame(root, text="Configuracoes (gravam na EEPROM)")
        cfg.pack(fill="x", padx=10, pady=6)

        self.e_pct = self._cfg_row(
            cfg, "Ganho proporcional:", "0", "%   (O2 +X% / NOx -X%)",
            lambda: self.send(f"PCT:{to_float(self.e_pct.get()):g}"))

        self.e_trg = self._cfg_row(
            cfg, "Trigger O2 (corrige se O2 <):", "5", "",
            self.apply_trg)

        self.e_flr = self._cfg_row(
            cfg, "Piso 1 O2 (nao baixar de):", "3", "  (0 = desliga; oscila)",
            self.apply_flr)

        self.e_flr2 = self._cfg_row(
            cfg, "Piso 2 O2 (mais baixo):", "0", "  (0 = sem ciclo)",
            self.apply_flr2)

        self.e_hld = self._cfg_row(
            cfg, "Tempo X (segura no Piso 2):", "5000", "ms",
            lambda: self.send(f"HLD:{int(to_float(self.e_hld.get()))}"))

        self.e_per = self._cfg_row(
            cfg, "Tempo Y (segura no Piso 1):", "60000", "ms   (entre mergulhos)",
            lambda: self.send(f"PER:{int(to_float(self.e_per.get()))}"))

        self.e_rmp = self._cfg_row(
            cfg, "Rampa:", "2000", "ms   (suaviza entra/sai do trigger)",
            lambda: self.send(f"RMP:{int(to_float(self.e_rmp.get()))}"))

        # Modo de correcao (proporcional a faixa trigger<->piso)
        pm = ttk.Frame(cfg)
        pm.pack(fill="x", padx=10, pady=3)
        ttk.Label(pm, text="Correcao:", width=26).pack(side="left")
        self.prop_mode = ttk.Combobox(pm, width=20, state="readonly",
                                      values=["Fixa (piso oscila)",
                                              "Proporcional a faixa"])
        self.prop_mode.current(0)
        self.prop_mode.pack(side="left")
        ttk.Button(pm, text="Aplicar", command=self.apply_prop).pack(side="left", padx=6)

        # D3
        d3 = ttk.Frame(cfg)
        d3.pack(fill="x", padx=10, pady=3)
        ttk.Label(d3, text="D3 negativo:", width=26).pack(side="left")
        self.d3_mode = ttk.Combobox(d3, width=12, state="readonly",
                                    values=["Bypass", "Corrigindo"])
        self.d3_mode.current(0)
        self.d3_mode.pack(side="left")
        ttk.Button(d3, text="Aplicar", command=self.apply_d3).pack(side="left", padx=6)
        ttk.Label(d3, text="(pino D3 em nivel baixo)").pack(side="left")

        self.refresh_ports()
        self.poll_serial()

    # ---- widgets helpers ----
    def _reading(self, parent, label, fg="#000"):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=10, pady=1)
        ttk.Label(row, text=label, width=26, font=("Consolas", 10)).pack(side="left")
        val = ttk.Label(row, text="---", font=("Consolas", 13, "bold"), foreground=fg)
        val.pack(side="left")
        return val

    def _cfg_row(self, parent, label, default, suffix, cmd):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=10, pady=3)
        ttk.Label(row, text=label, width=26).pack(side="left")
        e = ttk.Entry(row, width=8)
        e.insert(0, default)
        e.pack(side="left")
        ttk.Button(row, text="Aplicar", command=cmd).pack(side="left", padx=6)
        if suffix:
            ttk.Label(row, text=suffix).pack(side="left")
        return e

    def apply_d3(self):
        # Bypass = 1, Corrigindo = 0
        self.send(f"D3:{1 if self.d3_mode.get() == 'Bypass' else 0}")

    def apply_trg(self):
        self.send(f"TRG:{to_float(self.e_trg.get()):g}")
        self.check_band()

    def apply_flr(self):
        self.send(f"FLR:{to_float(self.e_flr.get()):g}")
        self.check_band()

    def apply_flr2(self):
        self.send(f"FL2:{to_float(self.e_flr2.get()):g}")
        self.check_band()

    def apply_prop(self):
        # Proporcional a faixa = 1, Fixa = 0
        on = self.prop_mode.get().startswith("Prop")
        self.send(f"PRP:{1 if on else 0}")
        if on:
            self.check_band()

    def check_band(self):
        # Alertas simples de faixa: Piso 1 nao pode ficar acima do trigger, e o
        # Piso 2 (quando ligado) precisa ser MENOR que o Piso 1 para o ciclo valer.
        trg = to_float(self.e_trg.get())
        flr = to_float(self.e_flr.get())
        flr2 = to_float(self.e_flr2.get())
        if flr > 0 and flr > trg:
            messagebox.showwarning(
                "Piso acima do trigger",
                f"O Piso 1 de O2 ({flr:g}) esta MAIOR que o trigger ({trg:g}).\n"
                "A faixa de correcao fica invalida - ajuste para Piso 1 < trigger.")
        elif flr2 > 0 and flr2 >= flr:
            messagebox.showwarning(
                "Piso 2 acima do Piso 1",
                f"O Piso 2 ({flr2:g}) deve ser MENOR que o Piso 1 ({flr:g}).\n"
                "O ciclo de piso so vale com Piso 2 < Piso 1.")

    # ---- log CSV ----
    def toggle_log(self):
        if self.logf:
            self.stop_log()
        else:
            self.start_log()

    def start_log(self):
        fn = datetime.datetime.now().strftime("log_%Y%m%d_%H%M%S.csv")
        logdir = os.path.join(app_dir(), "logs")
        path = os.path.join(logdir, fn)
        try:
            os.makedirs(logdir, exist_ok=True)
            self.logf = open(path, "w", encoding="utf-8")
        except OSError as e:
            messagebox.showerror("Erro ao gravar log", str(e))
            self.logf = None
            return
        self.logf.write("t_s,o2_read,o2_corr,nox_read,nox_corr,"
                        "o2_trigger,o2_floor,o2_floor2,state,ramp_pct\n")
        self.log_t0 = time.perf_counter()
        self.log_n = 0
        self.log_path = path
        self.log_btn.config(text="Parar log")
        self.log_lbl.config(text=f"gravando -> {fn}  (0)", foreground="#b00")

    def stop_log(self):
        if self.logf:
            try:
                self.logf.close()
            except OSError:
                pass
        self.logf = None
        self.log_btn.config(text="Gravar log")
        if self.log_path:
            self.log_lbl.config(
                text=f"salvo: {os.path.basename(self.log_path)}  ({self.log_n})",
                foreground="#080")
        else:
            self.log_lbl.config(text="log parado", foreground="#777")

    def log_sample(self, o2r, o2c, nr, nc, st, kmil):
        if not self.logf:
            return
        t = time.perf_counter() - self.log_t0
        trg = to_float(self.e_trg.get())
        flr = to_float(self.e_flr.get())
        flr2 = to_float(self.e_flr2.get())
        try:
            self.logf.write(f"{t:.3f},{o2r:.3f},{o2c:.3f},{nr:.3f},{nc:.3f},"
                            f"{trg:g},{flr:g},{flr2:g},{st},{kmil / 10:.1f}\n")
        except OSError:
            self.stop_log()
            return
        self.log_n += 1
        if self.log_n % 5 == 0:
            self.logf.flush()
            self.log_lbl.config(
                text=f"gravando -> {os.path.basename(self.log_path)}  ({self.log_n})")

    # ---- serial ----
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_cb.get():
            self.port_cb.current(0)

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_cb.get()
        if not port:
            messagebox.showwarning("Porta", "Selecione uma porta serial.")
            return
        try:
            self.ser = serial.Serial(port, BAUD, timeout=0)
        except serial.SerialException as e:
            messagebox.showerror("Erro de conexao", str(e))
            return
        self.rxbuf = ""
        self.status.config(text=f"conectado ({port})", foreground="#080")
        self.connect_btn.config(text="Desconectar")
        self.root.after(400, lambda: self.send("GET"))  # pede config

    def disconnect(self):
        try:
            if self.ser:
                self.ser.close()
        finally:
            self.ser = None
        self.status.config(text="desconectado", foreground="#b00")
        self.connect_btn.config(text="Conectar")
        self.set_state(None)

    def send(self, line):
        if not (self.ser and self.ser.is_open):
            return
        try:
            self.ser.write((line + "\n").encode("ascii"))
        except serial.SerialException:
            self.disconnect()

    def poll_serial(self):
        if self.ser and self.ser.is_open:
            try:
                data = self.ser.read(4096)
            except serial.SerialException:
                self.disconnect()
                data = b""
            if data:
                self.rxbuf += data.decode("ascii", errors="ignore")
                while "\n" in self.rxbuf:
                    line, self.rxbuf = self.rxbuf.split("\n", 1)
                    self.handle_line(line.strip())
        self.root.after(50, self.poll_serial)

    # ---- estado ----
    def set_state(self, state, k=0):
        if state is None:
            self.state_lbl.config(text="AGUARDANDO", fg="#555", bg="#eee")
        elif state == 1:
            txt = "CORRIGINDO"
            if k < 999:
                txt += f"  (rampa {k / 10:.0f}%)"
            self.state_lbl.config(text=txt, fg="#fff", bg="#0a7d2c")
        else:
            txt = "BYPASS"
            if k > 1:
                txt += f"  (rampa {k / 10:.0f}%)"
            self.state_lbl.config(text=txt, fg="#fff", bg="#b06a00")

    def handle_line(self, line):
        if not line:
            return
        if line.startswith("T "):
            parts = line.split()
            if len(parts) == 7:
                try:
                    s1r, s1c, s2r, s2c, st, kmil = (int(x) for x in parts[1:])
                except ValueError:
                    return
                o2r = raw_to_diag(s2r, CAL_O2)
                o2c = raw_to_diag(s2c, CAL_O2)
                nr = raw_to_diag(s1r, CAL_NOX)
                nc = raw_to_diag(s1c, CAL_NOX)
                self.o2_read.config(text=f"{o2r:8.2f}")
                self.o2_corr.config(text=f"{o2c:8.2f}")
                self.nox_read.config(text=f"{nr:8.1f}")
                self.nox_corr.config(text=f"{nc:8.1f}")
                self.set_state(st, kmil)
                self.log_sample(o2r, o2c, nr, nc, st, kmil)
        elif line.startswith("CFG "):
            parts = line[4:].split(",")
            if len(parts) >= 5:
                v = [to_float(x) for x in parts]
                self._set(self.e_pct, v[0])
                self._set(self.e_trg, v[1])
                self._set(self.e_flr, v[2])
                self._set(self.e_rmp, int(v[3]))
                self.d3_mode.current(0 if int(v[4]) else 1)  # 1=Bypass, 0=Corrigindo
                if len(parts) >= 6:
                    self.prop_mode.current(1 if int(v[5]) else 0)
                if len(parts) >= 9:
                    self._set(self.e_flr2, v[6])
                    self._set(self.e_hld, int(v[7]))
                    self._set(self.e_per, int(v[8]))

    def _set(self, entry, val):
        entry.delete(0, "end")
        entry.insert(0, f"{val:g}")


def main():
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW",
                  lambda: (app.stop_log(), app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
