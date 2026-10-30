#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""高速抓取（固件 B 命令）的波形与频谱窗口。"""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from adc_core import ADC_FULL_SCALE, VREF_MV, burst_summary, compute_spectrum


class BurstWindow(tk.Toplevel):
    def __init__(self, master: tk.Misc, recapture):
        super().__init__(master)
        self.recapture = recapture
        self.samples: list[int] = []
        self.total_us = 0
        self.meta: dict = {}
        self.summary: dict | None = None
        self.sp = None

        self.title("高速噪声抓取 — 波形 / 频谱")
        self.geometry("980x760")
        self.minsize(760, 560)

        top = ttk.Frame(self, padding=(8, 6))
        top.pack(fill="x")
        self.info_var = tk.StringVar(value="尚未抓取")
        ttk.Label(top, textvariable=self.info_var, style="Hint.TLabel",
                  wraplength=640, justify="left").pack(side="left", fill="x", expand=True)
        ttk.Button(top, text="重新抓取", command=self.recapture).pack(side="right", padx=2)
        ttk.Button(top, text="导出 CSV", command=self.export_csv).pack(side="right", padx=2)

        self.fig = Figure(figsize=(9, 6), dpi=100, constrained_layout=True)
        self.ax_wave = self.fig.add_subplot(211)
        self.ax_spec = self.fig.add_subplot(212)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_empty()

    # ------------------------------------------------------------------ 状态
    def set_pending(self):
        self.info_var.set("抓取中…（固件正在高速读取原始码）")

    def set_error(self, msg: str):
        self.info_var.set("抓取失败：" + msg)

    # ------------------------------------------------------------------ 绘图
    def _draw_empty(self):
        self.ax_wave.clear()
        self.ax_wave.set_title("原始码波形")
        self.ax_wave.set_xlabel("时间 / ms")
        self.ax_wave.set_ylabel("mV")
        self.ax_wave.grid(True, alpha=0.3)
        self.ax_wave.text(0.5, 0.5, "等待抓取…", transform=self.ax_wave.transAxes,
                          ha="center", va="center", color="#888888")
        self.ax_spec.clear()
        self.ax_spec.set_title("幅度谱（去均值 + Hann 窗）")
        self.ax_spec.set_xlabel("频率 / Hz")
        self.ax_spec.set_ylabel("幅度 / mV")
        self.ax_spec.grid(True, alpha=0.3)
        self.canvas.draw_idle()

    def set_data(self, samples, total_us: int, meta: dict | None = None):
        self.samples = list(samples)
        self.total_us = int(total_us)
        self.meta = meta or {}
        summary = burst_summary(self.samples, self.total_us)
        self.summary = summary
        fs = summary["fs"]
        n = summary["n"]
        if n < 8 or fs <= 0:
            self.set_error("采样率无效")
            return

        mv = np.asarray(self.samples, dtype=float) * VREF_MV / ADC_FULL_SCALE
        t_ms = np.arange(n) / fs * 1000.0

        self.ax_wave.clear()
        self.ax_wave.plot(t_ms, mv, lw=0.8, color="#1f77b4")
        self.ax_wave.set_title(f"原始码波形（{n} 点 @ {fs:.0f} Hz）")
        self.ax_wave.set_xlabel("时间 / ms")
        self.ax_wave.set_ylabel("mV")
        self.ax_wave.grid(True, alpha=0.3)

        self.sp = compute_spectrum(self.samples, fs)
        freqs = self.sp["freqs"]
        mag = self.sp["mag_mv"]
        fmax = min(1000.0, fs / 2.0)
        mask = freqs <= fmax
        self.ax_spec.clear()
        self.ax_spec.plot(freqs[mask], mag[mask], lw=0.9, color="#ff7f0e")
        self.ax_spec.set_title(f"幅度谱（分辨率 {fs / n:.2f} Hz/bin）")
        self.ax_spec.set_xlabel("频率 / Hz")
        self.ax_spec.set_ylabel("幅度 / mV")
        self.ax_spec.grid(True, alpha=0.3)
        for f, label in ((50.0, "50 Hz"), (100.0, "100 Hz")):
            if f <= fmax:
                self.ax_spec.axvline(f, color="#999999", lw=0.8, ls="--")
                self.ax_spec.text(f, self.ax_spec.get_ylim()[1] * 0.92,
                                  label, fontsize=8, color="#666666", rotation=90,
                                  va="top", ha="right")
        if summary.get("f_peak"):
            self.ax_spec.plot([summary["f_peak"]], [summary["amp_peak_mv"]], "v",
                              color="#d62728", ms=7,
                              label=f"主频 {summary['f_peak']:.1f} Hz")
            self.ax_spec.legend(loc="upper right", fontsize=8)

        self.info_var.set(
            f"{n} 点 @ {fs:.0f} Hz（{fs / n:.2f} Hz/bin） | "
            f"均值 {summary['mean_code']:.1f} 码 | "
            f"峰峰 {summary['pp_code']:.0f} 码 / {summary['pp_mv']:.1f} mV | "
            f"σ {summary['std_code']:.2f} 码 / {summary['std_mv']:.2f} mV | "
            f"主频 {summary['f_peak']:.1f} Hz | "
            f"50 Hz {summary.get('amp_50_mv', 0):.2f} mV，"
            f"100 Hz {summary.get('amp_100_mv', 0):.2f} mV")
        self.canvas.draw_idle()

    # ------------------------------------------------------------------ 导出
    def export_csv(self):
        if not self.samples or not self.summary:
            messagebox.showinfo("没有数据", "还没有抓取到数据")
            return
        default = f"adc_burst_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path = filedialog.asksaveasfilename(
            title="导出高速抓取", defaultextension=".csv", initialfile=default,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        fs = self.summary["fs"]
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["# burst_n", len(self.samples)])
            w.writerow(["# fs_hz", f"{fs:.3f}"])
            w.writerow(["# total_us", self.total_us])
            w.writerow(["# chan", self.meta.get("chan", "")])
            w.writerow(["# atten_db", self.meta.get("atten", "")])
            for k in ("f_peak", "amp_peak_mv", "amp_50_mv", "amp_100_mv",
                      "mean_code", "pp_code", "std_code"):
                w.writerow([f"# {k}", self.summary.get(k)])
            w.writerow(["index", "t_ms", "raw_code", "mv"])
            for i, raw in enumerate(self.samples):
                w.writerow([i, f"{i / fs * 1000:.4f}", raw,
                            f"{raw * VREF_MV / ADC_FULL_SCALE:.3f}"])
        messagebox.showinfo("已导出", f"已保存到 {Path(path).name}")
