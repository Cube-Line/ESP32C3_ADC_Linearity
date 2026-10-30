#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32-C3 ADC 线性度采集 GUI

功能：
  * 串口读取固件输出的 D 行（原始码 + ESP-IDF 校准 mV）
  * 实时绘制「电压 vs ADC 原始码」两条曲线：校准值 / 原始码线性换算
  * 用万用表读数作为真值，逐点记录，计算误差、斜率、RMS、INL
  * 导出标定点 CSV

用法：
    python gui.py                 # 正常连板子
    python gui.py --demo          # 无硬件演示模式
    python gui.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import json
import queue
import random
import threading
import time
from collections import deque
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, font as tkfont

import numpy as np

from adc_core import (ADC_FULL_SCALE, VREF_MV, CalPoint, Sample,
                      build_report_markdown, burst_summary, parse_data_line,
                      parse_voltage_mv, setup_matplotlib_fonts, write_points_csv)

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from noise_view import BurstWindow

BAUD = 115200
MAX_POINTS = 6000

COL_CAL = "#1f77b4"
COL_LIN = "#ff7f0e"
COL_TRUE = "#2ca02c"


# ----------------------------------------------------------------------------
# 串口读取线程
# ----------------------------------------------------------------------------
class SerialReader(threading.Thread):
    def __init__(self, port: str, baud: int, q: queue.Queue):
        super().__init__(daemon=True, name="serial-reader")
        self.port = port
        self.baud = baud
        self.q = q
        self.stop_evt = threading.Event()
        self.ser = None
        self._write_lock = threading.Lock()

    def send(self, cmd: str) -> bool:
        """向固件发送一行命令（如 'B 4096'）。"""
        with self._write_lock:
            if self.ser is None or not self.ser.is_open:
                return False
            try:
                self.ser.write((cmd.strip() + "\n").encode("ascii"))
                self.ser.flush()
                return True
            except Exception as exc:  # noqa: BLE001
                self.q.put(("error", f"串口写入失败: {exc}"))
                return False

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            # USB Serial/JTAG 上 DTR/RTS 可能让主机写入被阻塞；显式释放。
            try:
                self.ser.dtr = False
                self.ser.rts = False
            except Exception:  # noqa: BLE001
                pass
        except Exception as exc:  # noqa: BLE001
            self.q.put(("error", f"打开 {self.port} 失败: {exc}"))
            self.q.put(("closed", None))
            return

        self.q.put(("open", self.port))
        buf = b""
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:  # noqa: BLE001
                if not self.stop_evt.is_set():
                    self.q.put(("error", f"串口读取失败: {exc}"))
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if line:
                    self.q.put(("line", line))
            if len(buf) > 65536:  # 异常长行，丢弃避免内存膨胀
                buf = b""
        try:
            if self.ser:
                self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        self.q.put(("closed", None))

    def stop(self):
        self.stop_evt.set()


# ----------------------------------------------------------------------------
# 演示数据源（无硬件时验证 GUI）
# ----------------------------------------------------------------------------
class DemoSource(threading.Thread):
    """模拟电位器来回扫动，并人为加入非线性与噪声。"""

    def __init__(self, q: queue.Queue):
        super().__init__(daemon=True, name="demo-source")
        self.q = q
        self.stop_evt = threading.Event()
        self.x = 30.0
        self.direction = 1.0
        self.t = 0

    def run(self):
        self.q.put(("line", "#FW,adc_linearity,2(demo)"))
        self.q.put(("line", "#CFG,pin=GPIO3,chan=3,atten=12,width=12,n=64,period_ms=50,cali=1"))
        while not self.stop_evt.is_set():
            self.t += 50
            self.x += self.direction * 20.0
            if self.x >= 4085:
                self.x, self.direction = 4085, -1.0
            elif self.x <= 10:
                self.x, self.direction = 10, 1.0
            x = self.x
            # “真实”输入电压：带一点系统性弯曲
            true_mv = VREF_MV * x / ADC_FULL_SCALE + 9.0 * np.sin(x / 640.0) - 5.0
            # 校准输出：残留误差 + 噪声
            cal_mv = true_mv + 5.0 * np.cos(x / 900.0) + random.gauss(0, 0.5)
            raw = int(round(x + random.gauss(0, 0.4)))
            mn, mx = raw - 3, raw + 4
            std_x100 = int(round(random.gauss(1.6, 0.3) * 100))
            line = (f"D,{self.t},{int(round(x * 100))},{mn},{mx},"
                    f"{int(round(cal_mv))},{int(round(cal_mv - 2))},"
                    f"{int(round(cal_mv + 2))},{std_x100}")
            self.q.put(("line", line))
            time.sleep(0.05)

    @staticmethod
    def true_mv(raw: float) -> float:
        """演示用的“真值”模型：与 run() 中的系统性弯曲一致。"""
        return VREF_MV * raw / ADC_FULL_SCALE + 9.0 * np.sin(raw / 640.0) - 5.0

    def send(self, cmd: str) -> bool:
        """演示模式下模拟固件命令，重点支持 B（噪声抓取）。"""
        c = cmd.strip().upper()
        if c.startswith("B"):
            n = 4096
            parts = c.split()
            if len(parts) > 1:
                try:
                    n = max(16, min(8192, int(parts[1])))
                except ValueError:
                    n = 4096
            threading.Thread(target=self._burst, args=(n,), daemon=True).start()
            return True
        return True

    def _burst(self, n: int):
        fs = 20000.0
        total_us = int(n / fs * 1e6)
        self.q.put(("line", f"#BURST_BEGIN,{n},0,3,12"))
        t = np.arange(n) / fs
        dc = 200 + 0.5 * self.x
        sig = (dc + 30.0 * np.sin(2 * np.pi * 50.0 * t)
               + 4.0 * np.sin(2 * np.pi * 150.0 * t)
               + np.random.normal(0, 1.2, n))
        for i, v in enumerate(np.clip(np.round(sig), 0, 4095).astype(int)):
            self.q.put(("line", f"B,{i},{v}"))
        self.q.put(("line", f"#BURST_END,{n},{total_us}"))

    def stop(self):
        self.stop_evt.set()


# ----------------------------------------------------------------------------
# 工具函数
# ----------------------------------------------------------------------------
def setup_cjk_fonts():
    setup_matplotlib_fonts()
    matplotlib.rcParams["axes.unicode_minus"] = False


def tk_cjk_family(root: tk.Misc) -> str:
    families = set(tkfont.families(root))
    for name in ("Sarasa Gothic SC", "WenQuanYi Zen Hei", "Noto Sans CJK SC",
                 "Microsoft YaHei", "Source Han Sans SC"):
        if name in families:
            return name
    return "TkDefaultFont"


# ----------------------------------------------------------------------------
# 主界面
# ----------------------------------------------------------------------------
class AdcApp:
    def __init__(self, root: tk.Tk, demo: bool = False, port: str | None = None):
        self.root = root
        self.demo = demo
        self.q: queue.Queue = queue.Queue()
        self.reader: SerialReader | None = None
        self.demo_src: DemoSource | None = None

        self.raw_hist: deque[float] = deque(maxlen=MAX_POINTS)
        self.cal_hist: deque[float] = deque(maxlen=MAX_POINTS)
        self.lin_hist: deque[float] = deque(maxlen=MAX_POINTS)
        self.t_hist: deque[float] = deque(maxlen=MAX_POINTS)
        self.std_hist: deque[float] = deque(maxlen=MAX_POINTS)

        self.points: list[CalPoint] = []
        self._points_version = -1
        self._points_drawn_version = -1
        self.last: Sample | None = None
        self.cfg: dict[str, str] = {}
        self.wifi_on = False
        self.wifi_info: dict[str, str] = {}
        self.connected = False
        self.port_name = ""
        self.rx_count = 0
        self.last_rx_time = 0.0
        self.rate = 0.0
        self.dirty = False
        self._closing = False
        self._tick = 0

        # 高速抓取 / 频谱
        self.burst_win: BurstWindow | None = None
        self.burst_active = False
        self.burst_samples: list[int] = []
        self.burst_meta: dict = {}
        self.burst_total_us = 0
        self.burst_last: dict | None = None
        self._stable_job = None
        self.autosave_path = Path("data/autosave_session.json")

        self._build_ui()
        self._refresh_ports()
        if port:
            self.port_var.set(port)
        self._draw_axes()
        self._update_stats_panel()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.bind("<F2>", lambda _e: self.record_point())
        self.root.bind("<F9>", lambda _e: self.capture_burst())
        self.root.after(50, self._poll_queue)
        self.root.after(200, self._periodic_redraw)
        self.root.after(700, self._maybe_restore_autosave)

        if demo:
            self._start_demo()
        elif port:
            self.root.after(300, self.toggle_connect)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        self.root.title("ESP32-C3 ADC 线性度采集")
        self.root.geometry("1500x980+80+20")
        self.root.minsize(1180, 800)

        family = tk_cjk_family(self.root)
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont"):
            try:
                f = tkfont.nametofont(name)
                f.configure(family=family, size=10)
            except tk.TclError:
                pass
        style.configure("Bold.TLabel", font=(family, 10, "bold"))
        style.configure("Big.TLabel", font=(family, 13, "bold"))
        style.configure("Hint.TLabel", foreground="#666666")

        outer = ttk.Frame(self.root, padding=6)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.columnconfigure(1, weight=0, minsize=480)
        outer.rowconfigure(0, weight=1)

        # ---- 左侧绘图区 ----
        plot_frame = ttk.Frame(outer)
        plot_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.fig = plt.Figure(figsize=(9, 7), dpi=100, constrained_layout=True)
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, plot_frame, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(fill="x")

        # ---- 右侧控制区 ----
        right = ttk.Frame(outer, width=480)
        right.grid(row=0, column=1, sticky="nsew")
        right.grid_propagate(False)
        right.columnconfigure(0, weight=1)
        row = 0

        # 串口
        ser_lf = ttk.LabelFrame(right, text="串口", padding=6)
        ser_lf.grid(row=row, column=0, sticky="ew", pady=(0, 6)); row += 1
        ser_lf.columnconfigure(1, weight=1)

        ttk.Label(ser_lf, text="端口").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(ser_lf, textvariable=self.port_var, width=18)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=4)
        self.port_combo.bind("<<ComboboxSelected>>", lambda _e: self._update_port_hint())
        self.btn_refresh = ttk.Button(ser_lf, text="刷新", width=5, command=self._refresh_ports)
        self.btn_refresh.grid(row=0, column=2)

        ttk.Label(ser_lf, text="波特率").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.baud_var = tk.StringVar(value=str(BAUD))
        ttk.Combobox(ser_lf, textvariable=self.baud_var, width=8,
                     values=("115200", "230400", "460800", "921600")).grid(
            row=1, column=1, sticky="w", padx=4, pady=(4, 0))
        self.btn_connect = ttk.Button(ser_lf, text="连接", command=self.toggle_connect)
        self.btn_connect.grid(row=1, column=2, pady=(4, 0))

        self.port_hint = ttk.Label(ser_lf, text="", style="Hint.TLabel", wraplength=460)
        self.port_hint.grid(row=2, column=0, columnspan=3, sticky="w", pady=(4, 0))

        self.btn_wifi = ttk.Button(ser_lf, text="Wi-Fi 负载: 关", command=self.toggle_wifi)
        self.btn_wifi.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6, 0))

        # 当前读数：每个量占「标签 / 数值」两行，避免长文本被挤压
        rd_lf = ttk.LabelFrame(right, text="当前读数", padding=6)
        rd_lf.grid(row=row, column=0, sticky="ew", pady=(0, 6)); row += 1
        rd_lf.columnconfigure(0, weight=1, uniform="rd")
        rd_lf.columnconfigure(1, weight=1, uniform="rd")

        def add_reading(r, c, label):
            ttk.Label(rd_lf, text=label, style="Hint.TLabel").grid(
                row=r, column=c, sticky="w", padx=(0, 8))
            var = tk.StringVar(value="—")
            ttk.Label(rd_lf, textvariable=var, style="Big.TLabel", width=20,
                      anchor="w").grid(
                row=r + 1, column=c, sticky="w", padx=(0, 8), pady=(0, 3))
            return var

        self.v_raw = add_reading(0, 0, "原始码")
        self.v_cal = add_reading(0, 1, "ESP-IDF 校准")
        self.v_lin = add_reading(2, 0, "线性换算 (3300/4095)")
        self.v_diff = add_reading(2, 1, "校准 − 线性")
        self.v_noise = add_reading(4, 0, "峰峰值 / σ (单窗口)")
        self.v_rate = add_reading(4, 1, "采样率")

        # 标定点录入
        rec_lf = ttk.LabelFrame(right, text="标定点（万用表真值）", padding=6)
        rec_lf.grid(row=row, column=0, sticky="ew", pady=(0, 6)); row += 1
        rec_lf.columnconfigure(1, weight=1)

        ttk.Label(rec_lf, text="万用表电压").grid(row=0, column=0, sticky="w")
        self.mv_var = tk.StringVar()
        ent = ttk.Entry(rec_lf, textvariable=self.mv_var, width=14)
        ent.grid(row=0, column=1, sticky="ew", padx=4)
        ent.bind("<Return>", lambda _e: self.record_point())
        ttk.Label(rec_lf, text="V 或 mV", style="Hint.TLabel").grid(row=0, column=2, sticky="w")

        ttk.Label(rec_lf, text="平均点数").grid(row=1, column=0, sticky="w", pady=(4, 0))
        self.avg_var = tk.StringVar(value="10")
        ttk.Spinbox(rec_lf, from_=1, to=500, textvariable=self.avg_var,
                    width=8).grid(row=1, column=1, sticky="w", padx=4, pady=(4, 0))
        ttk.Label(rec_lf, text="稳定阈值").grid(row=1, column=2, sticky="w", pady=(4, 0))
        self.stable_mv_var = tk.StringVar(value="50")
        ttk.Entry(rec_lf, textvariable=self.stable_mv_var, width=8).grid(
            row=1, column=3, sticky="w", padx=(2, 0), pady=(4, 0))
        ttk.Label(rec_lf, text="mV 峰峰", style="Hint.TLabel").grid(
            row=1, column=4, sticky="w", pady=(4, 0))

        self.stable_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(rec_lf, text="等待稳定后记录（推荐）",
                        variable=self.stable_var).grid(
            row=2, column=0, columnspan=5, sticky="w", pady=(4, 0))
        ttk.Button(rec_lf, text="记录当前点 (F2)",
                   command=self.record_point).grid(row=3, column=0, columnspan=5,
                                                   sticky="ew", pady=(6, 0))

        # 标定点表格
        tree_lf = ttk.LabelFrame(right, text="已记录的点", padding=4)
        tree_lf.grid(row=row, column=0, sticky="nsew", pady=(0, 6)); row += 1
        right.rowconfigure(row - 1, weight=1)
        tree_lf.columnconfigure(0, weight=1)
        tree_lf.rowconfigure(0, weight=1)

        cols = ("raw", "true", "cal", "lin", "ecal", "elin")
        self.tree = ttk.Treeview(tree_lf, columns=cols, show="headings", height=7)
        heads = {"raw": "原始码", "true": "真值mV", "cal": "校准mV",
                 "lin": "线性mV", "ecal": "校准误差", "elin": "线性误差"}
        widths = {"raw": 52, "true": 56, "cal": 52, "lin": 52, "ecal": 64, "elin": 64}
        for c in cols:
            self.tree.heading(c, text=heads[c])
            self.tree.column(c, width=widths[c], anchor="center", stretch=True)
        self.tree.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(tree_lf, orient="vertical", command=self.tree.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=sb.set)

        # 统计
        st_lf = ttk.LabelFrame(right, text="统计", padding=4)
        st_lf.grid(row=row, column=0, sticky="ew", pady=(0, 6)); row += 1
        self.stats_text = tk.Text(st_lf, height=6, width=44, wrap="word",
                                  relief="flat", background=self.root.cget("background"),
                                  font=(family, 9))
        self.stats_text.pack(fill="both", expand=True)
        self.stats_text.configure(state="disabled")

        # 按钮
        btn_row = ttk.Frame(right)
        btn_row.grid(row=row, column=0, sticky="ew"); row += 1
        for i in range(4):
            btn_row.columnconfigure(i, weight=1)
        ttk.Button(btn_row, text="删除选中", command=self.delete_selected).grid(
            row=0, column=0, sticky="ew", padx=1)
        ttk.Button(btn_row, text="清空点", command=self.clear_points).grid(
            row=0, column=1, sticky="ew", padx=1)
        ttk.Button(btn_row, text="清空曲线", command=self.clear_curve).grid(
            row=0, column=2, sticky="ew", padx=1)
        ttk.Button(btn_row, text="导出 CSV", command=self.export_csv).grid(
            row=0, column=3, sticky="ew", padx=1)
        ttk.Button(btn_row, text="保存会话", command=self.save_session).grid(
            row=1, column=0, sticky="ew", padx=1, pady=(3, 0))
        ttk.Button(btn_row, text="载入会话", command=self.load_session).grid(
            row=1, column=1, sticky="ew", padx=1, pady=(3, 0))
        ttk.Button(btn_row, text="导出报告", command=self.export_report).grid(
            row=1, column=2, sticky="ew", padx=1, pady=(3, 0))
        ttk.Button(btn_row, text="噪声频谱 (F9)", command=self.capture_burst).grid(
            row=1, column=3, sticky="ew", padx=1, pady=(3, 0))

        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        status = ttk.Label(self.root, textvariable=self.status_var, anchor="w",
                           relief="sunken", padding=(6, 2))
        status.pack(fill="x", side="bottom")

    # ------------------------------------------------------------- 绘图初始化
    def _draw_axes(self):
        ax1 = self.ax1
        ax1.set_xlim(0, 4095)
        ax1.set_ylim(0, 3400)
        ax1.set_xlabel("ADC 原始码 (12 bit)")
        ax1.set_ylabel("电压 / mV")
        ax1.set_title("实时：电压 vs ADC 原始码")
        ax1.grid(True, alpha=0.3)
        self.ln_cal, = ax1.plot([], [], ".", color=COL_CAL, ms=2, alpha=0.55,
                                label="ESP-IDF 校准曲线")
        self.ln_lin, = ax1.plot([], [], ".", color=COL_LIN, ms=2, alpha=0.55,
                                label="原始码线性换算 3300/4095")
        ax1.legend(loc="lower right", fontsize=9, markerscale=4)

        ax2 = self.ax2
        ax2.set_xlim(0, 4095)
        ax2.set_ylim(-60, 60)
        ax2.set_xlabel("ADC 原始码")
        ax2.set_ylabel("误差 / mV")
        ax2.set_title("标定点误差（测量值 - 万用表真值）")
        ax2.grid(True, alpha=0.3)
        ax2.axhline(0, color="#999999", lw=0.8)
        self.rec_cal, = ax2.plot([], [], "o", color=COL_CAL, ms=6, label="校准")
        self.rec_lin, = ax2.plot([], [], "s", color=COL_LIN, ms=6, mfc="none", label="线性换算")
        ax2.legend(loc="upper right", fontsize=9)

    # --------------------------------------------------------------- 串口控制
    def _refresh_ports(self):
        if list_ports is None:
            self.port_hint.configure(text="未安装 pyserial")
            return
        ports = list(list_ports.comports())
        # 常见 ESP32 设备（ttyACM / ttyUSB）排在前面
        ports.sort(key=lambda p: (0 if ("ttyACM" in p.device or "ttyUSB" in p.device) else 1,
                                  p.device))
        values = [p.device for p in ports]
        self.port_combo.configure(values=values)
        self._port_desc = {p.device: (p.description or "") for p in ports}
        if values and not self.port_var.get():
            self.port_var.set(values[0])
        if not values:
            self.port_hint.configure(
                text="未发现串口。WSL 下需要 usbipd 把 USB 设备转发进来；"
                     "也可以手动输入 /dev/ttyACM0 或 /dev/ttyUSB0。")
        else:
            self._update_port_hint()

    def _update_port_hint(self):
        dev = self.port_var.get().strip()
        desc = getattr(self, "_port_desc", {}).get(dev, "")
        if desc.lower() in ("n/a", "none"):
            desc = ""
        self.port_hint.configure(text=f"{dev}  {desc}" if desc else dev)

    def toggle_wifi(self):
        target = 0 if self.wifi_on else 1
        if self.demo:
            self.wifi_on = bool(target)
            self.wifi_info = {"state": str(target), "ssid": "ADC_Test",
                              "ip": "192.168.4.1", "tx": "0", "rx": "0"}
            self._update_wifi_button()
            self.status_var.set("演示模式：Wi-Fi 负载状态已切换")
            return
        if self.reader is None:
            messagebox.showinfo("未连接", "请先连接串口，再切换 Wi-Fi 负载")
            return
        if self.reader.send(f"W {target}"):
            self.status_var.set(f"已发送 W {target}，等待固件状态…")
        else:
            self.status_var.set("串口写入失败，无法切换 Wi-Fi")

    def _update_wifi_button(self):
        if not hasattr(self, "btn_wifi"):
            return
        if self.wifi_on:
            tx = self.wifi_info.get("tx", "?")
            rx = self.wifi_info.get("rx", "?")
            ip = self.wifi_info.get("ip", "")
            if ip in ("", "0.0.0.0"):
                ip = "等待 IP"
            self.btn_wifi.configure(text=f"Wi-Fi: 开  {ip}  tx={tx} rx={rx}")
            if self.wifi_info.get("ip") not in (None, "", "0.0.0.0"):
                if self.status_var.get().startswith("Wi-Fi"):
                    self.status_var.set(
                        f"Wi-Fi 已开：SSID={self.wifi_info.get('ssid', '?')}，"
                        f"IP={self.wifi_info.get('ip')}；"
                        f"运行 host/wifi_load.py（广播压测）")
        else:
            self.btn_wifi.configure(text="Wi-Fi 负载: 关")

    def toggle_connect(self):
        if self.demo:
            self.status_var.set("演示模式：无需连接串口")
            return
        if self.reader is not None:
            self.disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择或输入串口设备")
            return
        if serial is None:
            messagebox.showerror("缺少依赖", "未安装 pyserial，请先 pip install pyserial")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            baud = BAUD
        self.reader = SerialReader(port, baud, self.q)
        self.reader.start()
        self.port_name = port
        self.btn_connect.configure(text="断开")
        self.status_var.set(f"正在打开 {port} …")

    def disconnect(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.join(timeout=1.0)
            self.reader = None
        self.connected = False
        self.btn_connect.configure(text="连接")
        if not self.demo:
            self.status_var.set("已断开")

    # ------------------------------------------------------------------ 演示
    def _start_demo(self):
        self.demo_src = DemoSource(self.q)
        self.demo_src.start()
        self.connected = True
        self.port_name = "DEMO"
        self.btn_connect.configure(state="disabled")
        self.btn_refresh.configure(state="disabled")
        self.status_var.set("演示模式：模拟电位器来回扫动（不是真实硬件）")

    # ---------------------------------------------------------------- 数据解析
    def _poll_queue(self):
        if self._closing:
            return
        n = 0
        try:
            while n < 3000:
                kind, payload = self.q.get_nowait()
                n += 1
                if kind == "line":
                    self._handle_line(payload)
                elif kind == "open":
                    self.connected = True
                    self.status_var.set(f"已连接 {payload}，等待数据…")
                elif kind == "error":
                    self.status_var.set(payload)
                    messagebox.showerror("串口错误", payload)
                elif kind == "closed":
                    self.connected = False
                    self.burst_active = False
                    self.wifi_on = False
                    self._update_wifi_button()
                    if self.burst_win is not None and self.burst_win.winfo_exists():
                        self.burst_win.set_error("串口已断开")
                    if self.reader is not None:
                        self.reader = None
                    self.btn_connect.configure(text="连接")
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _handle_line(self, line: str):
        if not line:
            return

        # ---- 高速抓取（B 行） ----
        if line.startswith("#BURST_BEGIN"):
            p = line.split(",")
            try:
                self.burst_meta = {"n": int(p[1]),
                                   "chan": int(p[3]) if len(p) > 3 else -1,
                                   "atten": int(p[4]) if len(p) > 4 else -1}
            except (ValueError, IndexError):
                self.burst_meta = {}
            self.burst_samples = []
            self.burst_total_us = 0
            self.burst_active = True
            if self.burst_win is not None and self.burst_win.winfo_exists():
                self.burst_win.set_pending()
            self.status_var.set("正在高速抓取原始码…")
            return
        if line.startswith("B,"):
            if not self.burst_active:
                return
            p = line.split(",")
            if len(p) < 3:
                return
            try:
                self.burst_samples.append(int(p[2]))
            except ValueError:
                pass
            return
        if line.startswith("#BURST_END"):
            p = line.split(",")
            try:
                self.burst_total_us = int(p[2])
            except (ValueError, IndexError):
                self.burst_total_us = 0
            self._finish_burst()
            return

        # ---- 其余注释/状态行 ----
        if line.startswith("#"):
            if line.startswith("#CFG"):
                parts = line[5:].split(",")
                self.cfg = dict(p.split("=", 1) for p in parts if "=" in p)
            elif line.startswith("#ERR"):
                self.status_var.set("固件错误：" + line[1:])
            elif line.startswith("#HELP"):
                self.status_var.set(line[1:])
            elif line.startswith("#WIFI"):
                parts = line[6:].split(",")
                self.wifi_info = dict(p.split("=", 1) for p in parts if "=" in p)
                old = self.wifi_on
                self.wifi_on = self.wifi_info.get("state") == "1"
                self._update_wifi_button()
                if old != self.wifi_on:
                    if self.wifi_on:
                        self.status_var.set(
                            f"Wi-Fi 已开：SSID={self.wifi_info.get('ssid', '?')}，"
                            f"IP={self.wifi_info.get('ip', '?')}；连接后运行 host/wifi_load.py")
                    else:
                        self.status_var.set("Wi-Fi 已关")
            return

        s = parse_data_line(line)
        if s is None:
            return
        self.last = s
        self.rx_count += 1

        self.raw_hist.append(s.raw)
        self.cal_hist.append(float(s.cal_mv))
        self.lin_hist.append(s.lin_mv)
        self.std_hist.append(s.std)
        self.t_hist.append(s.t_ms / 1000.0)
        self.dirty = True

    # -------------------------------------------------------------- 噪声抓取
    def capture_burst(self, n: int = 4096):
        if self.burst_active:
            self.status_var.set("正在进行高速抓取，请稍候…")
            return
        if not self.connected:
            messagebox.showinfo("未连接", "请先连接串口（或使用 --demo）再抓取噪声。")
            return
        if self.burst_win is None or not self.burst_win.winfo_exists():
            self.burst_win = BurstWindow(self.root, self.capture_burst)
        self.burst_win.set_pending()
        self.burst_samples = []
        self.burst_total_us = 0
        self.burst_active = True
        self.status_var.set(f"发送 B {n}：高速抓取 {n} 点原始码…")
        if self.demo and self.demo_src is not None:
            self.demo_src.send(f"B {n}")
        elif self.reader is not None:
            if not self.reader.send(f"B {n}"):
                self.burst_active = False
                self.status_var.set("串口写入失败，无法抓取")
        else:
            self.burst_active = False
            self.status_var.set("串口未打开")

    def _finish_burst(self):
        self.burst_active = False
        if len(self.burst_samples) < 8:
            self.status_var.set("抓取失败：样本不足（固件版本需 ≥2）")
            if self.burst_win is not None and self.burst_win.winfo_exists():
                self.burst_win.set_error("样本不足，请确认固件为 v2 且支持 B 命令")
            return
        summary = burst_summary(self.burst_samples, self.burst_total_us)
        summary["chan"] = self.burst_meta.get("chan")
        summary["atten"] = self.burst_meta.get("atten")
        self.burst_last = summary
        if self.burst_win is None or not self.burst_win.winfo_exists():
            self.burst_win = BurstWindow(self.root, self.capture_burst)
        self.burst_win.set_data(self.burst_samples, self.burst_total_us,
                                self.burst_meta)
        peak = summary.get("f_peak")
        self.status_var.set(
            f"抓取完成：{summary['n']} 点 @ {summary['fs']:.0f} Hz，"
            f"峰峰 {summary['pp_mv']:.1f} mV，σ {summary['std_mv']:.2f} mV，"
            + (f"主频 {peak:.1f} Hz" if peak else "主频 —"))

    # ---------------------------------------------------------------- 定时刷新
    def _periodic_redraw(self):
        if self._closing:
            return
        if self.last is not None:
            s = self.last
            self.v_raw.set(f"{s.raw:.2f}")
            if s.cal_mv >= 0:
                self.v_cal.set(f"{s.cal_mv} mV")
                self.v_diff.set(f"{s.cal_mv - s.lin_mv:+.1f} mV")
            else:
                self.v_cal.set("不可用")
                self.v_diff.set("—")
            self.v_lin.set(f"{s.lin_mv:.1f} mV")
            pp = s.raw_max - s.raw_min
            noise_mv = pp * VREF_MV / ADC_FULL_SCALE
            self.v_noise.set(f"{pp} 码 / {noise_mv:.0f} mV  σ{s.std:.1f}")
            self._update_rate()
            self.v_rate.set(f"{self.rate:.1f} Hz")
        if self.dirty:
            self._redraw_curve()
            self.dirty = False
        self._tick += 1
        if self.last is not None and self._tick % 5 == 0:
            self._update_stats_panel()
        self.root.after(200, self._periodic_redraw)

    def _update_rate(self):
        """用固件时间戳算采样率，避免把一次读到的积压行误算成超高频率。"""
        n = min(len(self.t_hist), 50)
        if n < 2:
            return
        arr = list(self.t_hist)
        dt = arr[-1] - arr[-1 - (n - 1)]
        if dt > 0:
            self.rate = (n - 1) / dt

    def _redraw_curve(self):
        if not self.raw_hist:
            return
        step = max(1, len(self.raw_hist) // 2500)
        r = np.fromiter(self.raw_hist, float)[::step]
        c = np.fromiter(self.cal_hist, float)[::step]
        l = np.fromiter(self.lin_hist, float)[::step]
        mask = c >= 0
        self.ln_cal.set_data(r[mask], c[mask])
        self.ln_lin.set_data(r, l)
        self.canvas.draw_idle()

    # ---------------------------------------------------------------- 标定点
    def _read_true_mv(self) -> float | None:
        text = self.mv_var.get().strip()
        if not text:
            messagebox.showinfo("需要真值", "请先输入万用表测得的电压（例如 1.234 或 1234）")
            return None
        try:
            return parse_voltage_mv(text)
        except ValueError:
            messagebox.showerror("输入错误", "电压格式无效，请输入如 1.234、1234、1.234V 或 1234mV")
            return None

    def _avg_n(self) -> int:
        try:
            return max(1, min(500, int(float(self.avg_var.get()))))
        except (ValueError, AttributeError):
            return 1

    def _stable_limit_mv(self) -> float:
        try:
            return max(0.0, float(self.stable_mv_var.get()))
        except (ValueError, AttributeError):
            return 50.0

    def record_point(self):
        # 等待稳定期间再次按 F2/点击 = 立即记录，不再等
        forced = self._stable_job is not None
        if forced:
            self.root.after_cancel(self._stable_job)
            self._stable_job = None
        if self.last is None:
            self.status_var.set("还没有收到数据，无法记录")
            return
        if self.last.cal_mv < 0:
            self.status_var.set("固件未提供校准值，无法记录")
            return
        true_mv = self._read_true_mv()
        if true_mv is None:
            return
        # 真值与当前 ADC 换算明显不符时确认，避免输错单位/电位器位置
        lin_now = self.last.lin_mv
        if abs(lin_now - true_mv) > 1000:
            if not messagebox.askyesno(
                    "真值可能不对",
                    f"当前线性换算约 {lin_now:.0f} mV，输入真值 {true_mv:.0f} mV，"
                    "相差超过 1000 mV。\n"
                    "可能输错单位（3.3 表示 3.3 V，3300 表示 3300 mV），"
                    "或电位器不在该电压。\n仍要记录吗？"):
                return
        if self.last.raw >= 4090 and true_mv < 2800:
            if not messagebox.askyesno(
                    "ADC 已饱和",
                    f"当前 raw={self.last.raw:.0f}（≈{self.last.cal_mv} mV），"
                    f"但输入真值只有 {true_mv:.0f} mV。\n"
                    "12 dB 档在约 2.89 V 饱和，确认要记录这个点吗？"):
                return
        if forced:
            self._commit_point(true_mv, note="已立即记录（未等待稳定）")
        elif self.stable_var.get():
            self._record_when_stable(true_mv)
        else:
            self._commit_point(true_mv)

    def _record_when_stable(self, true_mv: float):
        """等待最近窗口峰峰值低于阈值并连续保持 3 次后再记录；超时仍记录。"""
        limit = self._stable_limit_mv()
        deadline = time.monotonic() + 4.0
        stable_hits = 0

        def check():
            nonlocal stable_hits
            self._stable_job = None
            if self._closing:
                return
            if self.last is None:
                self.status_var.set("没有数据，无法记录")
                return
            if time.monotonic() > deadline:
                self._commit_point(true_mv, note="稳定超时，已按当前值记录")
                return
            pp = self.last.noise_mv
            stable_hits = stable_hits + 1 if pp <= limit else 0
            if stable_hits >= 3:
                self._commit_point(true_mv)
                return
            self.status_var.set(
                f"等待稳定… 峰峰 {pp:.1f} mV（阈值 {limit:.1f} mV），"
                f"已连续 {stable_hits}/3；再按 F2 立即记录")
            self._stable_job = self.root.after(100, check)

        self.status_var.set(f"等待稳定… 阈值 {limit:.1f} mV")
        check()

    def _commit_point(self, true_mv: float, note: str = ""):
        n = self._avg_n()
        raws = list(self.raw_hist)[-n:]
        cals = list(self.cal_hist)[-n:]
        lins = list(self.lin_hist)[-n:]
        stds = list(self.std_hist)[-n:]
        triples = [(r, c, l) for r, c, l in zip(raws, cals, lins) if c >= 0]
        if not triples:
            self.status_var.set("没有可用的校准数据，无法记录")
            return
        k = len(triples)
        raw = sum(t[0] for t in triples) / k
        cal = sum(t[1] for t in triples) / k
        lin = sum(t[2] for t in triples) / k
        std = sum(stds) / len(stds) if stds else 0.0
        if self.demo:
            # 演示模式下用与仿真一致的“真值”，避免手工输入与扫动不同步导致荒谬误差
            true_mv = DemoSource.true_mv(raw)
        pt = CalPoint(self.last.t_ms if self.last else 0, raw, true_mv, cal, lin,
                      raw_std=std, n_avg=k)
        self.points.append(pt)
        self._points_version += 1
        self._autosave()
        self._refresh_tree()
        self._refresh_points_plot()
        self._update_stats_panel()
        msg = (f"已记录 #{len(self.points)}（{k} 点平均）：raw={pt.raw:.2f}，"
               f"真值={true_mv:.1f} mV，校准误差 {pt.cal_err:+.1f} mV，"
               f"线性误差 {pt.lin_err:+.1f} mV")
        self.status_var.set((note + "；" + msg) if note else msg)

    # ------------------------------------------------------------ 自动保存
    def _autosave(self):
        if self.demo:
            return
        try:
            if not self.points:
                if self.autosave_path.exists():
                    self.autosave_path.unlink()
                return
            self.autosave_path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "version": 1,
                "saved_at": datetime.now().isoformat(timespec="seconds"),
                "cfg": self.cfg,
                "vref_mv": VREF_MV,
                "full_scale": ADC_FULL_SCALE,
                "points": [asdict(p) for p in self.points],
                "burst": self.burst_last,
            }
            tmp = self.autosave_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2),
                           encoding="utf-8")
            tmp.replace(self.autosave_path)
        except Exception:  # noqa: BLE001
            pass

    def _maybe_restore_autosave(self):
        if self.demo or self._closing:
            return
        try:
            if not self.autosave_path.exists():
                return
            data = json.loads(self.autosave_path.read_text(encoding="utf-8"))
            pts = [CalPoint(**p) for p in data.get("points", [])]
        except Exception:  # noqa: BLE001
            return
        if not pts:
            return
        if not messagebox.askyesno(
                "恢复自动保存",
                f"发现 {len(pts)} 个未导出的标定点（{data.get('saved_at', '?')}）。\n"
                "是否恢复？选“否”会保留文件，下次启动再问。"):
            return
        self.points = pts
        self.cfg = data.get("cfg", {})
        self.burst_last = data.get("burst")
        self._points_version += 1
        self._autosave()
        self._refresh_tree()
        self._refresh_points_plot()
        self._update_stats_panel()
        self.status_var.set(f"已恢复自动保存的 {len(pts)} 个点")

    def _refresh_tree(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for i, pt in enumerate(self.points, 1):
            self.tree.insert("", "end", iid=str(i), values=(
                f"{pt.raw:.1f}", f"{pt.true_mv:.1f}", f"{pt.cal_mv:.0f}",
                f"{pt.lin_mv:.1f}", f"{pt.cal_err:+.1f}", f"{pt.lin_err:+.1f}"))

    def delete_selected(self):
        sel = self.tree.selection()
        if not sel:
            return
        idxs = sorted((int(x) - 1 for x in sel), reverse=True)
        for i in idxs:
            del self.points[i]
        self._points_version += 1
        self._autosave()
        self._refresh_tree()
        self._refresh_points_plot()
        self._update_stats_panel()

    def clear_points(self):
        if not self.points:
            return
        if messagebox.askyesno("确认", f"清空全部 {len(self.points)} 个标定点？"):
            self.points.clear()
            self._points_version += 1
            self._autosave()
            self._refresh_tree()
            self._refresh_points_plot()
            self._update_stats_panel()

    def clear_curve(self):
        self.raw_hist.clear()
        self.cal_hist.clear()
        self.lin_hist.clear()
        self.std_hist.clear()
        self.t_hist.clear()
        self.ln_cal.set_data([], [])
        self.ln_lin.set_data([], [])
        self.canvas.draw_idle()
        self.status_var.set("实时曲线已清空")

    # ---------------------------------------------------------------- 统计
    def _refresh_points_plot(self):
        if not self.points:
            self.rec_cal.set_data([], [])
            self.rec_lin.set_data([], [])
            self.ax2.set_ylim(-60, 60)
        else:
            raw = np.array([p.raw for p in self.points])
            self.rec_cal.set_data(raw, [p.cal_err for p in self.points])
            self.rec_lin.set_data(raw, [p.lin_err for p in self.points])
            errs = np.array([p.cal_err for p in self.points] + [p.lin_err for p in self.points])
            span = max(10.0, float(np.max(np.abs(errs))) * 1.4)
            self.ax2.set_ylim(-span, span)
        self.canvas.draw_idle()

    def _update_stats_panel(self):
        lines = []
        if self.cal_hist and self.lin_hist:
            c = np.fromiter(self.cal_hist, float)
            l = np.fromiter(self.lin_hist, float)
            d = c - l
            lines.append(f"实时 校准−线性：均值 {d.mean():+.1f} mV，"
                         f"范围 {d.min():+.1f} ~ {d.max():+.1f} mV")

        if not self.points:
            lines.append("")
            lines.append("尚无标定点。旋转电位器到位后，输入万用表读数并点“记录当前点”。")
            lines.append("至少记录 3 个点才能给出拟合斜率与 INL。")
        else:
            lines.append(f"标定点数：{len(self.points)}")
            for key, name in (("cal", "ESP-IDF 校准"), ("lin", "原始码线性")):
                if key == "cal":
                    vals = np.array([p.cal_mv for p in self.points])
                else:
                    vals = np.array([p.lin_mv for p in self.points])
                true = np.array([p.true_mv for p in self.points])
                err = vals - true
                txt = (f"[{name}] max|e| {np.max(np.abs(err)):.1f}  "
                       f"RMS {np.sqrt(np.mean(err ** 2)):.1f}  "
                       f"均值 {err.mean():+.1f} mV")
                if len(self.points) >= 3:
                    a, b = np.polyfit(true, vals, 1)
                    inl = vals - (a * true + b)
                    txt += (f"\n    斜率 {a:.4f}  截距 {b:+.1f} mV  "
                            f"INL {np.max(np.abs(inl)):.1f} mV")
                lines.append(txt)

        self.stats_text.configure(state="normal")
        self.stats_text.delete("1.0", "end")
        self.stats_text.insert("1.0", "\n".join(lines))
        self.stats_text.configure(state="disabled")
        if self._points_version != self._points_drawn_version:
            self._refresh_points_plot()
            self._points_drawn_version = self._points_version

    # ---------------------------------------------------------------- 导出
    def export_csv(self):
        if not self.points:
            messagebox.showinfo("没有数据", "还没有记录任何标定点")
            return
        default = f"adc_points_{datetime.now():%Y%m%d_%H%M%S}.csv"
        path = filedialog.asksaveasfilename(
            title="导出标定点", defaultextension=".csv", initialfile=default,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        write_points_csv(self.points, path, cfg=self.cfg, vref_mv=VREF_MV,
                         full_scale=ADC_FULL_SCALE, burst=self.burst_last)
        self.status_var.set(f"已导出 {len(self.points)} 个点到 {Path(path).name}")

    def save_session(self):
        if not self.points:
            messagebox.showinfo("没有数据", "还没有记录任何标定点")
            return
        default = f"adc_session_{datetime.now():%Y%m%d_%H%M%S}.json"
        path = filedialog.asksaveasfilename(
            title="保存会话", defaultextension=".json", initialfile=default,
            filetypes=[("JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        data = {
            "version": 1,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "cfg": self.cfg,
            "vref_mv": VREF_MV,
            "full_scale": ADC_FULL_SCALE,
            "points": [asdict(p) for p in self.points],
            "burst": self.burst_last,
        }
        Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2),
                              encoding="utf-8")
        self.status_var.set(f"会话已保存：{Path(path).name}")

    def load_session(self):
        path = filedialog.askopenfilename(
            title="载入会话", filetypes=[("JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            pts = [CalPoint(**p) for p in data["points"]]
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("载入失败", f"文件格式不正确：{exc}")
            return
        if not pts:
            messagebox.showinfo("空会话", "该会话没有标定点")
            return
        if self.points and not messagebox.askyesno(
                "确认", f"用 {len(pts)} 个点替换当前 {len(self.points)} 个点？"):
            return
        self.points = pts
        self.cfg = data.get("cfg", {})
        self.burst_last = data.get("burst")
        self._points_version += 1
        self._autosave()
        self._refresh_tree()
        self._refresh_points_plot()
        self._update_stats_panel()
        self.status_var.set(f"已载入 {len(pts)} 个点：{Path(path).name}")

    def export_report(self):
        if not self.points:
            messagebox.showinfo("没有数据", "还没有记录任何标定点")
            return
        default = f"adc_report_{datetime.now():%Y%m%d_%H%M%S}.md"
        path = filedialog.asksaveasfilename(
            title="导出报告", defaultextension=".md", initialfile=default,
            filetypes=[("Markdown", "*.md"), ("All files", "*.*")])
        if not path:
            return
        out = Path(path)
        md = build_report_markdown(self.points, self.cfg, vref_mv=VREF_MV,
                                   full_scale=ADC_FULL_SCALE, burst=self.burst_last)
        out.write_text(md, encoding="utf-8")
        png = out.with_name(out.stem + "_plot.png")
        try:
            self._make_report_png(png)
            extra = f"，配图 {png.name}"
        except Exception as exc:  # noqa: BLE001
            extra = f"（配图失败：{exc}）"
        self.status_var.set(f"报告已导出：{out.name}{extra}")

    def _make_report_png(self, path: Path):
        fig = Figure(figsize=(8, 8), dpi=100, constrained_layout=True)
        ax1, ax2 = fig.subplots(2, 1)
        sat = [p for p in self.points if p.raw >= ADC_FULL_SCALE - 5]
        if sat:
            x0 = min(p.true_mv for p in sat)
            x1 = max(p.true_mv for p in sat)
            for a in (ax1, ax2):
                a.axvspan(x0, x1, color="#d62728", alpha=0.08, zorder=0)
            ax1.text((x0 + x1) / 2, 0.92, "饱和区", ha="center", va="top",
                     color="#d62728", fontsize=9,
                     transform=ax1.get_xaxis_transform())
        true = [p.true_mv for p in self.points]
        cal = [p.cal_mv for p in self.points]
        lin = [p.lin_mv for p in self.points]
        ax1.plot(true, cal, "o-", color=COL_CAL, ms=4, label="ESP-IDF 校准")
        ax1.plot(true, lin, "s--", color=COL_LIN, ms=4, mfc="none", label="原始码线性")
        hi = max(true) if true else 1
        ax1.plot([0, hi], [0, hi], color="#999999", lw=0.8, label="理想 1:1")
        ax1.set_xlabel("万用表真值 / mV")
        ax1.set_ylabel("测量值 / mV")
        ax1.set_title("测量值 vs 真值")
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc="upper left", fontsize=9)

        ax2.axhline(0, color="#999999", lw=0.8)
        ax2.plot(true, [p.cal_err for p in self.points], "o-", color=COL_CAL,
                 ms=4, label="ESP-IDF 校准")
        ax2.plot(true, [p.lin_err for p in self.points], "s--", color=COL_LIN,
                 ms=4, mfc="none", label="原始码线性")
        ax2.set_xlabel("万用表真值 / mV")
        ax2.set_ylabel("误差 / mV")
        ax2.set_title("误差（测量 − 真值）")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="upper right", fontsize=9)
        fig.savefig(path, dpi=130)

    # ---------------------------------------------------------------- 关闭
    def on_close(self):
        self._closing = True
        if self._stable_job is not None:
            try:
                self.root.after_cancel(self._stable_job)
            except tk.TclError:
                pass
            self._stable_job = None
        if self.burst_win is not None and self.burst_win.winfo_exists():
            self.burst_win.destroy()
            self.burst_win = None
        if self.demo_src is not None:
            self.demo_src.stop()
            self.demo_src = None
        if self.reader is not None:
            self.reader.stop()
            self.reader.join(timeout=0.8)
            self.reader = None
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description="ESP32-C3 ADC 线性度采集 GUI")
    ap.add_argument("--port", help="串口设备，如 /dev/ttyACM0")
    ap.add_argument("--demo", action="store_true", help="无硬件演示模式")
    args = ap.parse_args()

    setup_cjk_fonts()
    root = tk.Tk()
    AdcApp(root, demo=args.demo, port=args.port)
    root.mainloop()


if __name__ == "__main__":
    main()
