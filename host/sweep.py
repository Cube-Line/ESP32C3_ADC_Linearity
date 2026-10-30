#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""命令行标定向导：逐点读取万用表真值并记录。

用法：
    python host/sweep.py --port /dev/ttyACM0
    python host/sweep.py --port /dev/ttyACM0 --out data/points.csv --n 20

交互：
    输入万用表电压（1.234 表示 V，1234 表示 mV）并回车 → 等待稳定后记录
    直接回车        → 刷新当前读数
    u               → 撤销上一个点
    q               → 结束并生成 CSV + Markdown 报告

不依赖 tkinter，适合 SSH/无桌面环境；GUI 仍可用于实时曲线与频谱。
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

sys.path.insert(0, str(Path(__file__).resolve().parent))

from adc_core import (ADC_FULL_SCALE, VREF_MV, CalPoint, Sample,
                      build_report_markdown, compare_methods, parse_data_line,
                      parse_voltage_mv, write_points_csv)


class Sampler(threading.Thread):
    def __init__(self, port: str, baud: int = 115200):
        super().__init__(daemon=True, name="adc-sampler")
        self.port = port
        self.baud = baud
        self.samples: deque[Sample] = deque(maxlen=4000)
        self.cfg: dict[str, str] = {}
        self.stop_evt = threading.Event()
        self.ser = serial.Serial(port, baud, timeout=0.2)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:  # noqa: BLE001
            pass

    def run(self):
        buf = b""
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception:  # noqa: BLE001
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                if line.startswith("#CFG"):
                    parts = line[5:].split(",")
                    self.cfg = dict(p.split("=", 1) for p in parts if "=" in p)
                    continue
                s = parse_data_line(line)
                if s is not None:
                    self.samples.append(s)
            if len(buf) > 65536:
                buf = b""

    def send(self, cmd: str) -> None:
        self.ser.write((cmd.strip() + "\n").encode("ascii"))

    def recent(self, n: int) -> list[Sample]:
        return list(self.samples)[-n:]

    @property
    def last(self) -> Sample | None:
        return self.samples[-1] if self.samples else None

    def close(self):
        self.stop_evt.set()
        self.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass


def show_current(reader: Sampler) -> bool:
    s = reader.last
    if s is None:
        print("  还没有收到 D 行，检查串口/固件")
        return False
    print(f"  当前 raw={s.raw:.2f}  cal={s.cal_mv} mV  lin={s.lin_mv:.1f} mV  "
          f"峰峰={s.pp_code} 码/{s.noise_mv:.1f} mV  σ={s.std:.2f} 码")
    return s.cal_mv >= 0


def average(reader: Sampler, n: int, stable_mv: float, timeout: float) -> CalPoint | None:
    """等待稳定（连续 3 次峰峰 ≤ 阈值）后取最近 n 点平均。"""
    n = max(1, n)
    deadline = time.monotonic() + timeout
    hits = 0
    while time.monotonic() < deadline:
        recent = reader.recent(n)
        if len(recent) < min(n, 3):
            time.sleep(0.1)
            continue
        s = recent[-1]
        hits = hits + 1 if s.noise_mv <= stable_mv else 0
        if hits >= 3:
            break
        sys.stdout.write(f"\r  等待稳定… 峰峰 {s.noise_mv:.1f} mV / 阈值 {stable_mv:.1f} mV")
        sys.stdout.flush()
        time.sleep(0.1)
    else:
        print("\n  稳定超时，仍按当前最近点记录（可撤销）")
    recent = [x for x in reader.recent(n) if x.cal_mv >= 0]
    if not recent:
        return None
    k = len(recent)
    return CalPoint(
        t_ms=recent[-1].t_ms,
        raw=sum(x.raw for x in recent) / k,
        true_mv=0.0,  # 由调用者填写
        cal_mv=sum(x.cal_mv for x in recent) / k,
        lin_mv=sum(x.lin_mv for x in recent) / k,
        raw_std=sum(x.std for x in recent) / k,
        n_avg=k,
    )


def print_stats(points: list[CalPoint]):
    comp = compare_methods(points)
    cal, lin = comp["cal"], comp["lin"]
    print(f"\n  点数 {len(points)} | 校准 max|e| {cal['max_abs_err']:.2f} mV  "
          f"RMS {cal['rms_err']:.2f} | 线性 max|e| {lin['max_abs_err']:.2f} mV  "
          f"RMS {lin['rms_err']:.2f}")
    if cal["gain"] is not None:
        print(f"  校准斜率 {cal['gain']:.4f} 截距 {cal['offset']:+.1f} mV | "
              f"线性斜率 {lin['gain']:.4f} 截距 {lin['offset']:+.1f} mV")


def main() -> int:
    ap = argparse.ArgumentParser(description="ESP32-C3 ADC 标定向导")
    ap.add_argument("--port", help="串口，默认自动找 ttyACM/ttyUSB")
    ap.add_argument("--out", help="输出 CSV（默认 data/points_时间.csv）")
    ap.add_argument("--n", type=int, default=20, help="每点平均的 D 行数（默认 20 = 1s）")
    ap.add_argument("--stable-mv", type=float, default=50.0, help="稳定阈值（峰峰 mV，默认 50；加 100 nF 后可降到 5~10）")
    ap.add_argument("--timeout", type=float, default=8.0, help="等待稳定超时秒数")
    args = ap.parse_args()

    if serial is None:
        print("未安装 pyserial：.venv/bin/pip install pyserial", file=sys.stderr)
        return 2
    port = args.port
    if not port and list_ports:
        cands = [p.device for p in list_ports.comports()
                 if "ttyACM" in p.device or "ttyUSB" in p.device]
        port = cands[0] if cands else None
    if not port:
        print("找不到串口，请用 --port 指定", file=sys.stderr)
        return 2

    out = Path(args.out) if args.out else Path("data") / f"points_{datetime.now():%Y%m%d_%H%M%S}.csv"
    out.parent.mkdir(parents=True, exist_ok=True)

    try:
        reader = Sampler(port)
    except Exception as exc:  # noqa: BLE001
        print(f"打开 {port} 失败：{exc}", file=sys.stderr)
        return 2
    reader.start()
    time.sleep(0.5)
    if reader.last is None:
        print("等待数据…（确认固件已烧录且未被其他程序占用）")
        for _ in range(30):
            time.sleep(0.2)
            if reader.last is not None:
                break
    if reader.last is None:
        print("没有收到 D 行，退出", file=sys.stderr)
        reader.close()
        return 2

    print(f"已连接 {port}")
    if reader.cfg:
        print("配置：" + "，".join(f"{k}={v}" for k, v in reader.cfg.items()))
    print("输入万用表电压（V 或 mV）回车记录；空回车刷新；u 撤销；q 结束并生成报告。")

    points: list[CalPoint] = []
    while True:
        if not show_current(reader):
            print("  固件未提供校准值，无法记录")
        try:
            text = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if text.lower() in ("q", "quit", "exit"):
            break
        if text.lower() == "u":
            if points:
                points.pop()
                print("  已撤销上一个点")
                write_points_csv(points, out, cfg=reader.cfg, burst=None)
                print_stats(points)
            continue
        if not text:
            continue
        try:
            true_mv = parse_voltage_mv(text)
        except ValueError:
            print("  格式无效：请输入 1.234（V）或 1234（mV）")
            continue

        pt = average(reader, args.n, args.stable_mv, args.timeout)
        if pt is None:
            print("  没有可用的校准数据")
            continue
        pt.true_mv = true_mv
        points.append(pt)
        write_points_csv(points, out, cfg=reader.cfg, burst=None)
        print(f"\r  已记录 #{len(points)}：raw={pt.raw:.2f}  {pt.n_avg} 点平均  "
              f"真值={true_mv:.1f} mV  校准误差 {pt.cal_err:+.1f} mV  "
              f"线性误差 {pt.lin_err:+.1f} mV")
        print_stats(points)
        print(f"  已保存 {out}")

    reader.close()
    if not points:
        print("没有记录任何点")
        return 0
    write_points_csv(points, out, cfg=reader.cfg, burst=None)
    md = out.with_name(out.stem + "_report.md")
    md.write_text(build_report_markdown(points, reader.cfg), encoding="utf-8")
    print(f"\nCSV：{out}\n报告：{md}")
    try:
        from analyze import make_plot
        png = out.with_name(out.stem + "_report.png")
        if make_plot(points, png, reader.cfg):
            print(f"配图：{png}")
    except Exception as exc:  # noqa: BLE001
        print(f"（未生成配图：{exc}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
