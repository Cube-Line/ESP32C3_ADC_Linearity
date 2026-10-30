#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ESP32-C3 ADC 线性度实验的纯数据/算法层。

不依赖 tkinter / matplotlib，便于单测与离线分析：
  * 串口数据行解析
  * 万用表电压输入解析
  * 两种换算方式（ESP-IDF 校准 / 原始码线性）的误差、拟合、INL
  * 标定点 CSV 读写
  * 高速抓取（B 行）的频谱分析
  * Markdown 报告生成
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence

VREF_MV = 3300.0          # “直接使用”假设的满量程
ADC_FULL_SCALE = 4095.0


def setup_matplotlib_fonts() -> str | None:
    """给 matplotlib 选一个可用的中文字体（无则回退 DejaVu Sans）。"""
    import matplotlib
    from matplotlib import font_manager

    installed = {f.name for f in font_manager.fontManager.ttflist}
    candidates = [
        "Sarasa Gothic SC", "Sarasa Mono SC", "WenQuanYi Zen Hei",
        "Noto Sans CJK SC", "Noto Serif CJK SC", "Source Han Sans SC",
        "Microsoft YaHei", "SimHei", "AR PL UMing CN",
        "AR PL UMing TW MBE", "Droid Sans Fallback",
    ]
    for name in candidates:
        if name in installed:
            matplotlib.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            matplotlib.rcParams["axes.unicode_minus"] = False
            return name
    return None


# ----------------------------------------------------------------------------
# 数据结构
# ----------------------------------------------------------------------------
@dataclass(slots=True)
class Sample:
    t_ms: int
    raw_x100: int
    raw_min: int
    raw_max: int
    cal_mv: int
    cal_min_mv: int
    cal_max_mv: int
    raw_std_x100: int = 0

    @property
    def raw(self) -> float:
        return self.raw_x100 / 100.0

    @property
    def std(self) -> float:
        """64 点过采样窗口内的标准差（原始码）。"""
        return self.raw_std_x100 / 100.0

    @property
    def pp_code(self) -> int:
        return self.raw_max - self.raw_min

    @property
    def lin_mv(self) -> float:
        return self.raw * VREF_MV / ADC_FULL_SCALE

    @property
    def noise_mv(self) -> float:
        return self.pp_code * VREF_MV / ADC_FULL_SCALE


@dataclass(slots=True)
class CalPoint:
    t_ms: int
    raw: float
    true_mv: float
    cal_mv: float
    lin_mv: float
    raw_std: float = 0.0
    n_avg: int = 1

    @property
    def cal_err(self) -> float:
        return self.cal_mv - self.true_mv

    @property
    def lin_err(self) -> float:
        return self.lin_mv - self.true_mv


# ----------------------------------------------------------------------------
# 解析
# ----------------------------------------------------------------------------
def parse_voltage_mv(text: str) -> float:
    """接受 '3.301'（V）、'3301'（mV）、'3.301V'、'3301mV'。

    规则：带 V/mV 后缀按后缀；不带后缀时，数值 < 5 视为 V，否则视为 mV。
    （本实验输入上限 3.3 V，因此 5 以上不可能是 V。）
    """
    s = text.strip().lower()
    if not s:
        raise ValueError("空输入")
    if s.endswith("mv"):
        return float(s[:-2])
    if s.endswith("v"):
        return float(s[:-1]) * 1000.0
    val = float(s)
    if not math.isfinite(val):
        raise ValueError("不是有限数值")
    return val * 1000.0 if abs(val) < 5.0 else val


def parse_data_line(line: str) -> Sample | None:
    """解析固件 D 行，兼容 8 字段（v1）与 9 字段（v2，带 σ）。"""
    line = line.strip()
    if not line.startswith("D,"):
        return None
    p = line.split(",")
    if len(p) < 8:
        return None
    try:
        std = int(p[8]) if len(p) > 8 and p[8] != "" else 0
        return Sample(int(p[1]), int(p[2]), int(p[3]), int(p[4]),
                      int(p[5]), int(p[6]), int(p[7]), std)
    except ValueError:
        return None


# ----------------------------------------------------------------------------
# 统计
# ----------------------------------------------------------------------------
def fit_line(x: Sequence[float], y: Sequence[float]) -> tuple[float, float] | None:
    """最小二乘 y = a*x + b；点数 < 2 或 x 全相同时返回 None。"""
    n = len(x)
    if n < 2:
        return None
    sx = math.fsum(x)
    sy = math.fsum(y)
    sxx = math.fsum(v * v for v in x)
    sxy = math.fsum(a * b for a, b in zip(x, y))
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    return a, b


def method_stats(points: Sequence[CalPoint], method: str) -> dict:
    """method 取 'cal' 或 'lin'，返回误差/拟合/INL 统计。"""
    if method not in ("cal", "lin"):
        raise ValueError("method must be 'cal' or 'lin'")
    vals = [p.cal_mv if method == "cal" else p.lin_mv for p in points]
    true = [p.true_mv for p in points]
    n = len(points)
    out: dict = {"method": method, "n": n}
    if n == 0:
        out.update(max_abs_err=None, rms_err=None, mean_err=None,
                   gain=None, offset=None, inl=None, inl_pct=None,
                   span_mv=None, worst=None)
        return out

    err = [v - t for v, t in zip(vals, true)]
    abs_err = [abs(e) for e in err]
    worst_i = max(range(n), key=lambda i: abs_err[i])
    out.update(
        max_abs_err=max(abs_err),
        rms_err=math.sqrt(math.fsum(e * e for e in err) / n),
        mean_err=math.fsum(err) / n,
        span_mv=max(true) - min(true),
        worst=(points[worst_i], err[worst_i]),
    )

    fit = fit_line(true, vals)
    if fit is None:
        out.update(gain=None, offset=None, inl=None, inl_pct=None)
    else:
        a, b = fit
        resid = [v - (a * t + b) for v, t in zip(vals, true)]
        inl = max(abs(r) for r in resid)
        out.update(gain=a, offset=b, inl=inl,
                   inl_pct=(inl / out["span_mv"] * 100.0) if out["span_mv"] > 0 else None)
    return out


def compare_methods(points: Sequence[CalPoint]) -> dict:
    cal = method_stats(points, "cal")
    lin = method_stats(points, "lin")
    winner = None
    if cal["max_abs_err"] is not None and lin["max_abs_err"] is not None:
        winner = "cal" if cal["max_abs_err"] <= lin["max_abs_err"] else "lin"
    return {"cal": cal, "lin": lin, "winner": winner}


# ----------------------------------------------------------------------------
# 频谱（依赖 numpy，延迟导入）
# ----------------------------------------------------------------------------
def compute_spectrum(samples: Sequence[float], fs: float) -> dict:
    """对高速抓取的原始码做去均值 + Hann 窗 + 单边幅度谱（单位：mV）。

    幅度谱用于观察频率分布；单频点的准确幅度请用 ``sine_amp_mv``。
    """
    import numpy as np

    x = np.asarray(samples, dtype=float)
    if x.size < 8 or fs <= 0:
        raise ValueError("样本太少或采样率无效")
    x = x - x.mean()
    n = x.size
    w = np.hanning(n)
    spec = np.fft.rfft(x * w)
    mag = np.abs(spec) * 2.0 / w.sum()
    mag_mv = mag * VREF_MV / ADC_FULL_SCALE
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    return {"freqs": freqs, "mag_mv": mag_mv, "fs": fs, "n": n}


def sine_amp_mv(samples: Sequence[float], fs: float, f: float) -> float:
    """最小二乘拟合指定频率正弦的幅度（mV），不受 FFT 栅栏效应影响。"""
    import numpy as np

    x = np.asarray(samples, dtype=float)
    n = x.size
    if n < 8 or fs <= 0:
        return 0.0
    t = np.arange(n) / fs
    w = 2.0 * np.pi * f * t
    m = np.column_stack((np.sin(w), np.cos(w), np.ones_like(t)))
    try:
        coef, *_ = np.linalg.lstsq(m, x, rcond=None)
    except np.linalg.LinAlgError:
        return 0.0
    return float(np.hypot(coef[0], coef[1]) * VREF_MV / ADC_FULL_SCALE)



def burst_summary(samples: Sequence[int], total_us: float) -> dict:
    """高速抓取统计：采样率、均值、峰峰值、σ、主频、50/100 Hz 幅度。"""
    import numpy as np

    x = np.asarray(samples, dtype=float)
    n = x.size
    fs = n / (total_us / 1e6) if total_us > 0 else 0.0
    out = {
        "n": n,
        "fs": fs,
        "mean_code": float(x.mean()) if n else 0.0,
        "pp_code": float(x.max() - x.min()) if n else 0.0,
        "std_code": float(x.std(ddof=1)) if n > 1 else 0.0,
        "f_peak": None,
        "amp_peak_mv": None,
        "amp_50_mv": None,
        "amp_100_mv": None,
        "bin_hz": (fs / n) if n and fs else None,
    }
    if n >= 8 and fs > 0:
        sp = compute_spectrum(x, fs)
        freqs, mag = sp["freqs"], sp["mag_mv"]
        # 忽略直流附近（前 3 个 bin）
        lo = 3 if len(mag) > 3 else 0
        if len(mag) > lo:
            k = lo + int(np.argmax(mag[lo:]))
            delta = 0.0
            if 0 < k < len(mag) - 1:
                y0, y1, y2 = mag[k - 1], mag[k], mag[k + 1]
                den = y0 - 2.0 * y1 + y2
                if abs(den) > 1e-12:
                    delta = 0.5 * (y0 - y2) / den
                    delta = max(-0.5, min(0.5, delta))
            f_peak = (k + delta) * fs / n
            out["f_peak"] = float(f_peak)
            out["amp_peak_mv"] = sine_amp_mv(x, fs, f_peak)
        out["amp_50_mv"] = sine_amp_mv(x, fs, 50.0)
        out["amp_100_mv"] = sine_amp_mv(x, fs, 100.0)
    out["pp_mv"] = out["pp_code"] * VREF_MV / ADC_FULL_SCALE
    out["std_mv"] = out["std_code"] * VREF_MV / ADC_FULL_SCALE
    return out


# ----------------------------------------------------------------------------
# CSV
# ----------------------------------------------------------------------------
POINT_HEADER = ["index", "t_ms", "raw_code", "raw_std", "true_mv", "cal_mv",
                "lin_mv", "cal_err_mv", "lin_err_mv", "n_avg"]


def write_points_csv(points: Sequence[CalPoint], path: str | Path,
                     cfg: dict | None = None, vref_mv: float = VREF_MV,
                     full_scale: float = ADC_FULL_SCALE,
                     burst: dict | None = None) -> None:
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(POINT_HEADER)
        for i, p in enumerate(points, 1):
            w.writerow([i, p.t_ms, f"{p.raw:.2f}", f"{p.raw_std:.2f}",
                        f"{p.true_mv:.1f}", f"{p.cal_mv:.1f}",
                        f"{p.lin_mv:.1f}", f"{p.cal_err:.2f}",
                        f"{p.lin_err:.2f}", p.n_avg])
        w.writerow([])
        w.writerow(["# vref_mv", vref_mv, "full_scale", full_scale])
        w.writerow(["# exported", datetime.now().isoformat(timespec="seconds")])
        for k, v in (cfg or {}).items():
            w.writerow([f"# cfg_{k}", v])
        if burst:
            w.writerow(["# burst_summary"])
            for k, v in burst.items():
                if isinstance(v, (int, float, str)) or v is None:
                    w.writerow([f"# burst_{k}", v])


def load_points_csv(path: str | Path) -> tuple[list[CalPoint], dict]:
    """读取本工程导出的标定点 CSV（也兼容 v1 表头）。返回 (points, meta)。"""
    points: list[CalPoint] = []
    meta: dict = {}
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            return points, meta
        idx = {name.strip(): i for i, name in enumerate(header)}
        if "true_mv" not in idx or "raw_code" not in idx:
            raise ValueError("不是本工程的标定点 CSV：缺少 true_mv / raw_code 表头")

        def get(row, name, default=""):
            i = idx.get(name)
            return row[i] if i is not None and i < len(row) else default

        for row in reader:
            if not row or not row[0].strip():
                continue
            if row[0].startswith("#"):
                key = row[0].lstrip("# ").strip()
                if key == "vref_mv" and len(row) > 1:
                    meta["vref_mv"] = float(row[1])
                elif key == "full_scale" and len(row) > 3:
                    meta["full_scale"] = float(row[3])
                elif key.startswith("cfg_") and len(row) > 1:
                    meta[key] = row[1]
                elif key.startswith("burst_") and len(row) > 1:
                    try:
                        meta[key] = float(row[1])
                    except ValueError:
                        meta[key] = row[1]
                elif key in ("exported", "burst_summary"):
                    if len(row) > 1:
                        meta[key] = row[1]
                continue
            try:
                points.append(CalPoint(
                    t_ms=int(float(get(row, "t_ms", 0) or 0)),
                    raw=float(get(row, "raw_code")),
                    true_mv=float(get(row, "true_mv")),
                    cal_mv=float(get(row, "cal_mv")),
                    lin_mv=float(get(row, "lin_mv")),
                    raw_std=float(get(row, "raw_std", 0) or 0),
                    n_avg=int(float(get(row, "n_avg", 1) or 1)),
                ))
            except ValueError:
                continue
    return points, meta


def split_invalid(points: Sequence[CalPoint],
                  threshold_mv: float = 1000.0) -> tuple[list[CalPoint], list[CalPoint]]:
    """分离明显异常的点：真值与 raw 线性换算相差 > threshold。

    典型原因：输错单位（3.3 当成 3300 mV）、电位器不在输入值、接错通道。
    正常点的线性误差最大约 400 mV（12 dB 满量程附近），1000 mV 阈值不会误伤。
    """
    good, bad = [], []
    for p in points:
        (bad if abs(p.lin_mv - p.true_mv) > threshold_mv else good).append(p)
    return good, bad


# ----------------------------------------------------------------------------
# 报告
# ----------------------------------------------------------------------------
def _fmt(v, nd=1, suffix=""):
    return "—" if v is None else f"{v:.{nd}f}{suffix}"


def build_report_markdown(points: Sequence[CalPoint], cfg: dict | None = None,
                          vref_mv: float = VREF_MV,
                          full_scale: float = ADC_FULL_SCALE,
                          title: str = "ESP32-C3 ADC 线性度实测报告",
                          burst: dict | None = None) -> str:
    cfg = cfg or {}
    sat_thr = full_scale - 5.0
    valid, bad = split_invalid(points)
    if not valid:
        valid = list(points)
        bad = []
    sat = [p for p in valid if p.raw >= sat_thr]
    usable = [p for p in valid if p.raw < sat_thr]
    stat_pts = usable if (sat and len(usable) >= 2) else list(valid)
    comp = compare_methods(stat_pts)
    full_comp = compare_methods(valid) if sat else comp
    cal, lin = comp["cal"], comp["lin"]
    lines: list[str] = [f"# {title}", ""]
    lines.append(f"- 生成时间：{datetime.now():%Y-%m-%d %H:%M:%S}")
    if cfg:
        lines.append("- 固件配置：" + "，".join(f"`{k}={v}`" for k, v in cfg.items()))
    lines.append(f"- 标定点数：{len(points)}；有效：{len(valid)}；"
                 f"线性度统计：{len(stat_pts)}；"
                 f"真值覆盖：{_fmt(cal['span_mv'], 0, ' mV')} 跨度")
    if valid:
        lo = min(p.true_mv for p in valid)
        hi = max(p.true_mv for p in valid)
        lines.append(f"- 有效真值范围：{lo:.1f} ~ {hi:.1f} mV")
    if bad:
        lines.append(f"- ⚠ 异常点 {len(bad)} 个（|线性换算−真值| > 1000 mV，"
                     "可能输错单位/电位器位置不对）已排除，见 1.2。")
    if sat:
        lines.append(f"- ⚠ 饱和点 {len(sat)} 个（raw ≥ {sat_thr:.0f}）已从线性度统计中排除，"
                     "全量程含饱和的误差见 1.1、饱和区见 4。")
    lines.append("")

    lines += [("## 1. 两种换算方式对比（可用范围）" if sat
               else "## 1. 两种换算方式对比"), "",
              "| 指标 | ESP-IDF 校准 | 原始码线性 (×3300/4095) |",
              "|---|---:|---:|"]
    rows = [
        ("max |误差| (mV)", "max_abs_err", 2),
        ("RMS 误差 (mV)", "rms_err", 2),
        ("平均误差 (mV)", "mean_err", 2),
        ("拟合斜率", "gain", 4),
        ("拟合截距 (mV)", "offset", 2),
        ("INL / 拟合残差 (mV)", "inl", 2),
    ]
    for name, key, nd in rows:
        lines.append(f"| {name} | {_fmt(cal[key], nd)} | {_fmt(lin[key], nd)} |")
    lines.append("")

    if sat:
        fc, fl = full_comp["cal"], full_comp["lin"]
        lines += ["### 1.1 全量程（含饱和点，仅供参考）", "",
                  "| 指标 | ESP-IDF 校准 | 原始码线性 |",
                  "|---|---:|---:|",
                  f"| max |误差| (mV) | {_fmt(fc['max_abs_err'], 2)} | {_fmt(fl['max_abs_err'], 2)} |",
                  f"| RMS 误差 (mV) | {_fmt(fc['rms_err'], 2)} | {_fmt(fl['rms_err'], 2)} |",
                  "", "> 饱和区输入无法测量，全量程误差会被饱和平台主导，"
                  "线性度结论应以 1 的可用范围为准。", ""]

    if bad:
        lines += ["### 1.2 异常点（已排除）", "",
                  "| # | raw | 真值 mV | 线性 mV | 线性−真值 |",
                  "|---:|---:|---:|---:|---:|"]
        for i, p in enumerate(bad, 1):
            lines.append(f"| {i} | {p.raw:.1f} | {p.true_mv:.1f} | "
                         f"{p.lin_mv:.1f} | {p.lin_mv - p.true_mv:+.1f} |")
        lines += ["", "> 这些点通常是输入真值与 ADC 读数不同步，"
                  "不能用于统计；请检查单位/电位器位置后重采。", ""]

    # 逐点表
    if points:
        lines += ["## 2. 标定点明细", "",
                  "| # | 原始码 | 真值 mV | 校准 mV | 线性 mV | 校准误差 | 线性误差 |",
                  "|---:|---:|---:|---:|---:|---:|---:|"]
        for i, p in enumerate(points, 1):
            lines.append(f"| {i} | {p.raw:.1f} | {p.true_mv:.1f} | {p.cal_mv:.1f} | "
                         f"{p.lin_mv:.1f} | {p.cal_err:+.1f} | {p.lin_err:+.1f} |")
        lines.append("")

    # 结论
    lines += ["## 3. 结论", ""]
    if not points:
        lines.append("尚未记录标定点，无法给出结论。")
    else:
        winner = comp["winner"]
        name = {"cal": "ESP-IDF 校准", "lin": "原始码线性换算"}[winner]
        best = min(cal["max_abs_err"], lin["max_abs_err"])
        worst = max(cal["max_abs_err"], lin["max_abs_err"])
        gain_txt = (f"校准斜率 {cal['gain']:.4f}、截距 {cal['offset']:+.1f} mV；"
                    f"线性斜率 {lin['gain']:.4f}、截距 {lin['offset']:+.1f} mV。"
                    if cal["gain"] is not None and lin["gain"] is not None else "")
        lines.append(f"- 最大误差更小的是 **{name}**"
                     f"（{best:.2f} mV vs {worst:.2f} mV）。")
        if worst > 0:
            lines.append(f"- 相对改善：{(worst - best) / worst * 100:.1f}%。")
        if gain_txt:
            lines.append(f"- {gain_txt}")
        lines.append(f"- 校准 RMS {cal['rms_err']:.2f} mV，"
                     f"线性 RMS {lin['rms_err']:.2f} mV。" if cal["rms_err"] is not None else "")
        noise = [p.raw_std for p in stat_pts if p.raw_std > 0]
        if noise:
            mean_std = sum(noise) / len(noise)
            lines.append(f"- 标定点平均单窗口 σ ≈ {mean_std:.2f} 码"
                         f"（≈ {mean_std * vref_mv / full_scale:.2f} mV）。")
        span = cal["span_mv"] or 0
        if len(stat_pts) < 10 or span < 2800:
            lines.append(f"- ⚠ 覆盖不足（{len(stat_pts)} 点，{span:.0f} mV）："
                         "建议至少 10 点覆盖 100~2800 mV 的可用范围。")
        else:
            lines.append(f"- 覆盖满足要求（{len(stat_pts)} 点，{span:.0f} mV 可用范围）。")
        if sat:
            sat_lo = min(p.true_mv for p in sat)
            sat_hi = max(p.true_mv for p in sat)
            plateau = [p.cal_mv for p in sat]
            top_ok = max(p.true_mv for p in usable) if usable else 0
            lines.append(
                f"- ⚠ 饱和：{len(sat)} 点（真值 {sat_lo:.0f}~{sat_hi:.0f} mV，"
                f"raw ≥ {sat_thr:.0f}），校准输出被钳在 "
                f"{min(plateau):.0f}~{max(plateau):.0f} mV；"
                f"12 dB 档位可用上限约 {top_ok:.0f} mV，不足以覆盖 3.3 V。")

    if sat:
        plateau = [p.cal_mv for p in sat]
        lin_plateau = [p.lin_mv for p in sat]
        lines += ["", "## 4. 饱和区", "",
                  "| 项目 | 值 |", "|---|---|",
                  f"| 判据 | raw ≥ {sat_thr:.0f}（满量程 {full_scale:.0f}） |",
                  f"| 点数 | {len(sat)} / {len(points)} |",
                  f"| 真值范围 | {min(p.true_mv for p in sat):.1f} ~ "
                  f"{max(p.true_mv for p in sat):.1f} mV |",
                  f"| 校准平台 | {min(plateau):.1f} ~ {max(plateau):.1f} mV |",
                  f"| 线性平台 | {min(lin_plateau):.1f} ~ {max(lin_plateau):.1f} mV |",
                  "", "> 此区间 ADC 已饱和；继续增大输入不会改变 raw，"
                  "两种换算都不能代表真实输入电压。", ""]

    if burst:
        lines += ["", ("## 5. 高速噪声抓取" if sat else "## 4. 高速噪声抓取"), "",
                  f"- 采样点数：{burst.get('n')}，等效采样率：{_fmt(burst.get('fs'), 1, ' Hz')}",
                  f"- 均值：{_fmt(burst.get('mean_code'), 1, ' 码')}，"
                  f"峰峰值：{_fmt(burst.get('pp_code'), 0, ' 码')}"
                  f"（{_fmt(burst.get('pp_mv'), 1, ' mV')}）",
                  f"- σ：{_fmt(burst.get('std_code'), 2, ' 码')}"
                  f"（{_fmt(burst.get('std_mv'), 2, ' mV')}）",
                  f"- 主频：{_fmt(burst.get('f_peak'), 1, ' Hz')}，"
                  f"幅度：{_fmt(burst.get('amp_peak_mv'), 2, ' mV')}",
                  f"- 50 Hz 幅度：{_fmt(burst.get('amp_50_mv'), 2, ' mV')}；"
                  f"100 Hz 幅度：{_fmt(burst.get('amp_100_mv'), 2, ' mV')}"]
        f50 = burst.get("f_peak")
        if f50 and 45 <= f50 <= 55:
            lines.append("- 主频落在 50 Hz 附近，说明未接滤波/高阻输入拾取了工频干扰；"
                         "加 100 nF 并缩短引线后复测。")

    sec = 4 + (1 if sat else 0) + (1 if burst else 0)
    lines += ["", f"## {sec}. 说明", "",
              f"- 线性换算按 `raw / {full_scale:g} × {vref_mv:g} mV`，"
              "未使用 eFuse 校准参数。",
              "- 真值为万用表人工读数；不含万用表自身精度（通常 ±0.5% 读数 + 几位）。",
              "- INL 定义为各自最佳拟合直线的最大残差，不是端点直线 INL。",
              ""]
    return "\n".join(lines)


# ----------------------------------------------------------------------------
# 便捷入口：从 CSV 直接生成报告
# ----------------------------------------------------------------------------
def report_from_csv(path: str | Path, out_dir: str | Path | None = None,
                    title: str = "ESP32-C3 ADC 线性度实测报告") -> Path:
    points, meta = load_points_csv(path)
    cfg = {k[4:]: v for k, v in meta.items() if k.startswith("cfg_")}
    burst = None
    if meta.get("burst_summary"):
        burst = {k[len("burst_"):]: v for k, v in meta.items()
                 if k.startswith("burst_") and k != "burst_summary"}
        for k, v in list(burst.items()):
            try:
                burst[k] = float(v)
            except (TypeError, ValueError):
                pass
    md = build_report_markdown(points, cfg, title=title, burst=burst)
    out = Path(out_dir) if out_dir else Path(path).with_suffix("")
    out = Path(str(out) + "_report.md") if out.suffix != ".md" else out
    out.write_text(md, encoding="utf-8")
    return out
