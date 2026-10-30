#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线分析标定点 CSV，输出统计、Markdown 报告与 PNG 图。

用法：
    python analyze.py adc_points_20261030_101500.csv
    python analyze.py points.csv --out ./report --title "C3 全量程实测"
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from adc_core import (ADC_FULL_SCALE, VREF_MV, build_report_markdown,
                      compare_methods, load_points_csv, split_invalid)

ROOT = Path(__file__).resolve().parent


def _fmt(v, nd=2):
    return "—" if v is None else f"{v:.{nd}f}"


def print_summary(comp: dict, heading: str = "") -> None:
    cal, lin = comp["cal"], comp["lin"]
    if heading:
        print(f"\n{heading}")
    print(f"{'指标':<24}{'ESP-IDF 校准':>16}{'原始码线性':>16}")
    print("-" * 56)
    rows = [("max |误差| (mV)", "max_abs_err", 2),
            ("RMS 误差 (mV)", "rms_err", 2),
            ("平均误差 (mV)", "mean_err", 2),
            ("拟合斜率", "gain", 4),
            ("拟合截距 (mV)", "offset", 2),
            ("INL (mV)", "inl", 2)]
    for name, key, nd in rows:
        print(f"{name:<24}{_fmt(cal[key], nd):>16}{_fmt(lin[key], nd):>16}")
    print(f"{'点数':<24}{cal['n']:>16}{lin['n']:>16}")
    winner = comp["winner"]
    if winner:
        print(f"\n最大误差更小：{'ESP-IDF 校准' if winner == 'cal' else '原始码线性'}")


def make_plot(points, out_png: Path, cfg: dict) -> bool:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from adc_core import setup_matplotlib_fonts
        setup_matplotlib_fonts()
    except ImportError:
        print("未安装 matplotlib，跳过绘图", file=sys.stderr)
        return False

    true = [p.true_mv for p in points]
    cal = [p.cal_mv for p in points]
    lin = [p.lin_mv for p in points]
    cal_e = [p.cal_err for p in points]
    lin_e = [p.lin_err for p in points]

    fig, axes = plt.subplots(2, 1, figsize=(8, 8), constrained_layout=True)
    sat = [p for p in points if p.raw >= ADC_FULL_SCALE - 5]
    if sat:
        x0 = min(p.true_mv for p in sat)
        x1 = max(p.true_mv for p in sat)
        for a in axes:
            a.axvspan(x0, x1, color="#d62728", alpha=0.08, zorder=0)
        axes[0].text((x0 + x1) / 2, 0.92, "饱和区", ha="center", va="top",
                     color="#d62728", fontsize=9,
                     transform=axes[0].get_xaxis_transform())

    ax = axes[0]
    ax.plot(true, cal, "o-", color="#1f77b4", ms=4, label="ESP-IDF 校准")
    ax.plot(true, lin, "s--", color="#ff7f0e", ms=4, mfc="none", label="原始码线性")
    ax.plot([0, max(true)], [0, max(true)], color="#999999", lw=0.8,
            label="理想 1:1")
    ax.set_xlabel("万用表真值 / mV")
    ax.set_ylabel("测量值 / mV")
    ax.set_title("测量值 vs 真值")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=9)

    ax = axes[1]
    ax.axhline(0, color="#999999", lw=0.8)
    ax.plot(true, cal_e, "o-", color="#1f77b4", ms=4, label="ESP-IDF 校准")
    ax.plot(true, lin_e, "s--", color="#ff7f0e", ms=4, mfc="none", label="原始码线性")
    ax.set_xlabel("万用表真值 / mV")
    ax.set_ylabel("误差 / mV")
    ax.set_title("误差（测量 - 真值）")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)

    cfg_txt = "  ".join(f"{k}={v}" for k, v in cfg.items()) if cfg else ""
    if cfg_txt:
        fig.suptitle(cfg_txt, fontsize=8)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description="ESP32-C3 ADC 线性度离线分析")
    ap.add_argument("csv", help="GUI 导出的标定点 CSV")
    ap.add_argument("--out", help="输出目录（默认与 CSV 同目录）")
    ap.add_argument("--title", default="ESP32-C3 ADC 线性度实测报告")
    ap.add_argument("--no-plot", action="store_true", help="不生成 PNG")
    args = ap.parse_args()

    path = Path(args.csv)
    points, meta = load_points_csv(path)
    if not points:
        print("CSV 中没有有效标定点", file=sys.stderr)
        return 2

    valid, bad = split_invalid(points)
    if not valid:
        valid = list(points)
        bad = []
    if bad:
        print(f"异常点 {len(bad)} 个（|线性−真值| > 1000 mV）已排除：")
        for p in bad:
            print(f"  raw {p.raw:.1f}  真值 {p.true_mv:.1f} mV  "
                  f"线性 {p.lin_mv:.1f} mV  差 {p.lin_mv - p.true_mv:+.1f} mV")
    sat_thr = ADC_FULL_SCALE - 5.0
    sat = [p for p in valid if p.raw >= sat_thr]
    usable = [p for p in valid if p.raw < sat_thr]
    stat_pts = usable if (sat and len(usable) >= 2) else valid
    comp = compare_methods(stat_pts)
    if sat:
        print(f"饱和点 {len(sat)} 个（raw ≥ {sat_thr:.0f}）已排除；"
              f"可用范围 {len(usable)} 点。")
    print_summary(comp, "可用范围" if sat else "全量程")
    if sat:
        print_summary(compare_methods(valid), "全量程（含饱和，仅供参考）")

    out_dir = Path(args.out) if args.out else path.parent
    out_dir.mkdir(parents=True, exist_ok=True)
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

    md_path = out_dir / f"{path.stem}_report.md"
    md_path.write_text(build_report_markdown(points, cfg,
                                             vref_mv=meta.get("vref_mv", VREF_MV),
                                             full_scale=meta.get("full_scale", ADC_FULL_SCALE),
                                             title=args.title, burst=burst),
                       encoding="utf-8")
    print(f"\n报告：{md_path}")

    if not args.no_plot:
        png = out_dir / f"{path.stem}_report.png"
        if make_plot(valid, png, cfg):
            print(f"配图：{png}")

    if comp["winner"]:
        cal, lin = comp["cal"], comp["lin"]
        print(f"结论：{'ESP-IDF 校准' if comp['winner'] == 'cal' else '原始码线性'} "
              f"最大误差更小（{min(cal['max_abs_err'], lin['max_abs_err']):.2f} mV）")
    if sat:
        print(f"注意：{len(sat)} 个饱和点（真值 {min(p.true_mv for p in sat):.0f}~"
              f"{max(p.true_mv for p in sat):.0f} mV）不能用于线性度统计。")
    if bad:
        print(f"注意：{len(bad)} 个异常点已排除，请检查是否输错单位或电位器位置。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
