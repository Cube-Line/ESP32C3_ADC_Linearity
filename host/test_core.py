#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""adc_core 单元测试（不依赖硬件/串口）。

运行：
    python -m unittest host/test_core.py -v
"""

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import adc_core as core


class TestParseVoltage(unittest.TestCase):
    def test_variants(self):
        cases = {
            "3.3": 3300.0, "3.3V": 3300.0, "3.3v": 3300.0,
            "3300": 3300.0, "3300mV": 3300.0, "3300 mv": 3300.0,
            "0": 0.0, "0.5": 500.0, "1234": 1234.0, "3mV": 3.0,
        }
        for text, expect in cases.items():
            self.assertAlmostEqual(core.parse_voltage_mv(text), expect, places=6, msg=text)
        for bad in ("", "abc", "nan", "1.2.3"):
            with self.assertRaises(ValueError):
                core.parse_voltage_mv(bad)


class TestParseLine(unittest.TestCase):
    def test_v1(self):
        s = core.parse_data_line("D,12345,204850,2040,2052,1649,1642,1655")
        self.assertIsNotNone(s)
        self.assertAlmostEqual(s.raw, 2048.5)
        self.assertEqual(s.pp_code, 12)
        self.assertEqual(s.raw_std_x100, 0)
        self.assertAlmostEqual(s.lin_mv, 2048.5 * 3300 / 4095)

    def test_v2_std(self):
        s = core.parse_data_line("D,1,100,99,101,80,79,81,35")
        self.assertAlmostEqual(s.std, 0.35)

    def test_bad(self):
        for line in ("", "#COL,a,b", "D,1,2,3", "D,1,x,2,3,4,5,6", "B,1,2"):
            self.assertIsNone(core.parse_data_line(line), line)


class TestStats(unittest.TestCase):
    def _points(self):
        pts = []
        for t in (100, 500, 1000, 1500, 2000, 2500, 3000):
            pts.append(core.CalPoint(0, t * 4095 / 3300, float(t), float(t),
                                     t * 1.01 + 5))
        return pts

    def test_perfect_cal(self):
        st = core.method_stats(self._points(), "cal")
        self.assertAlmostEqual(st["max_abs_err"], 0.0)
        self.assertAlmostEqual(st["rms_err"], 0.0)
        self.assertAlmostEqual(st["gain"], 1.0)
        self.assertAlmostEqual(st["offset"], 0.0)
        self.assertAlmostEqual(st["inl"], 0.0)

    def test_linear_gain_offset(self):
        st = core.method_stats(self._points(), "lin")
        self.assertAlmostEqual(st["gain"], 1.01, places=6)
        self.assertAlmostEqual(st["offset"], 5.0, places=6)
        self.assertAlmostEqual(st["inl"], 0.0, places=6)
        self.assertGreater(st["max_abs_err"], 0)

    def test_winner(self):
        comp = core.compare_methods(self._points())
        self.assertEqual(comp["winner"], "cal")

    def test_empty(self):
        st = core.method_stats([], "cal")
        self.assertEqual(st["n"], 0)
        self.assertIsNone(st["max_abs_err"])


class TestCsvReport(unittest.TestCase):
    def test_roundtrip(self):
        pts = [core.CalPoint(0, 100.0, 80.0, 81.0, 80.6, raw_std=0.5, n_avg=10),
               core.CalPoint(0, 2000.0, 1611.0, 1607.0, 1611.7, raw_std=1.2, n_avg=10),
               core.CalPoint(0, 4000.0, 3223.0, 3210.0, 3223.4, raw_std=2.0, n_avg=10)]
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "pts.csv"
            core.write_points_csv(pts, p, cfg={"pin": "GPIO3", "n": "64"},
                                  burst={"n": 4096, "fs": 30000.0, "f_peak": 50.0})
            back, meta = core.load_points_csv(p)
            self.assertEqual(len(back), 3)
            self.assertAlmostEqual(back[0].true_mv, 80.0)
            self.assertAlmostEqual(back[1].raw_std, 1.2)
            self.assertEqual(back[2].n_avg, 10)
            self.assertEqual(meta["cfg_pin"], "GPIO3")
            self.assertEqual(meta["burst_n"], 4096.0)

            md = core.build_report_markdown(back, {"pin": "GPIO3"},
                                            burst={"n": 4096, "fs": 30000.0,
                                                   "f_peak": 50.0,
                                                   "amp_50_mv": 12.0})
            self.assertIn("ESP-IDF 校准", md)
            self.assertIn("原始码线性", md)
            self.assertIn("50 Hz", md)
            self.assertIn("结论", md)

    def test_range_warning(self):
        pts = [core.CalPoint(0, 100.0, 80.0, 81.0, 80.6),
               core.CalPoint(0, 2000.0, 1611.0, 1607.0, 1611.7)]
        md = core.build_report_markdown(pts)
        self.assertIn("覆盖不足", md)


class TestInvalidPoints(unittest.TestCase):
    def test_split_invalid(self):
        good = core.CalPoint(0, 2000.0, 1600.0, 1600.0, 1611.0)
        bad = core.CalPoint(0, 0.3, 3300.0, 0.0, 0.2)
        g, b = core.split_invalid([good, bad])
        self.assertEqual(len(g), 1)
        self.assertEqual(len(b), 1)
        self.assertIs(b[0], bad)

    def test_report_excludes_invalid(self):
        pts = [core.CalPoint(0, 100.0, 80.0, 81.0, 80.6),
               core.CalPoint(0, 2000.0, 1611.0, 1607.0, 1611.7),
               core.CalPoint(0, 4000.0, 3223.0, 3210.0, 3223.4),
               core.CalPoint(0, 0.3, 3300.0, 0.0, 0.2)]
        md = core.build_report_markdown(pts)
        self.assertIn("异常点", md)
        self.assertIn("已排除", md)
        self.assertIn("标定点数：4；有效：3", md)


class TestSpectrum(unittest.TestCase):
    def test_50hz_peak(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy 未安装")
        fs = 10000.0
        n = 4096
        t = np.arange(n) / fs
        sig = 2048 + 200 * np.sin(2 * math.pi * 50 * t) + 30 * np.sin(2 * math.pi * 350 * t)
        summary = core.burst_summary(sig, total_us=n / fs * 1e6)
        self.assertAlmostEqual(summary["fs"], fs, delta=1.0)
        self.assertAlmostEqual(summary["f_peak"], 50.0, delta=fs / n)
        self.assertAlmostEqual(summary["amp_50_mv"], 200 * 3300 / 4095, delta=3.0)
        self.assertGreater(summary["pp_mv"], 300)
        self.assertGreater(summary["std_mv"], 50)


if __name__ == "__main__":
    unittest.main(verbosity=2)
