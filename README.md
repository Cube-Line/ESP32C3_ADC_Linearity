# ESP32C3_ADC_Linearity

ESP32-C3 SuperMini ADC 线性度实验：对比 **ESP-IDF 校准曲线** 与 **原始码线性换算**。

- 固件：ESP-IDF v5.5.x，ADC1（GPIO0~GPIO4 可切换），12 dB / 12 bit，可配置过采样与周期
- 上位机：Python + tkinter + matplotlib，实时曲线、稳定判据 + 多点平均、万用表真值标定、
  误差/INL/斜率、会话保存、CSV/报告导出
- 诊断：固件 `B` 命令高速抓取原始码，上位机做 FFT，定位 50 Hz 工频干扰/混叠
- Wi-Fi 干扰：STA 连接指定 Wi-Fi，GUI 一键开关，主机 UDP 广播压测（约 250 pps 双向）
- 离线分析：`host/analyze.py` 对 CSV 生成统计、Markdown 报告与 PNG
- 文档：见 [docs/README.md](docs/README.md)

## 快速开始

```bash
# 1. 编译烧录（本机已装 ESP-IDF v5.5.5）
source ~/.espressif/v5.5.5/esp-idf/export.sh
idf.py set-target esp32c3 && idf.py build
idf.py -p /dev/ttyACM0 flash

# 2. 上位机
python3 -m venv .venv
.venv/bin/pip install -r host/requirements.txt
.venv/bin/python host/gui.py --port /dev/ttyACM0

# 无硬件演示
.venv/bin/python host/gui.py --demo

# 或：命令行逐点标定（无需桌面）
.venv/bin/python host/sweep.py --port /dev/ttyACM0

# Wi-Fi 干扰实验：GUI 点「Wi-Fi 负载」后，电脑连同一 Wi-Fi，运行
.venv/bin/python host/wifi_load.py
```

## 接线

```
3V3 ── 103（10 kΩ）电位器固定端
GND ── 103（10 kΩ）电位器固定端
GPIO3（丝印 3）── 抽头
GPIO3 ──||── GND   (100 nF，建议)
```

输入范围 0~3.3 V，禁止超压。

## 固件串口命令（v2）

在串口终端输入（一行一条，`H` 查看帮助）：

| 命令 | 作用 |
|---|---|
| `S` | 打印当前配置 |
| `C 0..4` | 切换 ADC1 通道 / GPIO（交叉验证引脚） |
| `A 0/2/6/12` | 切换衰减 dB |
| `N 1..256` | 过采样点数（默认 64） |
| `P 10..5000` | 采样周期 ms（默认 50） |
| `B [n]` | 高速抓取 n 个原始码（默认 1024，最大 8192），用于频谱 |
| `W 1/0/s` | Wi-Fi 负载开/关/查状态（STA → 指定 Wi-Fi） |

GUI 的「噪声频谱 (F9)」会自动发送 `B 4096` 并绘图，显示主频与 50/100 Hz 幅度。

## 离线分析

```bash
.venv/bin/python host/analyze.py adc_points_xxx.csv
# 生成 adc_points_xxx_report.md 与 _report.png
```

## 自测

```bash
.venv/bin/python -m unittest host/test_core.py -v
```

## 当前状态

已完成 **291 点全量程扫描**，并在加 **100 nF** 后完成 31 点复测（0.1~2.7 V，24 点）：

| 指标（0.1~2.7 V） | ESP-IDF 校准 | 原始码线性 |
|---|---:|---:|
| max 误差 | **12.6 mV** | 306.0 mV |
| RMS 误差 | **6.73 mV** | 162.6 mV |
| INL | 5.56 mV | 30.3 mV |
| +两点修正 | max 5.53 / RMS 3.12 mV | — |

结论：
- **用 curve fitting 校准，不要用 `raw×3300/4095`**；线性换算在 0.5~2.5 V 偏大 +7%~+12%。
- 12 dB 档位约 **2.89 V** 饱和，实际可用上限约 **2.87 V**，不能直接覆盖 3.3 V，需外部 1:2 分压。
- **加 100 nF 后**：1.65 V 处 σ 从 30.8 mV 降到 **1.05 mV**，峰峰 250 → 7.3 mV；
  Wi-Fi 压测时 σ 升到 **1.79 mV**（+70%），绝对增量 <1 mV。

完整结论见 [docs/09-实验结论与建议.md](docs/09-实验结论与建议.md)；
原始数据、报告与配图见 [data/](data/README.md)。
