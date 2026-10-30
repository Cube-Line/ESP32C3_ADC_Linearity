#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Wi-Fi 负载压测：向 ESP32 所在局域网持续广播 UDP，并接收 ESP32 的广播/回显。

ESP32 固件用 `W 1` 连接 指定 Wi-Fi 并启动 Wi-Fi UDP 负载（`W 0` 关闭）。
本脚本默认向本机 Wi-Fi 接口所在网段的广播地址发包，不需要知道 ESP32 的 IP；
这样即使路由器开启了客户端隔离（单播互不可达），广播仍能到达 ESP32。

用法：
    python host/wifi_load.py                  # 自动找 Wi-Fi 接口和广播地址
    python host/wifi_load.py --esp 192.168.1.100   # 指定 ESP32 单播（可选示例）
    python host/wifi_load.py --seconds 10
    Ctrl-C 结束。
"""

from __future__ import annotations

import argparse
import re
import socket
import subprocess
import threading
import time


def list_ifaces() -> list[tuple[str, str, str]]:
    """返回 [(接口, IPv4, 广播地址)]。"""
    try:
        out = subprocess.check_output(["ip", "-4", "-o", "addr", "show"],
                                      text=True, timeout=2)
    except Exception:  # noqa: BLE001
        return []
    res = []
    for line in out.splitlines():
        m = re.match(r"\d+:\s+(\S+)\s+inet\s+(\d+\.\d+\.\d+\.\d+/\d+)", line)
        if not m:
            continue
        iface, cidr = m.group(1), m.group(2)
        b = re.search(r"brd\s+(\d+\.\d+\.\d+\.\d+)", line)
        if b:
            res.append((iface, cidr.split("/")[0], b.group(1)))
    return res


def pick_iface(esp: str | None) -> tuple[str, str, str] | None:
    infos = list_ifaces()
    if esp:
        prefix = ".".join(esp.split(".")[:3]) + "."
        infos = [x for x in infos if x[1].startswith(prefix)]
    infos.sort(key=lambda x: (0 if x[0].startswith("wl") else 1, x[0]))
    return infos[0] if infos else None


def main() -> int:
    ap = argparse.ArgumentParser(description="ESP32-C3 Wi-Fi UDP 压测")
    ap.add_argument("--esp", help="目标 IP；不填则用本机 Wi-Fi 网段的广播地址")
    ap.add_argument("--port", type=int, default=5000, help="UDP 端口")
    ap.add_argument("--size", type=int, default=1400, help="每包字节数")
    ap.add_argument("--seconds", type=float, default=0, help="运行秒数，0=直到 Ctrl-C")
    ap.add_argument("--bind", help="绑定本机 IP（可选）")
    ap.add_argument("--iface", help="绑定网络接口（默认自动找 wl*）")
    args = ap.parse_args()
    size = max(17, min(1472, args.size))

    info = pick_iface(args.esp)
    iface = args.iface or (info[0] if info else None)
    target = args.esp or (info[2] if info else None)
    if not target:
        print("找不到可用接口/目标；请用 --esp 或 --iface 指定", flush=True)
        return 2

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if iface:
        try:
            sock.setsockopt(socket.SOL_SOCKET, 25, iface.encode() + b"\0")  # SO_BINDTODEVICE
        except OSError as exc:
            print(f"绑定接口 {iface} 失败：{exc}（可尝试 sudo）", flush=True)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except OSError:
        pass
    sock.bind((args.bind or "", args.port))
    sock.settimeout(0.5)

    stop = threading.Event()
    tx_pkts = tx_bytes = 0
    rx_pkts = rx_bytes = 0
    lock = threading.Lock()
    last_rx_ts = time.monotonic()

    def receiver():
        nonlocal rx_pkts, rx_bytes, last_rx_ts
        while not stop.is_set():
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            with lock:
                rx_pkts += 1
                rx_bytes += len(data)
                last_rx_ts = time.monotonic()

    th = threading.Thread(target=receiver, daemon=True)
    th.start()

    payload = (b"ADC-WIFI-" + bytes(8)) + b"x" * (size - 17)
    payload = payload[:size]
    t0 = time.monotonic()
    last = t0
    prev_tx = prev_rx = 0
    print(f"目标 {target}:{args.port}，接口 {iface or '自动'}，每包 {size} B；Ctrl-C 结束")
    print("提示：ESP32 需先 W 1（GUI 的「Wi-Fi 负载」按钮），并连到同一 指定 Wi-Fi 网络")
    try:
        while True:
            now = time.monotonic()
            if args.seconds and now - t0 >= args.seconds:
                break
            for _ in range(8):
                try:
                    n = sock.sendto(payload, (target, args.port))
                except OSError:
                    time.sleep(0.002)
                    break
                with lock:
                    tx_pkts += 1
                    tx_bytes += n
            if now - last >= 1.0:
                dt = now - last
                with lock:
                    dtx = tx_pkts - prev_tx
                    drx = rx_pkts - prev_rx
                    prev_tx, prev_rx = tx_pkts, rx_pkts
                print(f"TX {dtx / dt:7.0f} pkt/s  RX {drx / dt:7.0f} pkt/s  "
                      f"TX {dtx * size * 8 / dt / 1e6:5.1f} Mb/s  "
                      f"RX {drx * size * 8 / dt / 1e6:5.1f} Mb/s  "
                      f"累计 tx={tx_pkts} rx={rx_pkts}", flush=True)
                if drx == 0 and time.monotonic() - last_rx_ts > 3:
                    print("  （3 s 无回包：确认 ESP32 已 W 1、与本机在同一 指定 Wi-Fi 网络）",
                          flush=True)
                last = now
    except KeyboardInterrupt:
        print("\n停止中…")
    finally:
        stop.set()
        th.join(timeout=1.0)
        sock.close()

    dt = max(1e-6, time.monotonic() - t0)
    print(f"累计：TX {tx_pkts} 包 / {tx_bytes / 1e6:.1f} MB，"
          f"RX {rx_pkts} 包 / {rx_bytes / 1e6:.1f} MB，"
          f"平均 {tx_pkts / dt:.0f} tx pps，{rx_pkts / dt:.0f} rx pps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
