#!/usr/bin/env python3
"""
GELLO 夹爪可视化脚本（只测 GELLO 端，不连 Franka）

- 在 gello 环境运行
- 持续读取 GelloAgent 的关节状态
- 取第 8 个关节作为夹爪开合值（0~1）
- 终端打印当前值 + 简单进度条
- 弹出实时曲线窗口，观察夹爪是否真的在开/合

使用方法：
    conda activate gello
    cd ~/Desktop/Workspace/robot/polymetis/lee
    python gello_gripper_visualizer.py \
        --gello-port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAW4VCW-if00-port0 \
        --rate 30
"""

import sys
import time
from pathlib import Path
from argparse import ArgumentParser

import numpy as np
import matplotlib
matplotlib.use("TkAgg")  # 使用交互式后端
import matplotlib.pyplot as plt

# 添加 gello 路径（参考 gello_sender）
GELLO_ROOT = Path("/home/kyji/Desktop/Workspace/robot/gello_software")
sys.path.insert(0, str(GELLO_ROOT))

from gello.agents.gello_agent import GelloAgent


def print_bar(value: float, width: int = 30) -> str:
    """把 0~1 的值画成一行简单进度条"""
    v = min(max(value, 0.0), 1.0)
    filled = int(v * width)
    bar = "#" * filled + "-" * (width - filled)
    return f"[{bar}] {v:6.3f}"


def main():
    parser = ArgumentParser(description="GELLO 夹爪可视化")
    parser.add_argument("--gello-port", default="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAW4VCW-if00-port0",
                        help="GELLO 串口设备")
    parser.add_argument("--rate", type=float, default=30.0, help="采样频率 Hz")
    args = parser.parse_args()

    print("=" * 60)
    print("GELLO 夹爪可视化（仅检测 GELLO 端）")
    print("=" * 60)
    print(f"GELLO 串口: {args.gello_port}")
    print(f"采样频率: {args.rate} Hz")
    print("按 Ctrl+C 退出")
    print("=" * 60)

    # 1. 连接 GELLO
    try:
        gello = GelloAgent(port=args.gello_port)
        joints = gello.act({})
        print(f"✓ GELLO 设备已连接，初始关节前 8 项: {np.round(joints[:8], 3)}")
    except Exception as e:
        print(f"✗ GELLO 初始化失败: {e}")
        return

    # 2. 准备绘图
    dt = 1.0 / args.rate
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 4))
    fig.canvas.manager.set_window_title("GELLO 夹爪值 (0~1)")

    times = []
    values = []
    line, = ax.plot([], [], "b-")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Gripper (0~1)")
    ax.set_ylim(-0.1, 1.1)
    ax.grid(True, alpha=0.3)

    start_time = time.time()
    last_print = start_time

    try:
        while True:
            t0 = time.time()

            joints = gello.act({})
            if joints.shape[0] >= 8:
                g = float(joints[7])
            else:
                g = float(joints[-1])

            now = time.time() - start_time
            times.append(now)
            values.append(g)

            # 终端打印（每 0.1s 打一次）
            if time.time() - last_print >= 0.1:
                bar = print_bar(g)
                print(f"t={now:6.2f}s  gripper={g:7.4f}  {bar}")
                last_print = time.time()

            # 更新曲线（只更新最近 10 秒的窗口）
            # 避免数据太多导致绘图变慢
            while times and now - times[0] > 10.0:
                times.pop(0)
                values.pop(0)

            line.set_data(times, values)
            if times:
                ax.set_xlim(times[0], times[-1] + 0.1)
            fig.canvas.draw_idle()
            plt.pause(0.001)

            # 控制频率
            elapsed = time.time() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，退出可视化...")
    finally:
        plt.ioff()
        try:
            plt.close(fig)
        except Exception:
            pass
        print("完成。")


if __name__ == "__main__":
    main()
