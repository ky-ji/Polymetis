#!/usr/bin/env python3
"""
GELLO 夹爪状态发送脚本（在 gello 环境运行）

- 连接 GELLO 设备，读取关节状态（包含 7 轴 + 1 夹爪）
- 把第 8 个关节（夹爪开合，0~1）通过 TCP socket 发送给机械臂控制脚本

使用方法：
    conda activate gello
    cd ~/Desktop/Workspace/robot/polymetis
    python gello_gripper_sender.py --server-host localhost --server-port 6000
"""

import sys
import time
import json
import socket
from pathlib import Path
from argparse import ArgumentParser
from datetime import datetime
import csv
import os

import numpy as np

# 添加 gello 路径（参考现有 gello_software 工程）
GELLO_ROOT = Path("/home/kyji/Desktop/Workspace/robot/gello_software")
sys.path.insert(0, str(GELLO_ROOT))

from gello.agents.gello_agent import GelloAgent


class GripperSenderLogger:
    """记录 GELLO 侧夹爪值到 CSV，方便对比灵敏度"""

    def __init__(self):
        base_dir = Path(__file__).resolve().parent
        log_root = base_dir / "trajectory_logs"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = log_root / f"gello_gripper_{timestamp}"
        os.makedirs(self.log_dir, exist_ok=True)

        self.csv_path = self.log_dir / "gello_gripper.csv"
        self.rows = []

        print(f"日志目录 (GELLO 夹爪): {self.log_dir}")

    def log(self, t: float, gripper: float):
        self.rows.append({
            "time": t,
            "gripper": gripper,
        })

    def save(self):
        if not self.rows:
            return
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["time", "gripper"])
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"✓ GELLO 夹爪日志已保存: {self.csv_path} ({len(self.rows)} 帧)")


class GelloGripperSender:
    """只负责从 GELLO 读取夹爪状态，并通过 socket 发送"""

    def __init__(self, gello_port: str, server_host: str, server_port: int):
        self.gello_port = gello_port
        self.server_host = server_host
        self.server_port = server_port

        self.gello = None
        self.sock = None
        self.logger = None

    def initialize(self) -> bool:
        print("=" * 60)
        print("GELLO 夹爪状态发送端")
        print("=" * 60)
        print(f"GELLO 串口: {self.gello_port}")
        print(f"目标服务器: {self.server_host}:{self.server_port}")
        print("=" * 60)

        # 1. 初始化 GELLO
        print("[1/2] 初始化 GELLO 设备...")
        try:
            self.gello = GelloAgent(port=self.gello_port)
            print("✓ GELLO 设备已连接")
            joints = self.gello.act({})
            print(f"  初始关节(前 8 项): {np.round(joints[:8], 3)}")
        except Exception as e:
            print(f"✗ GELLO 初始化失败: {e}")
            return False

        # 2. 连接到机械臂控制脚本（作为 TCP client）
        print("\n[2/2] 连接到机械臂控制端...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.server_host, self.server_port))
            print(f"✓ 已连接到 {self.server_host}:{self.server_port}")
        except Exception as e:
            print(f"✗ 连接服务器失败: {e}")
            return False

        # 3. 初始化日志记录器
        self.logger = GripperSenderLogger()

        print("\n✓ 初始化完成\n")
        return True

    def _send_json(self, data: dict):
        msg = (json.dumps(data) + "\n").encode("utf-8")
        self.sock.sendall(msg)

    def run(self, rate_hz: float = 30.0):
        dt = 1.0 / rate_hz
        print("=" * 60)
        print("开始发送 GELLO 夹爪状态")
        print("=" * 60)
        print(f"发送频率: {rate_hz} Hz")
        print("按 Ctrl+C 停止...\n")

        start_time = time.time()
        last_print = start_time
        count = 0

        try:
            while True:
                t0 = time.time()

                # 读取所有关节（含夹爪），GelloAgent.act 返回的是 get_joint_state()
                joints = self.gello.act({})
                # 取前 8 个（7 轴 + 1 夹爪），第 8 个是 [0,1] 的开合值
                if joints.shape[0] >= 8:
                    gripper_pos = float(joints[7])
                else:
                    gripper_pos = float(joints[-1])

                # 组包并发送
                payload = {
                    "type": "gripper_state",
                    "timestamp": time.time(),
                    "gripper": gripper_pos,
                }
                try:
                    self._send_json(payload)
                    count += 1
                except Exception as e:
                    print(f"\n✗ 发送失败: {e}")
                    break

                # 记录日志
                if self.logger is not None:
                    self.logger.log(time.time() - start_time, gripper_pos)

                # 每秒打印一次状态
                now = time.time()
                if now - last_print >= 1.0:
                    elapsed = now - start_time
                    freq = count / elapsed if elapsed > 0 else 0.0
                    print(f"[{elapsed:6.1f}s] 已发送: {count:5d} | 频率: {freq:5.1f} Hz | gripper={gripper_pos:.3f}")
                    last_print = now

                # 控制频率
                elapsed_loop = time.time() - t0
                sleep_t = dt - elapsed_loop
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except KeyboardInterrupt:
            print("\n检测到 Ctrl+C，正在停止...")
        finally:
            self.stop(start_time, count)

    def stop(self, start_time: float, count: int):
        print("\n" + "=" * 60)
        print("停止发送")
        print("=" * 60)

        # 发送停止信号
        if self.sock is not None:
            try:
                self._send_json({"type": "stop"})
            except Exception:
                pass
            try:
                self.sock.close()
                print("✓ 已关闭 socket 连接")
            except Exception:
                pass

        # 保存日志
        if self.logger is not None:
            self.logger.save()

        if start_time:
            total = time.time() - start_time
            freq = count / total if total > 0 else 0.0
            print(f"总运行时间: {total:.1f} s, 发送次数: {count}, 平均频率: {freq:.1f} Hz")

        print("\n✓ GelloGripperSender 已停止\n")


def main():
    parser = ArgumentParser(description="GELLO 夹爪状态发送端")
    parser.add_argument("--gello-port", default="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAW4VCW-if00-port0",
                        help="GELLO 串口设备")
    parser.add_argument("--server-host", default="localhost", help="机械臂控制端 IP")
    parser.add_argument("--server-port", type=int, default=6000, help="机械臂控制端端口")
    parser.add_argument("--rate", type=float, default=30.0, help="发送频率 Hz")
    args = parser.parse_args()

    sender = GelloGripperSender(
        gello_port=args.gello_port,
        server_host=args.server_host,
        server_port=args.server_port,
    )
    if not sender.initialize():
        return

    sender.run(rate_hz=args.rate)


if __name__ == "__main__":
    main()
