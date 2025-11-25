#!/usr/bin/env python3
"""
机械臂 + 夹爪控制脚本（在 robot 环境运行）

- 连接 Polymetis RobotInterface & GripperInterface
- 作为 TCP 服务器监听 GELLO 侧发送的夹爪状态（0~1）
- 根据收到的夹爪值控制 Franka 夹爪开合（同时可以扩展机械臂控制）

使用方法：
    conda activate robot
    cd ~/Desktop/Workspace/robot/polymetis
    python arm_gripper_controller.py --listen-port 6000 --robot-ip localhost --robot-port 50051

配合：
    conda activate gello
    cd ~/Desktop/Workspace/robot/polymetis
    python gello_gripper_sender.py --server-host localhost --server-port 6000
"""

import sys
import socket
import json
import time
from argparse import ArgumentParser
from datetime import datetime
from pathlib import Path
import csv
import os

import numpy as np
import torch

try:
    from polymetis import RobotInterface, GripperInterface
except ImportError as e:
    print(f"✗ 无法导入 polymetis: {e}")
    print("请确认已在 robot 环境中运行并安装 polymetis")
    sys.exit(1)


class GripperControllerLogger:
    """记录 Franka 侧夹爪控制命令，方便和 GELLO 侧对比"""

    def __init__(self):
        base_dir = Path(__file__).resolve().parent
        log_root = base_dir / "trajectory_logs"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = log_root / f"franka_gripper_{timestamp}"
        os.makedirs(self.log_dir, exist_ok=True)

        self.csv_path = self.log_dir / "franka_gripper.csv"
        self.rows = []

        print(f"日志目录 (Franka 夹爪): {self.log_dir}")

    def log(self, t: float, gripper_val: float, width: float):
        self.rows.append({
            "time": t,
            "gripper": gripper_val,
            "width": width,
        })

    def save(self):
        if not self.rows:
            return
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["time", "gripper", "width"])
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"✓ Franka 夹爪日志已保存: {self.csv_path} ({len(self.rows)} 帧)")


class ArmGripperController:
    """只负责连接 Polymetis，并根据外部夹爪状态控制夹爪"""

    def __init__(self, listen_host: str, listen_port: int, robot_ip: str, robot_port: int):
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.robot_ip = robot_ip
        self.robot_port = robot_port

        self.robot = None
        self.gripper = None
        self.server_sock = None
        self.client_sock = None
        self.recv_buffer = b""
        self.logger = None
        # 夹爪状态: 0=闭合, 1=打开
        self.gripper_toggle_state = 0
        # 上一帧按钮状态: 0=未按下, 1=按下（用于检测上升沿）
        self.last_button_state = 0

    # ---- 网络部分 ----

    def _recv_json(self):
        """从 client_sock 读取一行 JSON（以 \n 分隔）"""
        try:
            while True:
                if b"\n" in self.recv_buffer:
                    line, self.recv_buffer = self.recv_buffer.split(b"\n", 1)
                    return json.loads(line.decode("utf-8"))
                chunk = self.client_sock.recv(4096)
                if not chunk:
                    return None
                self.recv_buffer += chunk
        except Exception as e:
            print(f"✗ 接收数据失败: {e}")
            return None

    def _send_json(self, data: dict):
        msg = (json.dumps(data) + "\n").encode("utf-8")
        self.client_sock.sendall(msg)

    # ---- 初始化 Polymetis ----

    def _init_robot(self) -> bool:
        print("[1/2] 连接 Polymetis 机器人服务器...")
        try:
            self.robot = RobotInterface(ip_address=self.robot_ip, port=self.robot_port)
            print("✓ 已连接到机器人服务器")
        except Exception as e:
            print(f"✗ 连接机器人失败: {e}")
            return False

        # 连接夹爪
        print("连接夹爪服务器 (50052)...")
        try:
            self.gripper = GripperInterface(ip_address=self.robot_ip, port=50052)
            print("✓ 已连接到夹爪服务器")
        except Exception as e:
            print(f"✗ 连接夹爪失败: {e}")
            self.gripper = None

        # 启动关节阻抗（机械臂保持当前姿态）
        try:
            current = self.robot.get_joint_positions()
            print("当前关节:", np.round(current.numpy(), 3))
            self.robot.start_joint_impedance()
            time.sleep(0.5)
            self.robot.update_desired_joint_positions(current)
            print("✓ 关节阻抗控制已启动")
        except Exception as e:
            print(f"✗ 启动关节阻抗失败: {e}")
            return False

        return True

    # ---- 主流程 ----

    def initialize(self) -> bool:
        print("=" * 60)
        print("机械臂 + 夹爪控制端")
        print("=" * 60)
        print(f"监听地址: {self.listen_host}:{self.listen_port}")
        print(f"机器人服务器: {self.robot_ip}:{self.robot_port}")
        print("=" * 60)

        if not self._init_robot():
            return False

        # 建立 TCP 服务器
        print("\n[2/2] 启动 TCP 服务器，等待 GELLO 端连接...")
        try:
            self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_sock.bind((self.listen_host, self.listen_port))
            self.server_sock.listen(1)
            print("✓ TCP 服务器已启动，等待连接...")

            self.client_sock, addr = self.server_sock.accept()
            print(f"✓ GELLO 端已连接: {addr}")
        except Exception as e:
            print(f"✗ 启动/接受连接失败: {e}")
            return False

        # 初始化日志记录器
        self.logger = GripperControllerLogger()

        return True

    def run(self):
        print("\n开始接收夹爪状态并控制 Franka 夹爪 (0~0.09 m)...")
        print("按 Ctrl+C 停止\n")

        start_time = time.time()
        count = 0
        last_print = start_time

        try:
            while True:
                msg = self._recv_json()
                if not msg:
                    print("\n对端关闭连接，结束。")
                    break

                if msg.get("type") == "stop":
                    print("\n收到 stop 信号，结束。")
                    break

                if msg.get("type") != "gripper_state":
                    continue

                gripper_val = float(msg.get("gripper", 0.0))
                gripper_val = max(0.0, min(1.0, gripper_val))  # 裁剪到 [0,1]

                # 将 GELLO 夹爪值视为一个按钮：大于阈值视为按下
                button_threshold = 0.5
                button_state = 1 if gripper_val >= button_threshold else 0

                # 仅在从未按下 -> 按下的上升沿时切换夹爪状态
                if button_state == 1 and self.last_button_state == 0:
                    self.gripper_toggle_state = 1 - self.gripper_toggle_state  # 0<->1 取反

                self.last_button_state = button_state

                # 根据当前开闭状态设置目标宽度：0=完全闭合, 1=最大张开
                max_open = 0.09
                target_width = max_open * float(self.gripper_toggle_state)

                if self.gripper is not None:
                    try:
                        # 提高速度到 0.3，使夹爪更跟手
                        self.gripper.goto(width=target_width, speed=0.3, force=20.0)
                    except Exception as e:
                        print(f"✗ 夹爪控制失败: {e}")

                # 读取 Franka 实际宽度，归一化（仅用于日志）
                franka_norm = 0.0
                if self.gripper is not None:
                    try:
                        state = self.gripper.get_state()
                        franka_norm = state.width / max_open
                    except Exception as e:
                        print(f"✗ 读取夹爪状态失败: {e}")

                # 记录日志
                if self.logger is not None:
                    self.logger.log(time.time() - start_time, gripper_val, target_width)

                count += 1

                now = time.time()
                if now - last_print >= 1.0:
                    elapsed = now - start_time
                    freq = count / elapsed if elapsed > 0 else 0.0
                    print(f"[{elapsed:6.1f}s] 已处理: {count:5d} | 频率: {freq:5.1f} Hz | gripper={gripper_val:.3f} -> width={target_width:.3f} (Franka={franka_norm:.3f})")
                    last_print = now

        except KeyboardInterrupt:
            print("\n检测到 Ctrl+C，正在停止...")
        finally:
            self.stop()

    def stop(self):
        print("\n" + "=" * 60)
        print("停止控制端")
        print("=" * 60)

        if self.client_sock is not None:
            try:
                self.client_sock.close()
                print("✓ 已关闭客户端连接")
            except Exception:
                pass

        if self.server_sock is not None:
            try:
                self.server_sock.close()
                print("✓ 已关闭服务器监听")
            except Exception:
                pass

        if self.robot is not None:
            try:
                self.robot.terminate_current_policy()
                print("✓ 已终止机器人控制策略")
            except Exception:
                pass

        # 保存日志
        if self.logger is not None:
            self.logger.save()

        print("\n✓ ArmGripperController 已停止\n")


def main():
    parser = ArgumentParser(description="机械臂 + 夹爪控制端")
    parser.add_argument("--listen-host", default="0.0.0.0", help="监听地址")
    parser.add_argument("--listen-port", type=int, default=6000, help="监听端口")
    parser.add_argument("--robot-ip", default="localhost", help="Polymetis 机器人服务器 IP")
    parser.add_argument("--robot-port", type=int, default=50051, help="Polymetis 机器人服务器端口")
    args = parser.parse_args()

    ctrl = ArmGripperController(
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
    )

    if not ctrl.initialize():
        print("\n✗ 初始化失败，退出\n")
        return

    ctrl.run()


if __name__ == "__main__":
    main()
