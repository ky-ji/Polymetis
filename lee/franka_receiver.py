#!/usr/bin/env python3
"""
Franka 接收端
在 robot 环境中运行，接收 GELLO 数据并控制 Franka 机械臂

使用方法：
    conda activate robot
    python franka_receiver.py
"""

import sys
import time
import socket
import json
import numpy as np
import torch
import time
from argparse import ArgumentParser
from datetime import datetime
import csv
import os
import threading
import matplotlib
matplotlib.use('Agg')  # 无GUI后端
import matplotlib.pyplot as plt

try:
    import polymetis
    from polymetis import RobotInterface
    from polymetis import GripperInterface
    print("✓ Polymetis 库导入成功")
except ImportError as e:
    print(f"✗ 无法导入 Polymetis 库: {e}")
    print("请确保在 robot conda 环境中运行")
    sys.exit(1)


class TrajectoryLogger:
    """轨迹记录器，用于调试异响问题"""
    
    def __init__(self, log_dir=None):
        # 创建日志目录
        if log_dir is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_dir = f"trajectory_logs/run_{timestamp}"
        
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        
        self.csv_path = os.path.join(self.log_dir, "trajectory.csv")
        self.data = []
        self.prev_target = None
        self.prev_actual = None
        self.prev_time = None
        print(f"✓ 轨迹记录器已启动")
        print(f"  日志目录: {self.log_dir}")
    
    def log(self, timestamp, target_joints, actual_joints):
        """记录一帧数据"""
        # 计算速度（关节变化率）
        target_vel = np.zeros(7)
        actual_vel = np.zeros(7)
        target_acc = np.zeros(7)
        actual_acc = np.zeros(7)
        
        if self.prev_target is not None and self.prev_time is not None:
            dt = timestamp - self.prev_time
            if dt > 0:
                target_vel = (target_joints - self.prev_target) / dt
                actual_vel = (actual_joints - self.prev_actual) / dt
        
        # 记录数据
        row = {
            'time': timestamp,
            'dt': timestamp - self.prev_time if self.prev_time else 0,
        }
        
        # 目标位置和速度
        for i in range(7):
            row[f'target_q{i+1}'] = target_joints[i]
            row[f'target_v{i+1}'] = target_vel[i]
        
        # 实际位置和速度
        for i in range(7):
            row[f'actual_q{i+1}'] = actual_joints[i]
            row[f'actual_v{i+1}'] = actual_vel[i]
        
        # 位置误差
        for i in range(7):
            row[f'error_q{i+1}'] = target_joints[i] - actual_joints[i]
        
        # 最大速度和加速度
        row['max_target_vel'] = np.max(np.abs(target_vel))
        row['max_actual_vel'] = np.max(np.abs(actual_vel))
        
        self.data.append(row)
        self.prev_target = target_joints.copy()
        self.prev_actual = actual_joints.copy()
        self.prev_time = timestamp
    
    def save(self):
        """保存CSV文件和绘制分析图"""
        if not self.data:
            print("⚠️  没有数据需要保存")
            return
        
        # 1. 保存CSV
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.data[0].keys())
            writer.writeheader()
            writer.writerows(self.data)
        
        print(f"✓ 轨迹数据已保存: {self.csv_path} ({len(self.data)} 帧)")
        
        # 2. 生成分析图
        self._plot_analysis()
        
        # 3. 打印统计信息
        max_vels = [row['max_actual_vel'] for row in self.data]
        print(f"  最大实际速度: {np.max(max_vels):.3f} rad/s")
        print(f"  平均实际速度: {np.mean(max_vels):.3f} rad/s")
        print(f"  数据帧数: {len(self.data)}")
        print(f"\n所有文件已保存到: {self.log_dir}")
    
    def _plot_analysis(self):
        """绘制分析图表"""
        import pandas as pd
        df = pd.DataFrame(self.data)
        
        # 图1: 所有关节位置
        fig, axes = plt.subplots(7, 1, figsize=(12, 14))
        fig.suptitle('Joint Positions Over Time', fontsize=16)
        for i in range(7):
            axes[i].plot(df['time'], df[f'target_q{i+1}'], 'b-', label='Target', linewidth=1)
            axes[i].plot(df['time'], df[f'actual_q{i+1}'], 'r--', label='Actual', linewidth=1)
            axes[i].set_ylabel(f'J{i+1} (rad)')
            axes[i].legend(loc='upper right')
            axes[i].grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.savefig(os.path.join(self.log_dir, '1_joint_positions.png'), dpi=150)
        plt.close()
        
        # 图2: 所有关节速度
        fig, axes = plt.subplots(7, 1, figsize=(12, 14))
        fig.suptitle('Joint Velocities Over Time', fontsize=16)
        for i in range(7):
            axes[i].plot(df['time'], df[f'actual_v{i+1}'], 'g-', linewidth=1)
            axes[i].axhline(y=2.0, color='r', linestyle='--', label='Safe limit (2.0 rad/s)')
            axes[i].axhline(y=-2.0, color='r', linestyle='--')
            axes[i].set_ylabel(f'J{i+1} vel (rad/s)')
            axes[i].legend(loc='upper right')
            axes[i].grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.savefig(os.path.join(self.log_dir, '2_joint_velocities.png'), dpi=150)
        plt.close()
        
        # 图3: 最大速度时间序列
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(df['time'], df['max_actual_vel'], 'b-', linewidth=2)
        ax.axhline(y=2.0, color='r', linestyle='--', label='Safe limit (2.0 rad/s)', linewidth=2)
        ax.fill_between(df['time'], 0, df['max_actual_vel'], where=(df['max_actual_vel']>2.0), 
                        color='red', alpha=0.3, label='Exceeds limit')
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Max Velocity (rad/s)', fontsize=12)
        ax.set_title('Maximum Joint Velocity Over Time', fontsize=14)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(self.log_dir, '3_max_velocity.png'), dpi=150)
        plt.close()
        
        # 图4: 位置误差
        fig, axes = plt.subplots(7, 1, figsize=(12, 14))
        fig.suptitle('Position Tracking Errors', fontsize=16)
        for i in range(7):
            axes[i].plot(df['time'], df[f'error_q{i+1}'], 'r-', linewidth=1)
            axes[i].set_ylabel(f'J{i+1} error (rad)')
            axes[i].grid(True, alpha=0.3)
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.savefig(os.path.join(self.log_dir, '4_tracking_errors.png'), dpi=150)
        plt.close()
        
        # 图5: 控制频率（时间间隔）
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.plot(df['time'][1:], df['dt'][1:]*1000, 'b-', linewidth=1)  # 转换为毫秒
        ax.axhline(y=33.3, color='g', linestyle='--', label='Target: 30 Hz (33.3 ms)', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Control Period (ms)', fontsize=12)
        ax.set_title('Control Loop Timing', fontsize=14)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(self.log_dir, '5_control_timing.png'), dpi=150)
        plt.close()
        
        print(f"✓ 分析图表已保存 (5张图片)")
        
        # 生成README说明文件
        self._generate_readme(max_vels)
    
    def _generate_readme(self, max_vels):
        """生成说明文件"""
        readme_path = os.path.join(self.log_dir, 'README.md')
        
        with open(readme_path, 'w', encoding='utf-8') as f:
            f.write(f"# 轨迹记录分析报告\n\n")
            f.write(f"记录时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write(f"## 统计信息\n\n")
            f.write(f"- **数据帧数**: {len(self.data)}\n")
            f.write(f"- **记录时长**: {self.data[-1]['time']:.2f} 秒\n")
            f.write(f"- **最大速度**: {np.max(max_vels):.3f} rad/s\n")
            f.write(f"- **平均速度**: {np.mean(max_vels):.3f} rad/s\n")
            
            # 检查是否超速
            over_limit = [v for v in max_vels if v > 2.0]
            if over_limit:
                f.write(f"- **⚠️ 警告**: 检测到 {len(over_limit)} 次超速 (>2.0 rad/s)\n")
            else:
                f.write(f"- **✓ 正常**: 所有速度均在安全范围内\n")
            
            f.write(f"\n## 文件说明\n\n")
            f.write(f"### 数据文件\n")
            f.write(f"- `trajectory.csv`: 原始轨迹数据，包含目标位置、实际位置、速度、误差等\n\n")
            
            f.write(f"### 分析图表\n")
            f.write(f"1. `1_joint_positions.png`: 7个关节的位置曲线（蓝色=目标，红色=实际）\n")
            f.write(f"2. `2_joint_velocities.png`: 7个关节的速度曲线（绿色线，红色虚线为安全限制）\n")
            f.write(f"3. `3_max_velocity.png`: 最大关节速度时间序列（红色区域为超限）\n")
            f.write(f"4. `4_tracking_errors.png`: 7个关节的跟踪误差（目标-实际）\n")
            f.write(f"5. `5_control_timing.png`: 控制循环时间间隔（应接近33.3ms）\n\n")
            
            f.write(f"## 异响诊断建议\n\n")
            
            if np.max(max_vels) > 2.0:
                f.write(f"### ⚠️ 发现高速运动\n")
                f.write(f"- 最大速度 {np.max(max_vels):.3f} rad/s 超过安全限制 2.0 rad/s\n")
                f.write(f"- **可能原因**: 运动幅度过大、缩放系数设置过高\n")
                f.write(f"- **建议**: 降低 `--scale` 参数（当前1.0，建议0.5-0.7）\n\n")
            
            # 检查位置误差
            max_errors = []
            for i in range(7):
                errors = [abs(row[f'error_q{i+1}']) for row in self.data]
                max_errors.append(np.max(errors))
            
            if np.max(max_errors) > 0.1:
                f.write(f"### ⚠️ 发现大跟踪误差\n")
                f.write(f"- 最大误差 {np.max(max_errors):.3f} rad\n")
                f.write(f"- **可能原因**: 机器人跟不上目标、控制频率不足\n")
                f.write(f"- **建议**: 降低运动速度或增加控制频率\n\n")
            
            f.write(f"### 查看详细数据\n")
            f.write(f"```python\n")
            f.write(f"import pandas as pd\n")
            f.write(f"import matplotlib.pyplot as plt\n\n")
            f.write(f"# 读取数据\n")
            f.write(f"df = pd.read_csv('trajectory.csv')\n\n")
            f.write(f"# 查看超速时刻\n")
            f.write(f"high_vel = df[df['max_actual_vel'] > 2.0]\n")
            f.write(f"print(high_vel[['time', 'max_actual_vel']])\n")
            f.write(f"```\n")
        
        print(f"✓ 说明文件已生成: README.md")


class FrankaReceiver:
    """Franka 数据接收端"""
    
    def __init__(self, 
                 listen_host: str = "0.0.0.0",
                 listen_port: int = 5555,
                 robot_ip: str = "localhost",
                 robot_port: int = 50051,
                 position_scale: float = 1.0,
                 enable_logging: bool = False):
        """
        初始化 Franka 接收端
        
        Args:
            listen_host: Socket 监听地址
            listen_port: Socket 监听端口
            robot_ip: Polymetis 服务器 IP
            robot_port: Polymetis 服务器端口
            position_scale: 位置映射缩放因子（0.1-1.0，越小越安全）
        """
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.position_scale = position_scale
        
        self.server_socket = None
        self.client_socket = None
        self.robot = None
        self.gripper = None
        self.initial_gello_joints = None
        self.initial_robot_joints = None
        self.recv_buffer = b""  # 接收缓冲区
        
        # 轨迹记录器
        self.logger = TrajectoryLogger() if enable_logging else None
        
        # 控制线程相关
        self.current_target = None  # 当前目标位置
        self.target_lock = threading.Lock()  # 线程锁
        self.control_thread = None
        self.running = False
        
        # 夹爪开关逻辑：0=闭合, 1=张开；基于按钮上升沿取反
        self.gripper_toggle_state = 0
        self.last_gripper_button_state = 0
        
        # 关节偏置：用于整体调整工作区（例如略微降低末端高度）
        # 这里给一个保守初始值，主要作用在第2、3关节，你可以根据需要调整
        self.joint_offset = np.array([0.0, 0.10, -0.10, 0.0, 0.0, 0.0, 0.0])
        
        # FR3 GELLO 配置（来自 gello_software）
        # 关节顺序: [1:肩旋转(左右), 2:肩上下, 3:肘(前后), 4:腕旋转1, 5:腕旋转2, 6:腕上下, 7:末端旋转]
        # joint_signs: 关节4、6 为 -1，其余为 1
        self.gello_joint_signs = np.array([1, 1, 1, 1, 1, 1, 1])
        
        print("\n" + "="*60)
        print("Franka 数据接收端")
        print("="*60)
        print(f"监听地址: {listen_host}:{listen_port}")
        print(f"机器人服务器: {robot_ip}:{robot_port}")
        print(f"位置缩放: {position_scale}")
        print("="*60 + "\n")
    
    def initialize(self) -> bool:
        """初始化机器人和网络服务"""
        # 1. 连接机器人
        print("[1/2] 连接到 Polymetis 服务器...")
        try:
            self.robot = RobotInterface(
                ip_address=self.robot_ip,
                port=self.robot_port
            )
            print("✓ 已连接到机器人")
            
            # 尝试连接夹爪服务器
            try:
                self.gripper = GripperInterface(
                    ip_address=self.robot_ip,
                    port=50052  # 夹爪服务器默认端口
                )
                print("✓ 已连接到夹爪")
            except Exception as e:
                self.gripper = None
                print(f"ℹ️  夹爪服务器未启动，夹爪控制已禁用")
                print(f"   (如需使用夹爪，请用 --gripper 参数启动服务器)")
            
            # 读取当前关节位置（作为初始位置）
            self.initial_robot_joints = self.robot.get_joint_positions().numpy()
            print(f"  机器人当前关节: {np.round(self.initial_robot_joints, 3)}")
            print("  提示：操作前请确保机械臂在安全位置")
        except Exception as e:
            print(f"✗ 连接机器人失败: {e}")
            print("\n请确保:")
            print("  1. Polymetis 服务器已启动 (python start_server.py)")
            print("  2. 服务器地址正确")
            return False
        
        # 2. 启动 Socket 服务器
        print("\n[2/2] 启动 Socket 服务器...")
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.listen_host, self.listen_port))
            self.server_socket.listen(1)
            print(f"✓ Socket 服务器已启动，监听 {self.listen_host}:{self.listen_port}")
            print("\n等待 GELLO 发送端连接...")
            
            # 等待客户端连接
            self.client_socket, client_address = self.server_socket.accept()
            print(f"✓ GELLO 发送端已连接: {client_address}")
            
            # 接收初始化数据
            init_data = self._receive_data()
            if init_data and init_data.get("type") == "init":
                gello_joints_all = np.array(init_data["initial_joints"])
                print(f"  GELLO 初始关节: {np.round(gello_joints_all, 3)}")
                
                # 只取前7个关节（机械臂），忽略夹爪
                self.initial_gello_joints = gello_joints_all[:7]
                
                # 检查自由度（只检查机械臂关节）
                if len(self.initial_gello_joints) != len(self.initial_robot_joints):
                    print(f"✗ 自由度不匹配: GELLO={len(self.initial_gello_joints)}, Robot={len(self.initial_robot_joints)}")
                    return False
                
                print(f"✓ 自由度匹配: {len(self.initial_robot_joints)} DOF (机械臂)")
                if len(gello_joints_all) > 7:
                    print(f"✓ 夹爪数据: 1 DOF")
            else:
                print("✗ 未收到初始化数据")
                return False
                
        except Exception as e:
            print(f"✗ 启动 Socket 服务器失败: {e}")
            return False
        
        print("\n" + "="*60)
        print("✓ 初始化完成")
        print("="*60 + "\n")
        
        # 3. 启动关节阻抗控制器
        print("启动关节阻抗控制器...")
        try:
            self.robot.start_joint_impedance()
            print("✓ 控制器已启动，等待就绪...")
            
            # 等待并重试激活控制器
            current_pos = self.robot.get_joint_positions()
            activated = False
            for i in range(10):  # 最多重试10次
                time.sleep(0.5)  # 每次等待0.5秒
                try:
                    self.robot.update_desired_joint_positions(current_pos)
                    print("✓ 控制器就绪并激活")
                    activated = True
                    break
                except Exception as e:
                    if i < 9:
                        print(f"  等待控制器就绪... ({i+1}/10)")
                    else:
                        raise Exception(f"控制器启动超时: {e}")
            
            if not activated:
                raise Exception("控制器未能激活")
            
            # 初始化目标位置
            with self.target_lock:
                self.current_target = current_pos.numpy()
            
            # 启动后台控制线程
            self.running = True
            self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()
            print("✓ 后台控制线程已启动")
            
            # 发送 ready 信号给 GELLO 端
            ready_signal = {"type": "ready"}
            self._send_data(ready_signal)
            print("✓ 已发送 ready 信号给 GELLO 端\n")
        except Exception as e:
            print(f"✗ 启动控制器失败: {e}")
            return False
        
        return True
    
    def _control_loop(self):
        """后台控制循环，持续发送控制命令（避免通信约束违规）"""
        print("后台控制循环已启动，频率: 30 Hz")
        rate = 1.0 / 30.0  # 30 Hz
        
        while self.running:
            try:
                loop_start = time.time()
                
                # 获取当前目标位置
                with self.target_lock:
                    if self.current_target is not None:
                        target = torch.from_numpy(self.current_target).float()
                        self.robot.update_desired_joint_positions(target)
                
                # 控制循环频率
                elapsed = time.time() - loop_start
                sleep_time = rate - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                     
            except Exception as e:
                if not self.running:
                    break

                err_msg = str(e)
                print(f"控制循环错误: {err_msg}")

                # 如果控制器被服务器终止，尝试自动重启关节阻抗控制器
                if "no controller running" in err_msg or "start_joint_impedance" in err_msg:
                    try:
                        print("[CONTROL] 检测到控制器未运行，尝试重新启动关节阻抗控制器...")
                        self.robot.start_joint_impedance()
                        current_pos = self.robot.get_joint_positions()
                        with self.target_lock:
                            self.current_target = current_pos.numpy()
                        print("[CONTROL] 关节阻抗控制器已重新启动并激活")
                        # 重启后继续循环
                        continue
                    except Exception as e_restart:
                        print(f"[CONTROL] 重新启动关节阻抗控制器失败: {e_restart}")
                        break
                else:
                    # 其他未知错误，退出循环，避免疯狂重试
                    break
    
    def _send_data(self, data: dict):
        """发送 JSON 数据"""
        json_str = json.dumps(data)
        message = (json_str + "\n").encode('utf-8')
        self.client_socket.sendall(message)
    
    def _receive_data(self) -> dict:
        """接收 JSON 数据"""
        try:
            while True:
                # 检查缓冲区是否有完整的消息
                if b"\n" in self.recv_buffer:
                    line, self.recv_buffer = self.recv_buffer.split(b"\n", 1)
                    return json.loads(line.decode('utf-8'))
                
                # 接收更多数据
                chunk = self.client_socket.recv(4096)
                if not chunk:
                    return None
                self.recv_buffer += chunk
        except Exception as e:
            print(f"✗ 接收数据失败: {e}")
            return None
    
    def gello_to_robot_joints(self, gello_joints: np.ndarray) -> np.ndarray:
        """
        将 GELLO 关节角度映射到机器人关节角度
        
        Args:
            gello_joints: GELLO 当前关节角度
            
        Returns:
            机器人目标关节角度
        """
        # 应用 GELLO 关节符号（第6个关节是反向的）
        gello_joints_corrected = gello_joints * self.gello_joint_signs
        initial_gello_corrected = self.initial_gello_joints * self.gello_joint_signs
        
        # 计算 GELLO 相对初始位置的偏移
        gello_delta = gello_joints_corrected - initial_gello_corrected
        
        # 应用缩放因子
        scaled_delta = gello_delta * self.position_scale
        
        # 应用到机器人初始位置
        robot_target = self.initial_robot_joints + scaled_delta
        
        # 加上固定关节偏置（整体调整工作区，例如略微降低末端）
        robot_target = robot_target + self.joint_offset
        
        # 关节限位（放宽限制以便调试，接近硬件限位）
        # 原始安全限位：[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
        #              [ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]
        joint_limits_lower = np.array([-2.85, -1.75, -2.85, -3.05, -2.85, -0.01, -2.85])
        joint_limits_upper = np.array([2.85, 1.75, 2.85, -0.08, 2.85, 3.70, 2.85])
        
        robot_target = np.clip(robot_target, joint_limits_lower, joint_limits_upper)
        
        return robot_target
    
    def run(self):
        """运行接收和控制循环"""
        print("\n" + "="*60)
        print("开始接收并控制")
        print("="*60)
        print("\n按 Ctrl+C 停止...")
        print("="*60 + "\n")
        
        start_time = time.time()
        receive_count = 0
        last_print_time = start_time
        
        try:
            while True:
                # 1. 接收 GELLO 数据
                data = self._receive_data()
                
                if not data:
                    print("\n✗ 连接断开")
                    break
                
                # 2. 处理不同类型的消息
                if data["type"] == "stop":
                    print("\n收到停止信号")
                    break
                
                elif data["type"] == "joint_state":
                    gello_joints = np.array(data["joints"])
                    
                    # 分离关节和夹爪数据
                    if len(gello_joints) >= 8:
                        robot_joints = gello_joints[:7]  # 前7个关节
                        gripper_pos = float(gello_joints[7])    # 第8个视为按钮/夹爪值 (0~1)
                    else:
                        robot_joints = gello_joints[:7]
                        gripper_pos = None
                    
                    # 3. 转换为机器人目标
                    robot_target = self.gello_to_robot_joints(robot_joints)
                    
                    # 4. 更新目标位置（后台线程会持续发送）
                    with self.target_lock:
                        self.current_target = robot_target
                    
                    # 记录轨迹数据
                    if self.logger:
                        current_joints = self.robot.get_joint_positions().numpy()
                        self.logger.log(time.time() - start_time, robot_target, current_joints)
                    
                    # 夹爪控制（如果夹爪服务器已启动）
                    if self.gripper is not None and gripper_pos is not None:
                        # 将 GELLO 的第 8 维视为按钮：大于阈值视为按下
                        button_threshold = 0.5
                        button_state = 1 if gripper_pos >= button_threshold else 0

                        # 仅在未按下 -> 按下的上升沿时切换夹爪状态
                        if button_state == 1 and self.last_gripper_button_state == 0:
                            self.gripper_toggle_state = 1 - self.gripper_toggle_state  # 0<->1 取反

                        self.last_gripper_button_state = button_state

                        # 根据当前状态设置目标宽度：0=闭合, 1=最大张开
                        max_open = 0.09
                        gripper_width = max_open * float(self.gripper_toggle_state)

                        try:
                            self.gripper.goto(
                                width=gripper_width,
                                speed=0.3,
                                force=5.0,
                            )
                        except Exception:
                            # 夹爪错误不影响主控制
                            pass
                    
                    receive_count += 1
                    
                    # 6. 显示状态（每秒一次）
                    current_time = time.time()
                    if current_time - last_print_time >= 1.0:
                        elapsed = current_time - start_time
                        avg_rate = receive_count / elapsed if elapsed > 0 else 0
                        current_joints = self.robot.get_joint_positions().numpy()
                        
                        print(f"[{elapsed:6.1f}s] 已接收: {receive_count:5d} | "
                              f"频率: {avg_rate:5.1f} Hz | "
                              f"目标: {np.round(robot_target[:3], 3)} | "
                              f"当前: {np.round(current_joints[:3], 3)}")
                        
                        last_print_time = current_time
                
        except KeyboardInterrupt:
            print("\n\n检测到 Ctrl+C，正在停止...")
        finally:
            self.stop(start_time, receive_count)
    
    def stop(self, start_time: float, receive_count: int):
        """停止控制"""
        print("\n" + "="*60)
        print("停止控制")
        print("="*60)
        
        # 停止后台控制线程
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
            print("✓ 后台控制线程已停止")
        
        # 停止机器人运动
        if self.robot:
            try:
                # 停止控制器
                self.robot.terminate_current_policy()
                print("✓ 机器人已停止")
            except:
                pass
        
        # 关闭连接
        if self.client_socket:
            self.client_socket.close()
            print("✓ 已关闭客户端连接")
        
        if self.server_socket:
            self.server_socket.close()
            print("✓ 已关闭服务器")
        
        # 保存轨迹日志
        if self.logger:
            print()
            self.logger.save()
        
        # 统计信息
        if start_time:
            total_time = time.time() - start_time
            avg_rate = receive_count / total_time if total_time > 0 else 0
            print(f"\n统计信息:")
            print(f"  总运行时间: {total_time:.2f} 秒")
            print(f"  总接收次数: {receive_count}")
            print(f"  平均频率: {avg_rate:.1f} Hz")
        
        print("\n✓ 接收端已停止\n")


def main():
    """主函数"""
    parser = ArgumentParser(description='Franka 数据接收端')
    parser.add_argument('--listen-host', default='0.0.0.0',
                       help='Socket 监听地址')
    parser.add_argument('--listen-port', type=int, default=5555,
                       help='Socket 监听端口')
    parser.add_argument('--robot-ip', default='localhost',
                       help='Polymetis 服务器 IP')
    parser.add_argument('--robot-port', type=int, default=50051,
                       help='Polymetis 服务器端口')
    parser.add_argument('--scale', type=float, default=1.0,
                       help='位置缩放因子 (0.1-1.0，1.0为一比一映射)')
    parser.add_argument('--log', action='store_true',
                       help='启用轨迹记录（用于调试异响问题）')
    
    args = parser.parse_args()
    
    # 创建接收端
    receiver = FrankaReceiver(
        listen_host=args.listen_host,
        listen_port=args.listen_port,
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        position_scale=args.scale,
        enable_logging=args.log
    )
    
    # 初始化
    if not receiver.initialize():
        print("\n✗ 初始化失败，退出\n")
        return
    
    # 运行
    receiver.run()


if __name__ == '__main__':
    main()
