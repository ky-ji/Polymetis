#!/usr/bin/env python3
"""
GELLO 发送端
在 gello 环境中运行，读取 GELLO 数据并通过 socket 发送

使用方法：
    conda activate gello
    python gello_sender.py
"""

import sys
import time
import json
import socket
import numpy as np
from pathlib import Path
from argparse import ArgumentParser

# 添加 gello 路径
gello_path = Path("/home/kyji/Desktop/Workspace/robot/gello_software")
sys.path.insert(0, str(gello_path))

try:
    from gello.agents.gello_agent import GelloAgent
    print("✓ GELLO 库导入成功")
except ImportError as e:
    print(f"✗ 无法导入 GELLO 库: {e}")
    print(f"请确保路径正确: {gello_path}")
    sys.exit(1)


class GelloSender:
    """GELLO 数据发送端"""
    
    def __init__(self, 
                 gello_port: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAW4VCW-if00-port0",
                 server_host: str = "localhost",
                 server_port: int = 5555):
        """
        初始化 GELLO 发送端
        
        Args:
            gello_port: GELLO 设备端口
            server_host: Socket 服务器地址
            server_port: Socket 服务器端口
        """
        self.gello_port = gello_port
        self.server_host = server_host
        self.server_port = server_port
        
        self.gello = None
        self.socket = None
        self.initial_joints = None
        self.recv_buffer = b""  # 接收缓冲区
        
        print("\n" + "="*60)
        print("GELLO 数据发送端")
        print("="*60)
        print(f"GELLO 端口: {gello_port}")
        print(f"服务器地址: {server_host}:{server_port}")
        print("="*60 + "\n")
    
    def initialize(self) -> bool:
        """初始化 GELLO 设备和网络连接"""
        # 1. 初始化 GELLO
        print("[1/2] 初始化 GELLO 设备...")
        try:
            self.gello = GelloAgent(port=self.gello_port)
            print("✓ GELLO 设备已连接")
            
            # 读取初始位置（包括夹爪，共8个关节）
            self.initial_joints = self.gello.act({})[:8]  # 取前8个关节（7个机械臂+1个夹爪）
            print(f"  初始关节(含夹爪): {np.round(self.initial_joints, 3)}")
        except Exception as e:
            print(f"✗ GELLO 初始化失败: {e}")
            return False
        
        # 2. 连接到服务器
        print("\n[2/2] 连接到 Franka 控制端...")
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_host, self.server_port))
            print(f"✓ 已连接到 {self.server_host}:{self.server_port}")
            
            # 发送初始化数据
            init_data = {
                "type": "init",
                "initial_joints": self.initial_joints.tolist(),
                "num_dof": len(self.initial_joints)
            }
            self._send_data(init_data)
            print("✓ 已发送初始化数据")
            
            # 等待 Franka 端准备完成
            print("\n等待 Franka 端准备就绪...")
            ready_signal = self._receive_data()
            if not ready_signal or ready_signal.get("type") != "ready":
                print("✗ 未收到 ready 信号")
                return False
            print("✓ Franka 端已就绪")
            
        except Exception as e:
            print(f"✗ 连接服务器失败: {e}")
            print("\n请确保:")
            print("  1. Franka 控制端已启动 (python franka_receiver.py)")
            print("  2. 服务器地址正确")
            return False
        
        print("\n" + "="*60)
        print("✓ 初始化完成")
        print("="*60 + "\n")
        return True
    
    def _send_data(self, data: dict):
        """发送 JSON 数据"""
        json_str = json.dumps(data)
        # 添加换行符作为消息分隔符
        message = (json_str + "\n").encode('utf-8')
        self.socket.sendall(message)
    
    def _receive_data(self) -> dict:
        """接收 JSON 数据"""
        try:
            while True:
                # 检查缓冲区是否有完整的消息
                if b"\n" in self.recv_buffer:
                    line, self.recv_buffer = self.recv_buffer.split(b"\n", 1)
                    return json.loads(line.decode('utf-8'))
                
                # 接收更多数据
                chunk = self.socket.recv(4096)
                if not chunk:
                    return None
                self.recv_buffer += chunk
        except Exception as e:
            print(f"✗ 接收数据失败: {e}")
            return None
    
    def run(self, update_rate: float = 30.0):
        """
        运行数据采集和发送
        
        Args:
            update_rate: 发送频率（Hz）
        """
        print("\n" + "="*60)
        print("开始发送 GELLO 数据")
        print("="*60)
        print(f"发送频率: {update_rate} Hz")
        print("\n按 Ctrl+C 停止...")
        print("="*60 + "\n")
        
        start_time = time.time()
        dt = 1.0 / update_rate
        send_count = 0
        last_print_time = start_time
        
        try:
            while True:
                loop_start = time.time()
                
                # 1. 读取 GELLO 关节角度（包括夹爪）
                gello_joints = self.gello.act({})[:8]

                # 将第 8 个关节视为按钮：大于阈值视为按下，发送 0/1
                if gello_joints.shape[0] >= 8:
                    raw_gripper = gello_joints[7]
                    button_threshold = 0.5
                    gello_joints[7] = 1.0 if raw_gripper >= button_threshold else 0.0
                
                # 2. 构建数据包
                data = {
                    "type": "joint_state",
                    "timestamp": time.time(),
                    "joints": gello_joints.tolist()
                }
                
                # 3. 发送数据
                try:
                    self._send_data(data)
                    send_count += 1
                except Exception as e:
                    print(f"\n✗ 发送失败: {e}")
                    break
                
                # 4. 显示状态（每秒一次）
                current_time = time.time()
                if current_time - last_print_time >= 1.0:
                    elapsed = current_time - start_time
                    avg_rate = send_count / elapsed if elapsed > 0 else 0
                    
                    print(f"[{elapsed:6.1f}s] 已发送: {send_count:5d} | "
                          f"频率: {avg_rate:5.1f} Hz | "
                          f"关节: {np.round(gello_joints[:3], 3)}")
                    
                    last_print_time = current_time
                
                # 5. 控制频率
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
                
        except KeyboardInterrupt:
            print("\n\n检测到 Ctrl+C，正在停止...")
        finally:
            self.stop(start_time, send_count)
    
    def stop(self, start_time: float, send_count: int):
        """停止发送"""
        print("\n" + "="*60)
        print("停止发送")
        print("="*60)
        
        # 发送停止信号
        if self.socket:
            try:
                stop_data = {"type": "stop"}
                self._send_data(stop_data)
                print("✓ 已发送停止信号")
            except:
                pass
        
        # 关闭连接
        if self.socket:
            self.socket.close()
            print("✓ 已关闭连接")
        
        # 统计信息
        if start_time:
            total_time = time.time() - start_time
            avg_rate = send_count / total_time if total_time > 0 else 0
            print(f"\n统计信息:")
            print(f"  总运行时间: {total_time:.2f} 秒")
            print(f"  总发送次数: {send_count}")
            print(f"  平均频率: {avg_rate:.1f} Hz")
        
        print("\n✓ 发送端已停止\n")


def main():
    """主函数"""
    parser = ArgumentParser(description='GELLO 数据发送端')
    parser.add_argument('--gello-port', 
                       default="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAW4VCW-if00-port0",
                       help='GELLO 设备端口')
    parser.add_argument('--host', default='localhost',
                       help='Franka 控制端地址')
    parser.add_argument('--port', type=int, default=5555,
                       help='Franka 控制端端口')
    parser.add_argument('--rate', type=int, default=30,
                       help='发送频率 (Hz)')
    
    args = parser.parse_args()
    
    # 创建发送端
    sender = GelloSender(
        gello_port=args.gello_port,
        server_host=args.host,
        server_port=args.port
    )
    
    # 初始化
    if not sender.initialize():
        print("\n✗ 初始化失败，退出\n")
        return
    
    # 运行
    sender.run(update_rate=args.rate)


if __name__ == '__main__':
    main()
