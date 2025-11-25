#!/usr/bin/env python3
"""
键盘控制 Franka 机器人

使用方法：
    1. 先启动服务器: python start_server.py
    2. 运行此脚本: python keyboard_control.py
    
键盘控制：
    W - 向前 (X增加)
    S - 向后 (X减少)
    A - 向左 (Y减少)
    D - 向右 (Y增加)
    空格 - 暂停/继续
    Q 或 ESC - 退出

提示: 持续按住按键，机械臂会持续移动
"""
from typing import Dict
import torch
import time
import sys
import select
import termios
import tty
from threading import Lock
from polymetis import RobotInterface
import torchcontrol as toco


class CartesianPDPolicy(toco.PolicyModule):
    """
    笛卡尔空间PD控制策略，可以实时更新目标位置
    """
    def __init__(self, robot_model, joint_pos_current, Kp, Kd, ignore_gravity=True, **kwargs):
        super().__init__(**kwargs)
        
        # 初始化模块
        self.robot_model = robot_model
        self.invdyn = toco.modules.feedforward.InverseDynamics(
            self.robot_model, ignore_gravity=ignore_gravity
        )
        self.pose_pd = toco.modules.feedback.CartesianSpacePDFast(Kp, Kd)
        
        # 计算初始末端位置
        joint_pos_current = joint_pos_current if isinstance(joint_pos_current, torch.Tensor) else torch.Tensor(joint_pos_current)
        ee_pos_current, ee_quat_current = self.robot_model.forward_kinematics(joint_pos_current)
        
        # 目标位置和姿态（可更新）
        self.ee_pos_desired = torch.nn.Parameter(ee_pos_current)
        self.ee_quat_desired = torch.nn.Parameter(ee_quat_current)
        self.ee_vel_desired = torch.nn.Parameter(torch.zeros(3))
        self.ee_rvel_desired = torch.nn.Parameter(torch.zeros(3))
    
    def forward(self, state_dict: Dict[str, torch.Tensor]):
        # 状态提取
        joint_pos_current = state_dict["joint_positions"]
        joint_vel_current = state_dict["joint_velocities"]
        
        # 计算当前末端位置和速度
        ee_pos_current, ee_quat_current = self.robot_model.forward_kinematics(joint_pos_current)
        jacobian = self.robot_model.compute_jacobian(joint_pos_current)
        ee_twist_current = jacobian @ joint_vel_current
        
        # PD控制
        wrench_feedback = self.pose_pd(
            ee_pos_current,
            ee_quat_current,
            ee_twist_current,
            self.ee_pos_desired,
            self.ee_quat_desired,
            torch.cat([self.ee_vel_desired, self.ee_rvel_desired])
        )
        torque_feedback = jacobian.T @ wrench_feedback
        
        # 前馈补偿（科氏力）
        torque_feedforward = self.invdyn(
            joint_pos_current, joint_vel_current, torch.zeros_like(joint_pos_current)
        )
        
        torque_out = torque_feedback + torque_feedforward
        
        return {"joint_torques": torque_out}


class KeyboardController:
    def __init__(self, robot_ip="localhost"):
        """初始化键盘控制器"""
        print("=" * 70)
        print(" " * 20 + "Franka 键盘控制")
        print("=" * 70)
        print()
        print(f"连接到机器人: {robot_ip}")
        
        # 连接机器人
        self.robot = RobotInterface(ip_address=robot_ip)
        print("✓ 连接成功!")
        print()
        
        # 获取当前末端位置
        ee_pos, ee_quat = self.robot.get_ee_pose()
        print(f"当前末端位置: X={ee_pos[0]:.3f}, Y={ee_pos[1]:.3f}, Z={ee_pos[2]:.3f}")
        
        # 工作空间限制 (单位: 米)
        self.workspace_limits = {
            'x_min': 0.20,
            'x_max': 0.80,
            'y_min': -0.50,
            'y_max': 0.50,
            'z_fixed': ee_pos[2].item()  # 固定高度，不改变
        }
        
        print(f"工作空间限制:")
        print(f"  X: [{self.workspace_limits['x_min']}, {self.workspace_limits['x_max']}] m")
        print(f"  Y: [{self.workspace_limits['y_min']}, {self.workspace_limits['y_max']}] m")
        print(f"  Z: 固定于 {self.workspace_limits['z_fixed']:.3f} m")
        print()
        
        # 键盘控制参数
        self.keys_pressed = {
            'w': False,
            's': False,
            'a': False,
            'd': False
        }
        self.key_last_pressed = {}
        self.keys_lock = Lock()
        
        # 移动速度 (m/s)
        self.move_speed = 1.0  # 1.0m/s = 100cm/s
        
        # 调试
        self.debug_log = True
        self.last_update_time = 0
        self.update_count = 0
        
        # 打开日志文件
        import datetime
        log_filename = f"keyboard_control_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        self.log_file = open(log_filename, 'w')
        print(f"日志保存到: {log_filename}")
        self.log_file.write(f"键盘控制日志 - {datetime.datetime.now()}\n")
        self.log_file.write("=" * 70 + "\n")
        self.log_file.flush()
        
        # 按键保持时间（秒）- 使用frankx的两层检测策略
        # 真正超时 = key_hold_timeout * 2
        self.key_hold_timeout = 0.2  # 0.2 * 2 = 0.4秒真正超时
        
        # 状态变量
        self.running = False
        self.paused = True
        
        # 终端设置
        self.old_settings = None
        
        # 策略状态
        self.policy = None
        self.policy_active = False
    
    def init_policy(self):
        """初始化控制策略 - 使用阻抗控制"""
        print("启动笛卡尔阻抗控制...")
        # 使用默认阻抗参数，更稳定
        self.robot.start_cartesian_impedance()
        self.policy_active = True
        time.sleep(0.5)  # 等待激活，延长等待时间
        print("✓ 阻抗控制已激活\n")
    
    def update_target_position(self, dt=0.02):
        """
        根据按键状态更新目标位置
        dt: 时间步长（秒）
        """
        current_time = time.time()
        
        if not self.policy_active:
            return
            
        with self.keys_lock:
            keys = self.keys_pressed.copy()
        
        # 检查是否有按键被按下
        if not any(keys.values()):
            if self.debug_log and self.update_count > 0:
                self.log_file.write(f"[{current_time:.3f}] 按键释放，停止更新\n")
                self.log_file.flush()
                self.update_count = 0
            return
        
        # 根据按键计算位移增量
        dx = 0.0
        dy = 0.0
        
        if keys['w']:  # 向前
            dx += self.move_speed * dt
        if keys['s']:  # 向后
            dx -= self.move_speed * dt
        if keys['a']:  # 向左
            dy -= self.move_speed * dt
        if keys['d']:  # 向右
            dy += self.move_speed * dt
        
        # 如果没有实际移动，跳过
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return
        
        # 记录日志
        self.update_count += 1
        if self.last_update_time > 0:
            interval = current_time - self.last_update_time
            if self.debug_log and (self.update_count % 10 == 1 or interval > 0.05):
                active_keys = [k for k, v in keys.items() if v]
                self.log_file.write(f"[{current_time:.3f}] 更新#{self.update_count}, 间隔={interval*1000:.1f}ms, 按键={active_keys}, dx={dx:.4f}, dy={dy:.4f}\n")
                self.log_file.flush()
        
        self.last_update_time = current_time
        
        # 获取当前末端位置
        ee_pos, ee_quat = self.robot.get_ee_pose()
        
        # 计算新的目标位置
        new_x = ee_pos[0].item() + dx
        new_y = ee_pos[1].item() + dy
        
        # 应用工作空间限制
        new_x = max(self.workspace_limits['x_min'],
                   min(self.workspace_limits['x_max'], new_x))
        new_y = max(self.workspace_limits['y_min'],
                   min(self.workspace_limits['y_max'], new_y))
        
        # 创建新的目标位置（保持Z不变）
        new_pos = torch.Tensor([new_x, new_y, self.workspace_limits['z_fixed']])
        
        # 更新阻抗控制目标
        try:
            self.robot.update_desired_ee_pose(position=new_pos)
        except Exception as e:
            # 如果更新失败，尝试重新启动阻抗控制
            self.log_file.write(f"[{current_time:.3f}] 更新失败: {e}，尝试重启阻抗控制\n")
            self.log_file.flush()
            try:
                self.robot.start_cartesian_impedance()
                time.sleep(0.2)
                self.robot.update_desired_ee_pose(position=new_pos)
            except Exception as e2:
                self.log_file.write(f"[{current_time:.3f}] 重启后仍失败: {e2}\n")
                self.log_file.flush()
    
    def control_loop(self):
        """主控制循环"""
        print("\n开始键盘控制...")
        print("控制说明:")
        print("  W - 向前 (X增加)")
        print("  S - 向后 (X减少)")
        print("  A - 向左 (Y减少)")
        print("  D - 向右 (Y增加)")
        print("  空格 - 暂停/继续")
        print("  Q 或 ESC - 退出")
        print("\n提示: 持续按住按键，机械臂会持续移动\n")
        
        # 设置终端为原始模式
        self.setup_terminal()
        
        try:
            while self.running:
                current_time = time.time()
                
                if not self.paused:
                    # 检查按键（非阻塞）
                    key = self.get_key()
                    if key:
                        key_lower = key.lower()
                        
                        if key_lower == '\x1b' or key_lower == 'q':  # ESC或Q
                            print("\n正在退出...")
                            self.stop()
                            break
                        elif key_lower == ' ':  # 空格
                            self.paused = not self.paused
                            status = "暂停" if self.paused else "继续"
                            print(f"\n{status}控制...")
                        elif key_lower in self.keys_pressed:
                            # WASD键被按下，更新时间戳
                            with self.keys_lock:
                                was_pressed = self.keys_pressed[key_lower]
                                self.keys_pressed[key_lower] = True
                                self.key_last_pressed[key_lower] = current_time
                                if self.debug_log and not was_pressed:
                                    self.log_file.write(f"[{current_time:.3f}] 按键 '{key_lower}' 按下\n")
                                    self.log_file.flush()
                    
                    # 检查按键是否应该保持按下状态（两层超时检测，参考frankx）
                    # 键盘重复间隔约200-300ms，使用2倍超时给予宽容度
                    with self.keys_lock:
                        for k in list(self.keys_pressed.keys()):
                            if self.keys_pressed[k]:
                                if k in self.key_last_pressed:
                                    time_since_press = current_time - self.key_last_pressed[k]
                                    # 两层检测：只有超过2倍超时才真正释放
                                    if time_since_press > self.key_hold_timeout * 2:
                                        if self.debug_log:
                                            self.log_file.write(f"[{current_time:.3f}] 按键 '{k}' 超时 ({time_since_press*1000:.1f}ms)，标记为释放\n")
                                            self.log_file.flush()
                                        self.keys_pressed[k] = False
                    
                    # 持续更新目标位置
                    with self.keys_lock:
                        has_key_pressed = any(self.keys_pressed.values())
                    
                    if has_key_pressed:
                        try:
                            self.update_target_position(dt=0.02)
                        except Exception as e:
                            print(f"错误: {e}")
                else:
                    # 暂停时也检查退出键
                    key = self.get_key()
                    if key:
                        key_lower = key.lower()
                        if key_lower == '\x1b' or key_lower == 'q':
                            print("\n正在退出...")
                            self.stop()
                            break
                        elif key_lower == ' ':
                            self.paused = False
                            print("\n继续控制...")
                
                time.sleep(0.02)  # 50Hz更新频率，避免更新太快
        finally:
            self.restore_terminal()
    
    def setup_terminal(self):
        """设置终端为原始模式"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    
    def restore_terminal(self):
        """恢复终端设置"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        """非阻塞获取按键"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def start(self):
        """启动控制"""
        self.running = True
        
        # 初始化控制策略
        self.init_policy()
        
        # 等待用户按回车
        print("按回车键开始键盘控制...")
        input()
        
        self.paused = False
        
        # 启动控制循环
        self.control_loop()
    
    def stop(self):
        """停止控制"""
        self.running = False
        self.paused = True
        
        # 恢复终端设置
        self.restore_terminal()
        
        print("\n正在停止机械臂...")
        
        # 停止阻抗控制
        if self.policy_active:
            self.robot.terminate_current_policy()
            self.policy_active = False
        
        # 关闭日志文件
        if hasattr(self, 'log_file') and self.log_file and not self.log_file.closed:
            try:
                self.log_file.write(f"\n日志结束 - 共{self.update_count}次更新\n")
                self.log_file.close()
                print("日志已保存")
            except Exception as e:
                print(f"日志保存失败: {e}")
        
        print("已停止")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='键盘控制Franka机器人')
    parser.add_argument('--ip', type=str, default='localhost',
                       help='机器人服务器IP地址（默认: localhost）')
    parser.add_argument('--speed', type=float, default=1.0,
                       help='移动速度 (m/s)，默认 1.0')
    parser.add_argument('--timeout', type=float, default=0.2,
                       help='按键超时时间 (秒)，默认 0.2。实际超时=timeout×2。减小=松开后快速停止，增大=更流畅')
    args = parser.parse_args()
    
    controller = None
    try:
        # 创建控制器
        controller = KeyboardController(robot_ip=args.ip)
        if args.speed:
            controller.move_speed = args.speed
        if args.timeout:
            controller.key_hold_timeout = args.timeout
        
        # 启动控制
        controller.start()
        
    except KeyboardInterrupt:
        print("\n\n用户中断，正在退出...")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        print("\n请确保:")
        print("1. Polymetis服务器已启动")
        print("2. 机器人连接正常")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        if controller:
            controller.stop()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
