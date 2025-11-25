#!/usr/bin/env python3
"""
简单的移动测试脚本

演示基本的移动功能，无需键盘交互

使用方法：
    python simple_move.py [--ip IP地址] [--demo 演示编号]
    
可选参数：
    --demo 1 : 末端画正方形
    --demo 2 : 关节正弦运动
    --demo 3 : 末端圆形轨迹
    --demo 4 : 交互式移动测试
"""
import torch
import time
import sys
import numpy as np
from polymetis import RobotInterface


def demo_square(robot, size=0.1, height=0.4):
    """演示1: 末端画正方形"""
    print("\n" + "=" * 70)
    print("演示1: 末端画正方形")
    print("=" * 70)
    
    # 获取起始位置
    ee_pos, ee_quat = robot.get_ee_pose()
    center_x, center_y = ee_pos[0].item(), ee_pos[1].item()
    
    print(f"中心位置: ({center_x:.3f}, {center_y:.3f}, {height:.3f})")
    print(f"正方形边长: {size*2:.3f}m\n")
    
    # 定义正方形的四个角点
    corners = [
        [center_x + size, center_y + size, height],
        [center_x - size, center_y + size, height],
        [center_x - size, center_y - size, height],
        [center_x + size, center_y - size, height],
        [center_x + size, center_y + size, height],  # 回到起点
    ]
    
    # 使用阻抗控制
    robot.start_cartesian_impedance()
    
    for i, corner in enumerate(corners):
        target = torch.Tensor(corner)
        print(f"移动到角点{i+1}: ({corner[0]:.3f}, {corner[1]:.3f}, {corner[2]:.3f})")
        
        # 平滑移动
        current_pos, _ = robot.get_ee_pose()
        steps = 50
        for step in range(steps):
            alpha = (step + 1) / steps
            interpolated = current_pos * (1 - alpha) + target * alpha
            robot.update_desired_ee_pose(position=interpolated)
            time.sleep(0.02)
        
        time.sleep(0.5)
    
    robot.terminate_current_policy()
    print("\n✓ 正方形轨迹完成!")


def demo_sine_joint(robot, joint_idx=5, amplitude=0.3, periods=2):
    """演示2: 单个关节正弦运动"""
    print("\n" + "=" * 70)
    print(f"演示2: 关节{joint_idx+1}正弦运动")
    print("=" * 70)
    
    joint_pos = robot.get_joint_positions()
    initial_pos = joint_pos[joint_idx].item()
    
    print(f"初始角度: {initial_pos:.3f} rad ({initial_pos*180/np.pi:.1f}°)")
    print(f"振幅: ±{amplitude:.3f} rad (±{amplitude*180/np.pi:.1f}°)")
    print(f"周期数: {periods}\n")
    
    robot.start_joint_impedance()
    
    duration = 5.0  # 总时长(秒)
    hz = 50
    steps = int(duration * hz)
    
    for i in range(steps):
        t = i / hz
        joint_pos[joint_idx] = initial_pos + amplitude * np.sin(2 * np.pi * periods * t / duration)
        robot.update_desired_joint_positions(joint_pos)
        time.sleep(1 / hz)
    
    robot.terminate_current_policy()
    print("\n✓ 正弦运动完成!")


def demo_circle(robot, radius=0.05, height=0.4, revolutions=1):
    """演示3: 末端圆形轨迹"""
    print("\n" + "=" * 70)
    print("演示3: 末端圆形轨迹")
    print("=" * 70)
    
    # 获取起始位置
    ee_pos, ee_quat = robot.get_ee_pose()
    center_x, center_y = ee_pos[0].item(), ee_pos[1].item()
    
    print(f"圆心: ({center_x:.3f}, {center_y:.3f}, {height:.3f})")
    print(f"半径: {radius:.3f}m")
    print(f"圈数: {revolutions}\n")
    
    robot.start_cartesian_impedance()
    
    duration = 5.0 * revolutions
    hz = 50
    steps = int(duration * hz)
    
    for i in range(steps):
        angle = 2 * np.pi * revolutions * i / steps
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
        z = height
        
        target = torch.Tensor([x, y, z])
        robot.update_desired_ee_pose(position=target)
        time.sleep(1 / hz)
    
    robot.terminate_current_policy()
    print("\n✓ 圆形轨迹完成!")


def demo_interactive(robot):
    """演示4: 交互式移动测试"""
    print("\n" + "=" * 70)
    print("演示4: 交互式移动测试")
    print("=" * 70)
    print("\n测试可用命令:")
    print("  1. 末端沿X轴移动")
    print("  2. 末端沿Y轴移动")
    print("  3. 末端沿Z轴移动")
    print("  4. 回Home位置")
    print("  0. 退出\n")
    
    while True:
        try:
            choice = input("选择测试 (0-4): ").strip()
            
            if choice == '0':
                break
            elif choice == '4':
                print("回到Home位置...")
                robot.go_home()
                print("✓ 完成")
                continue
            
            # 获取移动距离
            distance = float(input("输入移动距离 (m, 正数或负数): ").strip())
            
            ee_pos, ee_quat = robot.get_ee_pose()
            target = ee_pos.clone()
            
            if choice == '1':
                target[0] += distance
                axis = 'X'
            elif choice == '2':
                target[1] += distance
                axis = 'Y'
            elif choice == '3':
                target[2] += distance
                axis = 'Z'
            else:
                print("无效选择")
                continue
            
            print(f"移动末端沿{axis}轴 {distance:+.3f}m ...")
            robot.move_to_ee_pose(position=target, time_to_go=2.0)
            
            new_pos, _ = robot.get_ee_pose()
            print(f"✓ 当前位置: ({new_pos[0]:.3f}, {new_pos[1]:.3f}, {new_pos[2]:.3f})\n")
            
        except ValueError:
            print("输入错误，请输入有效数字\n")
        except KeyboardInterrupt:
            print("\n中断")
            break


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='简单移动测试')
    parser.add_argument('--ip', type=str, default='localhost',
                       help='机器人服务器IP（默认: localhost）')
    parser.add_argument('--demo', type=int, default=0,
                       help='演示编号 (1=正方形, 2=正弦, 3=圆形, 4=交互, 0=菜单)')
    args = parser.parse_args()
    
    print("=" * 70)
    print(" " * 20 + "Franka 移动测试")
    print("=" * 70)
    print(f"\n连接到: {args.ip}")
    
    try:
        # 连接机器人
        robot = RobotInterface(ip_address=args.ip)
        print("✓ 连接成功!\n")
        
        # 显示当前状态
        ee_pos, ee_quat = robot.get_ee_pose()
        joint_pos = robot.get_joint_positions()
        print(f"当前末端位置: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")
        print(f"当前关节角度: {joint_pos.numpy()}")
        
        # 回Home
        print("\n准备中...")
        robot.go_home()
        time.sleep(1)
        
        # 选择演示
        if args.demo == 0:
            print("\n" + "=" * 70)
            print("可用演示:")
            print("  1. 末端画正方形")
            print("  2. 关节正弦运动")
            print("  3. 末端圆形轨迹")
            print("  4. 交互式移动测试")
            print("=" * 70)
            
            demo_num = int(input("\n选择演示 (1-4): ").strip())
        else:
            demo_num = args.demo
        
        # 运行演示
        if demo_num == 1:
            demo_square(robot)
        elif demo_num == 2:
            demo_sine_joint(robot)
        elif demo_num == 3:
            demo_circle(robot)
        elif demo_num == 4:
            demo_interactive(robot)
        else:
            print("无效的演示编号")
            return 1
        
        # 回Home
        print("\n回到Home位置...")
        robot.go_home()
        print("\n✓ 所有测试完成!")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        print("\n请确保:")
        print("1. Polymetis服务器已启动")
        print("2. 机器人连接正常")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
