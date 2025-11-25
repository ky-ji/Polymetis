#!/usr/bin/env python3
"""
夹爪测试脚本
用于验证夹爪控制是否正常工作
"""
import time
from polymetis import GripperInterface

def test_gripper():
    """测试夹爪控制"""
    print("连接到夹爪...")
    gripper = GripperInterface(ip_address="localhost", port=50052)
    
    # 读取初始状态
    state = gripper.get_state()
    print(f"初始状态: 宽度={state.width:.4f}m")
    
    # 测试打开
    print("\n测试1: 打开夹爪 (0.08m)...")
    gripper.goto(width=0.08, speed=0.1, force=1.0, blocking=True)
    time.sleep(1.0)
    state = gripper.get_state()
    print(f"结果: 宽度={state.width:.4f}m")
    
    # 测试关闭（使用 grasp 方法）
    print("\n测试2: 抓取/关闭夹爪...")
    gripper.grasp(speed=0.1, force=1.0, epsilon_inner=0.1, epsilon_outer=0.1, blocking=True)
    time.sleep(1.0)
    state = gripper.get_state()
    print(f"结果: 宽度={state.width:.4f}m")
    
    # 测试再次打开
    print("\n测试3: 再次打开夹爪 (0.08m)...")
    gripper.goto(width=0.08, speed=0.1, force=1.0, blocking=True)
    time.sleep(1.0)
    state = gripper.get_state()
    print(f"结果: 宽度={state.width:.4f}m")
    
    print("\n✓ 夹爪测试完成")

if __name__ == '__main__':
    test_gripper()
