#!/usr/bin/env python3
"""
测试机器人是否真的能动
"""
import torch
from polymetis import RobotInterface

# 连接机器人
robot = RobotInterface(ip_address="localhost")

print("当前位置:")
ee_pos, ee_quat = robot.get_ee_pose()
print(f"  X={ee_pos[0]:.3f}, Y={ee_pos[1]:.3f}, Z={ee_pos[2]:.3f}")

# 测试1: 向右移动5cm
print("\n测试1: Y轴向右移动5cm")
target = ee_pos.clone()
target[1] += 0.05
print(f"目标: X={target[0]:.3f}, Y={target[1]:.3f}, Z={target[2]:.3f}")
robot.move_to_ee_pose(position=target, time_to_go=3.0)

# 检查
ee_pos_new, _ = robot.get_ee_pose()
print(f"到达: X={ee_pos_new[0]:.3f}, Y={ee_pos_new[1]:.3f}, Z={ee_pos_new[2]:.3f}")

# 测试2: 回到原位
print("\n测试2: 回到原位")
robot.move_to_ee_pose(position=ee_pos, time_to_go=3.0)
ee_pos_final, _ = robot.get_ee_pose()
print(f"最终: X={ee_pos_final[0]:.3f}, Y={ee_pos_final[1]:.3f}, Z={ee_pos_final[2]:.3f}")

print("\n✓ 测试完成")
