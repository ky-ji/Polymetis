#!/usr/bin/env python3
"""
单独测试 Polymetis 夹爪服务器是否可连接、可控制。

使用方法（在 robot 环境）：
    conda activate robot
    cd ~/Desktop/Workspace/robot/polymetis/lee
    python test_gripper.py --ip localhost --port 50052

默认连接本机 50052 端口（start_server.py 硬件模式下的默认夹爪端口）。
"""
import argparse
import time

from polymetis import GripperInterface


def main():
    parser = argparse.ArgumentParser(description="测试 Polymetis 夹爪服务器")
    parser.add_argument("--ip", default="localhost", help="夹爪服务器 IP 地址")
    parser.add_argument("--port", type=int, default=50052, help="夹爪服务器端口")
    args = parser.parse_args()

    print("=" * 70)
    print("Polymetis 夹爪服务器连接测试")
    print("=" * 70)
    print(f"目标服务器: {args.ip}:{args.port}")

    try:
        print("\n[1/3] 连接夹爪服务器...")
        gripper = GripperInterface(ip_address=args.ip, port=args.port)
        print("✓ 已连接到夹爪服务器")

        print("\n[2/3] 读取当前夹爪状态...")
        try:
            state = gripper.get_state()
            print(f"  当前位置: {state.width:.4f} m")
        except Exception as e:
            print(f"  无法获取状态: {e}")

        print("\n[3/3] 做一次简单开合测试 (如果安全的话)...")
        try:
            # 尝试打开到 0.08m
            open_width = 0.08
            close_width = 0.0
            print(f"  -> 打开到 {open_width:.3f} m")
            gripper.goto(width=open_width, speed=0.1, force=20.0)
            time.sleep(1.0)

            print(f"  -> 闭合到 {close_width:.3f} m")
            gripper.goto(width=close_width, speed=0.1, force=20.0)
            time.sleep(1.0)

            print("\n✓ 夹爪开合测试完成")
        except Exception as e:
            print(f"  ⚠️ 开合测试失败: {e}")

        print("\n测试结束。")

    except Exception as e:
        print("✗ 无法连接夹爪服务器")
        print(f"  详细错误: {e}")
        print("\n请检查:")
        print("  1. start_server.py 是否以硬件模式运行")
        print("  2. 日志中是否有 '[2] 启动夹爪服务器...' 和 '✓ 夹爪服务器已启动'")
        print("  3. IP/端口 是否与此脚本参数一致")


if __name__ == "__main__":
    main()
