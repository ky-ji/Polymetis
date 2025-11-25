#!/bin/bash
# 只启动：机械臂 + 夹爪控制端 & GELLO 夹爪状态发送端
# 注意：两个进程在不同的 conda 环境中
#
# 使用方法：
#   cd ~/Desktop/Workspace/robot/polymetis/lee
#   chmod +x run_gripper_only.sh
#   ./run_gripper_only.sh

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

ARM_PID=""
GELLO_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}======================================${NC}"
    echo -e "${YELLOW}正在停止所有进程...${NC}"
    echo -e "${YELLOW}======================================${NC}"

    # 停止 GELLO 发送端
    if [ -n "$GELLO_PID" ] && kill -0 $GELLO_PID 2>/dev/null; then
        echo -e "${BLUE}[1/2] 停止 GELLO 夹爪发送端 (PID: $GELLO_PID)${NC}"
        kill -SIGINT $GELLO_PID 2>/dev/null || true
        wait $GELLO_PID 2>/dev/null || true
        echo -e "${GREEN}✓ GELLO 夹爪发送端已停止${NC}"
    fi

    # 停止 机械臂 + 夹爪 控制端
    if [ -n "$ARM_PID" ] && kill -0 $ARM_PID 2>/dev/null; then
        echo -e "${BLUE}[2/2] 停止 机械臂 + 夹爪 控制端 (PID: $ARM_PID)${NC}"
        kill -SIGINT $ARM_PID 2>/dev/null || true
        wait $ARM_PID 2>/dev/null || true
        echo -e "${GREEN}✓ 机械臂 + 夹爪 控制端已停止${NC}"
    fi

    echo ""
    echo -e "${GREEN}======================================${NC}"
    echo -e "${GREEN}所有进程已停止${NC}"
    echo -e "${GREEN}======================================${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}            GELLO 夹爪 + Franka 夹爪 联合启动脚本${NC}"
echo -e "${BLUE}======================================================================${NC}"

# 检查 conda
if ! command -v conda &> /dev/null; then
    echo -e "${RED}✗ 未找到 conda 命令${NC}"
    exit 1
fi

# 初始化 conda
eval "$(conda shell.bash hook)"

# 1. 启动 机械臂 + 夹爪 控制端（robot 环境）
echo -e "${YELLOW}[1/2] 启动 机械臂 + 夹爪 控制端 (robot 环境)...${NC}"
echo -e "${BLUE}----------------------------------------------------------------------${NC}"
(
    conda activate robot
    cd "$SCRIPT_DIR"
    python arm_gripper_controller.py 2>&1 | while IFS= read -r line; do
        echo -e "${GREEN}[ARM]${NC} $line"
    done
) &
ARM_PID=$!

echo -e "${GREEN}✓ 机械臂 + 夹爪 控制端已启动 (PID: $ARM_PID)${NC}"
echo ""

echo -e "${YELLOW}等待 ARM 控制端初始化 (TCP 服务器监听)...${NC}"
sleep 3

# 2. 启动 GELLO 夹爪发送端（gello 环境）
echo -e "${YELLOW}[2/2] 启动 GELLO 夹爪发送端 (gello 环境)...${NC}"
echo -e "${BLUE}----------------------------------------------------------------------${NC}"
(
    conda activate gello
    cd "$SCRIPT_DIR/.."   # 脚本在 polymetis/lee，下一级是 polymetis 根
    python lee/gello_gripper_sender.py 2>&1 | while IFS= read -r line; do
        echo -e "${YELLOW}[GELLO]${NC} $line"
    done
) &
GELLO_PID=$!

echo -e "${GREEN}✓ GELLO 夹爪发送端已启动 (PID: $GELLO_PID)${NC}"

echo ""
echo -e "${BLUE}======================================================================${NC}"
echo -e "${GREEN}✓ 所有进程已启动${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""
echo -e "${YELLOW}进程信息:${NC}"
echo -e "  机械臂 + 夹爪 控制端: PID ${ARM_PID} (robot 环境)"
echo -e "  GELLO 夹爪发送端:     PID ${GELLO_PID} (gello 环境)"
echo ""
echo -e "${RED}按 Ctrl+C 停止所有进程${NC}"
echo -e "${BLUE}======================================================================${NC}"

# 等待任意一个进程结束
wait -n $ARM_PID $GELLO_PID

# 有任一进程退出，则清理其它
cleanup
