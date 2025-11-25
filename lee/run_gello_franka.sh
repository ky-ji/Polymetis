#!/bin/bash
# GELLO + Franka 联合启动脚本
# 按 Ctrl+C 停止所有进程
#
# 使用方法：
#   ./run_gello_franka.sh          # 正常运行
#   ./run_gello_franka.sh --log    # 启用轨迹记录

set -e

# 解析参数
ENABLE_LOG=""
if [ "$1" == "--log" ]; then
    ENABLE_LOG="--log"
    echo "✓ 轨迹记录已启用"
fi

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 进程 PID
FRANKA_PID=""
GELLO_PID=""

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}======================================${NC}"
    echo -e "${YELLOW}正在停止所有进程...${NC}"
    echo -e "${YELLOW}======================================${NC}"
    
    # 停止 GELLO 发送端
    if [ -n "$GELLO_PID" ] && kill -0 $GELLO_PID 2>/dev/null; then
        echo -e "${BLUE}[1/2] 停止 GELLO 发送端 (PID: $GELLO_PID)${NC}"
        kill -SIGINT $GELLO_PID 2>/dev/null || true
        wait $GELLO_PID 2>/dev/null || true
        echo -e "${GREEN}✓ GELLO 发送端已停止${NC}"
    fi
    
    # 停止 Franka 接收端
    if [ -n "$FRANKA_PID" ] && kill -0 $FRANKA_PID 2>/dev/null; then
        echo -e "${BLUE}[2/2] 停止 Franka 接收端 (PID: $FRANKA_PID)${NC}"
        kill -SIGINT $FRANKA_PID 2>/dev/null || true
        wait $FRANKA_PID 2>/dev/null || true
        echo -e "${GREEN}✓ Franka 接收端已停止${NC}"
    fi
    
    echo ""
    echo -e "${GREEN}======================================${NC}"
    echo -e "${GREEN}所有进程已停止${NC}"
    echo -e "${GREEN}======================================${NC}"
    exit 0
}

# 捕获 Ctrl+C 信号
trap cleanup SIGINT SIGTERM

echo -e "${BLUE}======================================================================${NC}"
echo -e "${BLUE}               GELLO + Franka 联合控制系统启动${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

# 检查 conda
if ! command -v conda &> /dev/null; then
    echo -e "${RED}✗ 未找到 conda 命令${NC}"
    exit 1
fi

# 初始化 conda
eval "$(conda shell.bash hook)"

echo -e "${YELLOW}[1/2] 启动 Franka 接收端 (robot 环境)...${NC}"
echo -e "${BLUE}----------------------------------------------------------------------${NC}"

# 启动 Franka 接收端（后台运行，输出带前缀）
(
    conda activate robot
    cd "$SCRIPT_DIR"
    python -u franka_receiver.py $ENABLE_LOG 2>&1 | while IFS= read -r line; do
        echo -e "${GREEN}[Franka]${NC} $line"
    done
) &
FRANKA_PID=$!

echo -e "${GREEN}✓ Franka 接收端已启动 (PID: $FRANKA_PID)${NC}"
echo ""

# 等待 Franka 接收端初始化（等待 Socket 服务器启动）
echo -e "${YELLOW}等待 Franka 接收端初始化...${NC}"
sleep 3

echo -e "${YELLOW}[2/2] 启动 GELLO 发送端 (gello 环境)...${NC}"
echo -e "${BLUE}----------------------------------------------------------------------${NC}"

# 启动 GELLO 发送端（后台运行，输出带前缀）
(
    conda activate gello
    cd "$SCRIPT_DIR"
    python -u gello_sender.py 2>&1 | while IFS= read -r line; do
        echo -e "${YELLOW}[GELLO]${NC} $line"
    done
) &
GELLO_PID=$!

echo -e "${GREEN}✓ GELLO 发送端已启动 (PID: $GELLO_PID)${NC}"
echo ""

echo -e "${BLUE}======================================================================${NC}"
echo -e "${GREEN}✓ 系统启动完成${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""
echo -e "${YELLOW}进程信息:${NC}"
echo -e "  Franka 接收端: PID ${FRANKA_PID} (robot 环境)"
echo -e "  GELLO 发送端:  PID ${GELLO_PID} (gello 环境)"
echo ""
echo -e "${RED}按 Ctrl+C 停止所有进程${NC}"
echo -e "${BLUE}======================================================================${NC}"
echo ""

# 等待任意进程结束
wait -n $FRANKA_PID $GELLO_PID

# 如果任意一个进程异常退出，清理其他进程
cleanup
