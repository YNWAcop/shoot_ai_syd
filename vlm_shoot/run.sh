#!/bin/bash
# ==============================================================================
# VLM Shoot 视觉大模型射击任务 - 一键启动脚本
# 支持命令行交互模式与快速测试
# ==============================================================================

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PYTHON_BIN="/home/abot/anaconda3/envs/py39/bin/python"

source /opt/ros/melodic/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)/env_syd.sh"

echo "=========================================================="
echo "          ★ VLM Shoot 视觉大模型射击系统 ★              "
echo "=========================================================="
echo " 1) 快速单次测试 (直接调用摄像头拍照并识别输出两个数字)"
echo " 2) 启动 ROS 服务模式 (自动启动摄像头与识别节点，按回车随时触发)"
echo " 3) 打开图形多标签窗口 (桌面 GUI gnome-terminal 模式)"
echo " q) 退出"
echo "=========================================================="
read -p "请选择操作 [1/2/3/q] (默认: 1): " choice
choice=${choice:-1}

case "$choice" in
  1)
    echo -e "\n>>> 正在启动快速单次测试..."
    $PYTHON_BIN "$DIR/test_pipeline.py"
    ;;
  2)
    echo -e "\n>>> 正在检查并启动 ROS 节点..."
    
    # 检查 roscore
    if ! pgrep -x "roscore" > /dev/null && ! pgrep -f "rosmaster" > /dev/null; then
        echo ">>> 启动 roscore..."
        roscore > /dev/null 2>&1 &
        ROSCORE_PID=$!
        sleep 3
    else
        echo ">>> roscore 已在运行。"
    fi

    # 检查摄像头节点
    if ! rosnode list 2>/dev/null | grep -q "/usb_cam"; then
        echo ">>> 启动 usb_cam 摄像头节点..."
        roslaunch usb_cam usb_cam-test.launch > /dev/null 2>&1 &
        CAM_PID=$!
        sleep 3
    else
        echo ">>> usb_cam 节点已在运行。"
    fi

    # 启动 vlm_node
    echo ">>> 启动 vlm_node.py 识别节点..."
    $PYTHON_BIN "$DIR/vlm_node.py" &
    NODE_PID=$!
    sleep 2

    cleanup() {
        echo -e "\n>>> 正在停止相关节点..."
        [ -n "$NODE_PID" ] && kill -9 "$NODE_PID" 2>/dev/null
        [ -n "$CAM_PID" ] && kill -9 "$CAM_PID" 2>/dev/null
        echo ">>> 已安全退出。"
        exit 0
    }
    trap cleanup SIGINT SIGTERM

    echo -e "\n=========================================================="
    echo " [系统已就绪]"
    echo "   - 按 [回车键 Enter] 触发一次拍照识别"
    echo "   - 输入 [q] 并回车退出系统"
    echo "=========================================================="
    while true; do
        read -p ">> 按回车触发识别 (或输入 q 退出): " cmd
        if [ "$cmd" = "q" ] || [ "$cmd" = "Q" ]; then
            cleanup
        fi
        echo ">>> 正在触发拍照识别..."
        rosparam set /detect 1
        sleep 1
    done
    ;;
  3)
    echo ">>> 启动桌面 GUI 终端..."
    bash "$DIR/gui_start.sh"
    ;;
  q|Q)
    echo "退出。"
    exit 0
    ;;
  *)
    echo "未知选项，退出。"
    exit 1
    ;;
esac
