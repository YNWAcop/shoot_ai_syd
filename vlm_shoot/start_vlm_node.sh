#!/bin/bash
# 使用已配置完整依赖的 Python 3.9 环境启动 ROS 视觉识别节点
source /opt/ros/melodic/setup.bash
source /home/abot/abot_ws/devel/setup.bash 2>/dev/null || true
PYTHON_BIN=/home/abot/anaconda3/envs/py39/bin/python

echo "=== 正在启动 vlm_node.py (Python 3.9) ==="
$PYTHON_BIN /home/abot/vlm_shoot/vlm_node.py
