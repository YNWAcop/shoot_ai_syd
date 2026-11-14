#!/bin/bash
# 使用已配置完整依赖的 Python 3.9 环境启动 ROS 视觉识别节点
source /opt/ros/melodic/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)/env_syd.sh"
PYTHON_BIN=/home/abot/anaconda3/envs/py39/bin/python

echo "=== 正在启动 vlm_node.py (Python 3.9) ==="
exec "$PYTHON_BIN" "$(dirname "$(readlink -f "$0")")/vlm_node.py"
