#!/bin/bash
# jinghong_car 开赛：先在另一个终端启动 base_nodes.launch，等 20 秒左右再运行这个。
unset PYTHONPATH                      # 车上 anaconda 往 PYTHONPATH 里加了 Python3 的库，ROS 的 python2 会报错
source /opt/ros/melodic/setup.bash
source ~/748573/devel/setup.bash
export PYTHONIOENCODING=utf-8
cd "$(dirname "$0")/scripts" && python2 run_game.py
