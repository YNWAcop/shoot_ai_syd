#!/bin/bash
# ============================================================
# env.sh：在新终端里手动运行 roslaunch / rosrun 之前先执行一次：
#     source ~/N8NNWC/src/shoot_race/scripts/env.sh
#
# 为什么需要：车上 ~/.bashrc 往 PYTHONPATH 里加了 Python3 的库目录（还激活了 anaconda），
# ROS Melodic 的工具（roslaunch、rosrun 等）用 Python2 运行，会先找到 Python3 版的 yaml 而报
#     SyntaxError: invalid syntax（/usr/lib/python3/dist-packages/yaml/__init__.py）
# 旧的 N8NNWC.sh 开头 unset PYTHONPATH 就是为了这个。start_race.sh 和 check_car.sh 已经自带这一步。
# ============================================================
unset PYTHONPATH
# 工作空间 = 本文件往上三级（scripts → shoot_race → src → 工作空间），按真实路径算（readlink -f、pwd -P）。
# 2026-10-18 新车：工作空间是 ~/204809，用软链接 ~/N8NNWC 指过去。如果这里写死 ~/N8NNWC，
# ROS_PACKAGE_PATH 里是 N8NNWC/src，devel 里记的是 204809/src，同一个文件两个路径，
# roslaunch 会报 "multiple files named [bringup.launch] in package [shoot_race]"，rosrun 也会让人选。
SHOOT_RACE_WS="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../.." && pwd -P)"
source /opt/ros/melodic/setup.bash
source "$SHOOT_RACE_WS/devel/setup.bash"
export ROS_PACKAGE_PATH="$SHOOT_RACE_WS/src:$ROS_PACKAGE_PATH"
export PYTHONIOENCODING=utf-8
# 通过 SSH 登录时，让 find_object 等程序的窗口显示在车自己的桌面上，而不是经 X11 转发到电脑
# （转发图像很慢，还会占网络）。车上需要有人登录桌面（ToDesk 能看到桌面就说明已登录）。
if [ -n "$SSH_CONNECTION" ]; then
  export DISPLAY=:0
fi
echo "ROS 环境已准备好（PYTHONPATH 已清理）"
