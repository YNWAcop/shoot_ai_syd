#!/bin/bash
# lingyun_bot 启动底层节点（雷达、底盘、导航、相机、AR、find_object、开枪）。
unset PYTHONPATH
source /opt/ros/melodic/setup.bash
source ~/748573/devel/setup.bash
export DISPLAY=${DISPLAY:-:0}         # find_object 的窗口开在车自己的桌面上
roslaunch "$(dirname "$0")/launch/bringup_all.launch" "$@"
