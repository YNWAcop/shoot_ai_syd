#!/bin/bash
# 一键启动（替代旧的 N8NNWC.sh）：
#   第 1 个标签页 roscore
#   第 2 个标签页 底层节点（bringup.launch）
#   第 3 个标签页 比赛主程序（race.launch）—— 在这个标签页里输入 1 开赛；比赛程序结束后自动关闭底层节点（省电，2026-10-19）
# 用法：bash ~/N8NNWC/src/shoot_race/scripts/start_race.sh [额外的 race.launch 参数，如 dry_fire:=true]
unset PYTHONPATH
source /opt/ros/melodic/setup.bash
export ROS_PACKAGE_PATH=/home/abot/N8NNWC:$ROS_PACKAGE_PATH
# Python2 输出中文时按 UTF-8 编码（终端编码不是 UTF-8 时也不会报错）
export PYTHONIOENCODING=utf-8
# 从 Windows 拷贝过来的脚本会丢失可执行权限，这里补上
chmod +x /home/abot/N8NNWC/src/shoot_race/scripts/*.py
EXTRA_ARGS="$*"
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/N8NNWC/devel/setup.bash; roslaunch shoot_race bringup.launch; exec bash"' \
--tab -e "bash -c \"sleep 8; source ~/N8NNWC/devel/setup.bash; roslaunch shoot_race race.launch $EXTRA_ARGS; echo 比赛程序结束，关闭底层节点省电; pkill -INT -f 'roslaunch shoot_race bringup.launch'; exec bash\""
