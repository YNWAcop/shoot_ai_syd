#!/bin/bash
### VLM Shoot 桌面多窗口一键启动 ###
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PYTHON_BIN="/home/abot/anaconda3/envs/py39/bin/python"

gnome-terminal --window -e 'bash -c "source /opt/ros/melodic/setup.bash; roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source /opt/ros/melodic/setup.bash; roslaunch usb_cam usb_cam-test.launch; exec bash"' \
--tab -e "bash -c 'sleep 5; source /opt/ros/melodic/setup.bash; $PYTHON_BIN $DIR/vlm_node.py; exec bash'" \
--tab -e "bash -c 'sleep 6; echo \"=== 触发识别窗口 ===\"; echo \"输入: rosparam set /detect 1 即可识别\"; bash $DIR/trigger_detect.sh; exec bash'"
