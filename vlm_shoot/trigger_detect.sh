#!/bin/bash
# 触发一次识别
source /opt/ros/melodic/setup.bash
echo "=== 发送触发识别指令 ==="
rosparam set /detect 1
echo "已设置 rosparam /detect = 1"
