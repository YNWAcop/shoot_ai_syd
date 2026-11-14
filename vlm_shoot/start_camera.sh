#!/bin/bash
# 启动 ROS USB 相机节点
source /opt/ros/melodic/setup.bash
source /home/abot/abot_ws/devel/setup.bash 2>/dev/null || true
echo "=== 正在启动 USB 摄像头节点 (/dev/video0) ==="
roslaunch usb_cam usb_cam-test.launch
