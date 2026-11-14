#!/bin/bash
# 启动 ROS USB 相机节点
source /opt/ros/melodic/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)/env_syd.sh"
echo "=== 正在启动 USB 摄像头节点 (/dev/video0) ==="
roslaunch usb_cam usb_cam-test.launch
