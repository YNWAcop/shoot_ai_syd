#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
霍夫圆检测节点
替代 find_object_2d 的圆形靶识别，发布 /object_position

接口与原来的 print_objects_detected 完全一致：
  z=30  → 检测到圆形靶
  z=255 → 未检测到

用法：
  rosrun robot_slam circle_detector.py

依赖：
  ros-noetic-usb-cam  或同类型摄像头驱动，发布 /usb_cam/image_raw
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


# ==================== 可调参数 ====================

# 霍夫圆检测参数（如果圆形靶没识别出来，优先调这些）
#HOUGH_DP = 1.2          # 累加器分辨率与图像分辨率的反比
#HOUGH_MIN_DIST = 100    # 检测到的圆之间的最小距离（像素）
#HOUGH_PARAM1 = 100      # Canny 边缘检测的高阈值
#HOUGH_PARAM2 = 30       # 圆心检测阈值，越小越敏感（但误检也多）
#HOUGH_MIN_RADIUS = 20   # 最小圆半径（像素）
#HOUGH_MAX_RADIUS = 200  # 最大圆半径（像素）
#lq:
HOUGH_DP = 1.2          # 累加器分辨率与图像分辨率的反比
HOUGH_MIN_DIST = 80    # 检测到的圆之间的最小距离（像素）
HOUGH_PARAM1 = 85      # Canny 边缘检测的高阈值
HOUGH_PARAM2 = 30       # 圆心检测阈值，越小越敏感（但误检也多）
HOUGH_MIN_RADIUS = 20   # 最小圆半径（像素）
HOUGH_MAX_RADIUS = 200  # 最大圆半径（像素）

# 图像话题（如果你的摄像头话题不是这个，改这里）
IMAGE_TOPIC = "/usb_cam/image_raw"

# =================================================

bridge = CvBridge()
pub = rospy.Publisher('/object_position', Point, queue_size=10)


def image_callback(msg):
    """摄像头回调：每帧做霍夫圆检测"""
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge error: %s", e)
        return

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

    # 霍夫圆检测
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=HOUGH_DP,
        minDist=HOUGH_MIN_DIST,
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=HOUGH_MIN_RADIUS,
        maxRadius=HOUGH_MAX_RADIUS,
    )

    p = Point()
    if circles is not None:
        # 有检测到圆
        circles = np.round(circles[0, :]).astype("int")
        # 取半径最大的圆（排除小噪点）
        c = max(circles, key=lambda x: x[2])
        p.x = float(c[0])   # 圆心 x
        p.y = float(c[1])   # 圆心 y
        p.z = 30.0          # 有效检测标识
    else:
        # 没检测到圆
        p.x = 0.0
        p.y = 0.0
        p.z = 255.0         # 无靶标识

    pub.publish(p)


if __name__ == "__main__":
    rospy.init_node("circle_detector", anonymous=True)

    # 订阅摄像头图像
    rospy.Subscriber(IMAGE_TOPIC, Image, image_callback,
                     queue_size=1, buff_size=2**20)

    rospy.loginfo("circle_detector 启动，订阅 %s", IMAGE_TOPIC)
    rospy.loginfo("HoughCircles 参数: dp=%.1f, minDist=%.0f, "
                  "param1=%.0f, param2=%.0f, "
                  "minRadius=%.0f, maxRadius=%.0f",
                  HOUGH_DP, HOUGH_MIN_DIST,
                  HOUGH_PARAM1, HOUGH_PARAM2,
                  HOUGH_MIN_RADIUS, HOUGH_MAX_RADIUS)

    rospy.spin()
