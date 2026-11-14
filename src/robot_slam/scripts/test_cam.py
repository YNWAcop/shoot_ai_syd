#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
摄像头 + 圆形靶测试脚本
只开摄像头看画面和 /object_position 数值，不跑车

用法：
  rosrun robot_slam test_cam.py

按键：
  q         退出
  +/-       调整图像中心值
  r         重置中心为 337
  s         保存当前中心值到日志
"""

import rospy
import cv2
import sys
import os
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 图像中心（初始值，可调）
center_x = 337
center_y = None  # 自动从图像高度取半

bridge = CvBridge()
target_x = None
target_z = None
target_detected = False
target_count = 0
frame_h = 0
frame_w = 0


def object_cb(msg):
    global target_x, target_z, target_detected, target_count
    target_x = msg.x
    target_z = msg.z
    target_detected = (msg.z == 30)
    target_count += 1
    offset = target_x - center_x
    sys.stdout.write(
        "\r靶 x=%-6.1f  中心=%-3d  偏差=%-6.1fpx  z=%-2d  %s  "
        % (target_x, center_x, offset, msg.z,
           "[已对准]" if target_detected and abs(offset) <= 0.5 else
           "[对靶]" if target_detected else "[无靶]"))
    sys.stdout.flush()


def image_cb(msg):
    global frame_h, frame_w, center_y
    try:
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    frame_h, frame_w = cv_img.shape[:2]
    if center_y is None:
        center_y = frame_h // 2

    # 画十字线（绿色 = 当前假设的中心）
    cv2.line(cv_img, (center_x, 0), (center_x, frame_h), (0, 255, 0), 1)
    cv2.line(cv_img, (0, center_y), (frame_w, center_y), (0, 255, 0), 1)

    # 画物理中心（暗绿色虚线 = 图像实际正中间）
    phys_cx = frame_w // 2
    cv2.line(cv_img, (phys_cx, 0), (phys_cx, frame_h), (50, 180, 50), 1, cv2.LINE_AA)

    # 如果有靶子，显示位置
    if target_x is not None and target_detected:
        # 红色圆点 = 检测到的靶心
        cv2.circle(cv_img, (int(target_x), center_y), 6, (0, 0, 255), -1)
        cv2.circle(cv_img, (int(target_x), center_y), 8, (0, 0, 200), 2)
        # 偏差连线
        cv2.line(cv_img, (center_x, center_y),
                 (int(target_x), center_y), (0, 255, 255), 2)

    # 叠加文字信息
    info = "Center: %d  (phys: %d)  Res: %dx%d" % (center_x, phys_cx, frame_w, frame_h)
    cv2.putText(cv_img, info, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    if target_x is not None:
        det = "Tgt x=%.1f  dev=%.1fpx" % (target_x, target_x - center_x)
        cv2.putText(cv_img, det, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if target_detected else (0, 0, 255), 2)
    cv2.putText(cv_img, "+/-:adj  r:reset  s:save  q:quit",
                (10, frame_h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    cv2.imshow("Camera Test", cv_img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        rospy.signal_shutdown("user quit")
    elif key == ord('+') or key == ord('='):
        center_x += 1
        rospy.loginfo("center_x -> %d", center_x)
    elif key == ord('-') or key == ord('_'):
        center_x -= 1
        rospy.loginfo("center_x -> %d", center_x)
    elif key == ord('r'):
        center_x = 337
        rospy.loginfo("center_x reset to %d", center_x)
    elif key == ord('s'):
        log_path = os.path.expanduser("~/center_log.txt")
        with open(log_path, "a") as f:
            f.write("center_x=%d  res=%dx%d  target_x=%s  dev=%.1f\n" % (
                center_x, frame_w, frame_h,
                "%.1f" % target_x if target_x is not None else "None",
                target_x - center_x if target_x is not None else 0))
        rospy.loginfo("saved to %s", log_path)


if __name__ == "__main__":
    rospy.init_node("test_cam", anonymous=True)
    rospy.Subscriber("/object_position", Point, object_cb)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_cb,
                     queue_size=1, buff_size=2**20)

    print("=" * 55)
    print("  Camera Test Script")
    print("  Waiting for /object_position and /usb_cam/image_raw...")
    print("  Keys: +/- adjust center  r reset  s save  q quit")
    print("=" * 55)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

