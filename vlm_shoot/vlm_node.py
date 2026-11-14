#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 视觉识别服务节点
功能：
1. 订阅 /usb_cam/image_raw 获取实时相机画面
2. 监听参数 /detect (或 /top_view_shot_node/im_flag) 或 ROS 服务 /trigger_vlm 触发单次识别
3. 保存当前帧图像到 temp/vl_now.jpg
4. 调用大模型识别框架，返回两个数字 (计算结果, 水果代码: 6榴莲/7杨桃/8山竹)
5. 发布识别结果到 /vlm_result, /vlm_shoot/math_result, /vlm_shoot/fruit_code
"""
import os
import sys

# 将 ROS melodic python 库加入路径
sys.path.append("/opt/ros/melodic/lib/python2.7/dist-packages")
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerResponse

from vlm_api import call_vlm_api
from api_config import FRUIT_CODE_TO_NAME

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
TEMP_IMG_PATH = os.path.join(CURRENT_DIR, "temp", "vl_now.jpg")

current_frame = None
is_processing = False


def imgmsg_to_cv2(img_msg):
    """ROS Image 消息转 OpenCV BGR 格式"""
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(
        shape=(img_msg.height, img_msg.width, 3),
        dtype=dtype,
        buffer=img_msg.data
    )
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()

    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("不支持的图像编码格式: " + str(img_msg.encoding))
        return None

    return image_opencv


def image_callback(msg):
    """相机话题回调，持续缓存最新图像帧"""
    global current_frame
    current_frame = imgmsg_to_cv2(msg)


def execute_vlm_recognition():
    """执行一次完整的拍摄与识别流程"""
    global current_frame, is_processing
    if is_processing:
        rospy.logwarn("正在处理上一次识别请求，请稍后...")
        return None, None

    if current_frame is None:
        rospy.logerr("尚未接收到相机图像，请检查 /usb_cam/image_raw 是否正常发布！")
        return None, None

    is_processing = True
    try:
        # 1. 保存当前图像帧
        os.makedirs(os.path.dirname(TEMP_IMG_PATH), exist_ok=True)
        cv2.imwrite(TEMP_IMG_PATH, current_frame)
        rospy.loginfo("图像已保存至: " + TEMP_IMG_PATH)

        # 2. 调用大模型接口
        rospy.loginfo("正在请求视觉大模型分析卡片...")
        math_res, fruit_code = call_vlm_api(TEMP_IMG_PATH)
        fruit_name = FRUIT_CODE_TO_NAME.get(fruit_code, "未知水果")

        rospy.loginfo("========================================")
        rospy.loginfo("★ 视觉大模型识别结果 ★")
        rospy.loginfo("  [1] 计算题结果: %s" % math_res)
        rospy.loginfo("  [2] 水果识别码: %s (%s)" % (fruit_code, fruit_name))
        rospy.loginfo("  每次返回两个数字: %s %s" % (math_res, fruit_code))
        rospy.loginfo("========================================")

        # 3. 发布 ROS 话题
        res_str = "%s %s" % (math_res, fruit_code)
        pub_result_str.publish(res_str)
        if math_res is not None:
            pub_math.publish(int(math_res))
        if fruit_code is not None:
            pub_fruit.publish(int(fruit_code))

        return math_res, fruit_code
    except Exception as e:
        rospy.logerr("识别过程出错: " + str(e))
        return None, None
    finally:
        is_processing = False


def trigger_service_callback(req):
    """ROS Service 服务回调 (/trigger_vlm)"""
    math_res, fruit_code = execute_vlm_recognition()
    if math_res is not None and fruit_code is not None:
        msg = "SUCCESS: math=%s, fruit=%s" % (math_res, fruit_code)
        return TriggerResponse(success=True, message=msg)
    else:
        return TriggerResponse(success=False, message="RECOGNITION_FAILED")


def main():
    global pub_result_str, pub_math, pub_fruit
    rospy.init_node("vlm_shoot_node", anonymous=False)
    rospy.loginfo("========================================")
    rospy.loginfo(">>> vlm_shoot_node 视觉大模型识别节点启动 <<<")
    rospy.loginfo("========================================")

    # 订阅相机图像
    rospy.Subscriber("/usb_cam/image_raw", ROSImage, image_callback, queue_size=1)

    # 结果发布者
    pub_result_str = rospy.Publisher("/vlm_result", String, queue_size=10)
    pub_math = rospy.Publisher("/vlm_shoot/math_result", Int32, queue_size=10)
    pub_fruit = rospy.Publisher("/vlm_shoot/fruit_code", Int32, queue_size=10)

    # 提供 ROS Service 触发方式
    rospy.Service("/trigger_vlm", Trigger, trigger_service_callback)

    # 兼容参数服务器触发机制 (如文档中的 rosparam set /detect 1 或 /top_view_shot_node/im_flag 1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        detect_param = rospy.get_param("/detect", 0)
        im_flag_param = rospy.get_param("/top_view_shot_node/im_flag", 255)

        if detect_param == 1 or im_flag_param == 1:
            rospy.loginfo("检测到参数触发信号，启动拍照识别...")
            rospy.set_param("/detect", 0)
            rospy.set_param("/top_view_shot_node/im_flag", 255)
            execute_vlm_recognition()

        rate.sleep()


if __name__ == "__main__":
    main()
