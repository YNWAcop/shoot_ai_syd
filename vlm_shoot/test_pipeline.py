#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
端到端识别测试工具 (单次执行)
功能：
1. 若指定了已有图片路径，直接对该图片进行分析
2. 若未指定图片，则实时调用摄像头拍摄一张新照片
3. 调用 vlm_api 执行分析，打印返回的两个数字结果 (计算题结果, 水果代码)
"""
import os
import sys
import cv2

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(CURRENT_DIR)

from vlm_api import call_vlm_api
from api_config import FRUIT_CODE_TO_NAME

TEMP_IMG = os.path.join(CURRENT_DIR, "temp", "test_shot.jpg")

def capture_photo(device_id=0):
    print("[1/2] 正在调用摄像头 /dev/video%d 拍摄实时画面..." % device_id)
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        raise RuntimeError("无法打开摄像头 /dev/video%d" % device_id)

    for _ in range(5):
        cap.read()
    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        raise RuntimeError("摄像头捕获帧失败！")

    os.makedirs(os.path.dirname(TEMP_IMG), exist_ok=True)
    cv2.imwrite(TEMP_IMG, frame)
    print("      照片拍摄完毕，已保存至: %s" % TEMP_IMG)
    return TEMP_IMG

def main():
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        img_path = sys.argv[1]
        print("[1/2] 使用指定图片进行测试: %s" % img_path)
    else:
        img_path = capture_photo(0)

    print("[2/2] 正在送入视觉大模型框架进行分析...")
    math_res, fruit_code = call_vlm_api(img_path)
    fruit_name = FRUIT_CODE_TO_NAME.get(fruit_code, "未知水果")

    print("")
    print("=" * 45)
    print("           ★ 测试识别结果 ★")
    print("  >> [第一个数字] 计算题结果 : %s" % math_res)
    print("  >> [第二个数字] 水果代号   : %s (%s)" % (fruit_code, fruit_name))
    print("  >> 每次返回两个数字        : %s %s" % (math_res, fruit_code))
    print("=" * 45)
    print("")

if __name__ == "__main__":
    main()
