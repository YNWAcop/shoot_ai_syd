#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摄像头测试工具
功能：直接读取 /dev/video0 相机画面并保存测试图像
可用于验证摄像头物理连通性、对焦与画面清晰度
"""
import os
import sys
import cv2

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(CURRENT_DIR, "temp", "camera_test.jpg")

def test_camera(device_id=0):
    print("正在打开摄像头 /dev/video%d ..." % device_id)
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        print("错误: 无法打开 /dev/video%d，请检查相机连线！" % device_id)
        return False

    # 预热并丢弃前几帧以等待自动曝光稳定
    for _ in range(5):
        cap.read()

    ret, frame = cap.read()
    cap.release()

    if not ret or frame is None:
        print("错误: 摄像头打开成功但未能获取到图像帧！")
        return False

    h, w, c = frame.shape
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    cv2.imwrite(OUTPUT_PATH, frame)
    print("✓ 摄像头测试成功！")
    print("  画面分辨率: %dx%d, 通道数: %d" % (w, h, c))
    print("  测试照片已保存至: " + OUTPUT_PATH)
    return True

if __name__ == "__main__":
    dev = 0
    if len(sys.argv) > 1:
        dev = int(sys.argv[1])
    test_camera(dev)
