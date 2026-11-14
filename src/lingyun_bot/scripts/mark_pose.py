#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""记点位：把车摆到位，运行 python2 mark_pose.py p1，当前 AMCL 位姿写进 config/task.yaml 的 points.p1。
不带参数只打印当前位姿。要先启动 bringup_all.launch，并且 AMCL 已经定好位。"""
from __future__ import print_function

import math
import os
import re
import sys

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'task.yaml')


def main():
    rospy.init_node('lingyun_mark', anonymous=True)
    msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=10)
    p = msg.pose.pose
    yaw = math.degrees(euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[2])
    value = '[%.3f, %.3f, %.1f]' % (p.position.x, p.position.y, yaw)
    print('当前位姿 %s' % value)
    if len(sys.argv) < 2:
        return
    name = sys.argv[1]
    with open(CONFIG) as handle:
        text = handle.read()
    start = text.index('\npoints:')
    end = text.index('\nroutes:')                   # 只改 points 这一节
    block = text[start:end]
    pattern = re.compile(r'^(  %s:\s*)\[[^\]]*\]' % re.escape(name), re.M)
    if not pattern.search(block):
        print('task.yaml 的 points 里没有 %s' % name)
        return
    text = text[:start] + pattern.sub(lambda m: m.group(1) + value, block, count=1) + text[end:]
    with open(CONFIG, 'w') as handle:
        handle.write(text)
    print('已写入 points.%s' % name)


if __name__ == '__main__':
    main()
