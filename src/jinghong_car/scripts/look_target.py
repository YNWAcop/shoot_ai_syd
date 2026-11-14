#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""看靶（车不动、不开枪），标弹着点、检查点位用。
  python2 look_target.py ring      环形靶：打印 find_object 的靶心 x 和离 aim_x 差多少
  python2 look_target.py 3         AR 码 3：打印像素位置和离弹着点差多少（1~5 旋转靶，6~8 移动靶）
  python2 look_target.py wheel     转轮：打印圆心、半径，以及要转多少度车头圆才经过弹着点
按 Ctrl+C 结束。"""
from __future__ import print_function

import math
import os
import sys

import rospy
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_game import CONFIG, fit_circle  # noqa: E402


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else 'ring'
    with open(CONFIG) as handle:
        cfg = yaml.safe_load(handle)
    cam = cfg['camera']
    rospy.init_node('jinghong_look', anonymous=True)

    def on_object(msg):
        if msg.z in cfg['ring']['ids']:
            print('环形靶 x=%.1f  离弹着点 %+.1f 像素' % (msg.x, msg.x - cfg['ring']['aim_x']))

    def on_ar(msg):
        pix = {}
        for m in msg.markers:
            p = m.pose.pose.position
            if p.z > 0:
                pix[m.id] = (cam['fx'] * p.x / p.z + cam['cx'], cam['fy'] * p.y / p.z + cam['cy'])
        if what == 'wheel':
            pts = [uv for k, uv in pix.items() if 1 <= k <= 5]
            if len(pts) >= 3:
                fit = fit_circle(pts)
                if fit:
                    au, av = cfg['rotating']['aim']
                    dv = av - fit[1]
                    half = math.sqrt(fit[2] ** 2 - dv ** 2) if abs(dv) < fit[2] else 0.0
                    want = min((au - half, au + half), key=lambda u: abs(u - fit[0]))
                    print('圆心 (%.0f, %.0f) 半径 %.0f；车头再转 %+.1f 度圆就经过弹着点' % (
                        fit[0], fit[1], fit[2], (want - fit[0]) / cfg['rotating']['px_per_deg']))
        elif what.isdigit() and int(what) in pix:
            u, v = pix[int(what)]
            if int(what) <= 5:
                au, av = cfg['rotating']['aim']
            else:
                au, av = cfg['moving']['aim_u'], v
            print('%s 号码 (%.1f, %.1f)  离弹着点 (%+.1f, %+.1f)' % (what, u, v, u - au, v - av))

    if what == 'ring':
        rospy.Subscriber('/object_position', Point, on_object)
    else:
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, on_ar)
    rospy.spin()


if __name__ == '__main__':
    main()
