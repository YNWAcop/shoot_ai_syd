# -*- coding: utf-8 -*-
"""Wait for post-calibration IMU streams without starting the mission.

The abot_imu driver publishes /imu/data_raw only AFTER calibration finishes.
The perform_calibration ROS parameter is not updated on completion, so it
must not be used as a readiness flag.
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import math
import threading
import time


class FreshStream(object):
    def __init__(self):
        self.first = None
        self.last = None
        self.stamp = None
        self.count = 0

    def observe(self, now, stamp, ros_now, valid=True):
        if (not valid or stamp <= 0 or not -0.1 <= ros_now - stamp <= 0.5):
            self.__init__()
            return
        if self.last is not None and (now - self.last > 0.5 or stamp < self.stamp):
            self.__init__()
        if self.stamp is not None and stamp <= self.stamp:
            return
        if self.first is None:
            self.first = now
        self.last, self.stamp = now, stamp
        self.count += 1

    def ready(self, now, ros_now):
        return (self.count >= 5 and self.last - self.first >= 1.0
                and 0 <= now - self.last <= 0.5
                and -0.1 <= ros_now - self.stamp <= 0.5)


def wait_for_imu_ready(log, timeout_s=60.0):
    import rospy
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Twist

    streams = {'raw': FreshStream(), 'filtered': FreshStream()}
    lock = threading.Lock()
    subscribers = []
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    clock = getattr(time, 'monotonic', time.time)

    def callback(msg, key):
        values = [msg.angular_velocity.x, msg.angular_velocity.y,
                  msg.angular_velocity.z, msg.linear_acceleration.x,
                  msg.linear_acceleration.y, msg.linear_acceleration.z]
        valid = all(not math.isnan(v) and not math.isinf(v) for v in values)
        if key == 'filtered':
            q = msg.orientation
            norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            valid = valid and 0.5 < norm < 1.5
        with lock:
            streams[key].observe(clock(), msg.header.stamp.to_sec(),
                                 rospy.get_time(), valid)

    started = clock()
    next_report = started + 5.0
    log.event('preflight', 'imu_wait',
              '等待 IMU 校准完成及数据稳定，请保持车身静止；最长等待 60 秒')
    try:
        subscribers.append(rospy.Subscriber('/imu/data_raw', Imu, callback,
                                            callback_args='raw', queue_size=10))
        subscribers.append(rospy.Subscriber('/imu/data', Imu, callback,
                                            callback_args='filtered', queue_size=10))
        while not rospy.is_shutdown():
            now = clock()
            publisher.publish(Twist())
            with lock:
                ready = all(stream.ready(now, rospy.get_time()) for stream in streams.values())
            if ready:
                log.event('preflight', 'imu_ready',
                          'IMU 原始及融合数据已连续更新至少 1 秒，等待用时 %.1f 秒' % (now - started))
                return True
            if now - started >= timeout_s:
                log.event('preflight', 'imu_timeout',
                          'IMU 未就绪，取消启动；检查 IMU 数据和校准状态')
                return False
            if now >= next_report:
                log.info('IMU 尚未就绪，继续保持静止（已等待 %.0f 秒）' % (now - started))
                next_report = now + 5.0
            time.sleep(0.05)
        return False
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        publisher.unregister()
