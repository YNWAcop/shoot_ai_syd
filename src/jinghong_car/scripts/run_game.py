#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""jinghong_car 比赛主程序。

流程：播报开始 → 去识别区认卡片 → 播报靶号 → 1 号点打环形靶 → 2 号点打旋转靶 → 3 号点打移动靶 → 去终点播报结束。
走路用 move_base，环形靶和卡片用 find_object，AR 码用 ar_track_alvar，开枪发 /shoot 给厂家的 shoot 节点。
参数在 config/game.yaml。先用 launch/base_nodes.launch 启动底层节点，再运行 start_game.sh。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import collections
import math
import os
import subprocess
import sys
import time

import actionlib
import rospy
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config', 'game.yaml')

try:
    input = raw_input       # Python 2
except NameError:
    pass


def angle_diff(a, b):
    """a - b，换到 -pi ~ pi。"""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def median(values):
    values = sorted(values)
    n = len(values)
    if n == 0:
        return None
    return values[n // 2] if n % 2 else 0.5 * (values[n // 2 - 1] + values[n // 2])


def fit_circle(points):
    """三个以上的点拟合圆，返回 (圆心 u, 圆心 v, 半径)。"""
    n = len(points)
    su = sum(p[0] for p in points) / n
    sv = sum(p[1] for p in points) / n
    # 平移到重心附近再解，数值更稳
    uu = [p[0] - su for p in points]
    vv = [p[1] - sv for p in points]
    suu = sum(u * u for u in uu)
    svv = sum(v * v for v in vv)
    suv = sum(u * v for u, v in zip(uu, vv))
    suuu = sum(u * u * u for u in uu)
    svvv = sum(v * v * v for v in vv)
    suvv = sum(u * v * v for u, v in zip(uu, vv))
    svuu = sum(v * u * u for u, v in zip(uu, vv))
    det = suu * svv - suv * suv
    if abs(det) < 1e-9:
        return None
    b1 = 0.5 * (suuu + suvv)
    b2 = 0.5 * (svvv + svuu)
    cu = (b1 * svv - b2 * suv) / det
    cv = (suu * b2 - suv * b1) / det
    radius = math.sqrt(cu * cu + cv * cv + (suu + svv) / n)
    return cu + su, cv + sv, radius


class Car(object):
    """和 ROS 打交道的部分：走点、发速度、开枪、收识别结果。"""

    def __init__(self, cfg):
        self.cfg = cfg
        cam = cfg['camera']
        self.fx, self.fy, self.cx, self.cy = cam['fx'], cam['fy'], cam['cx'], cam['cy']
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.shoot_pub = rospy.Publisher(cfg['shoot_topic'], String, queue_size=1)
        self.init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.amcl_yaw = None
        self.odom_yaw = None
        self.ar_frames = collections.deque(maxlen=400)     # (时刻, {码号: (u, v)})
        self.ring_seen = collections.deque(maxlen=200)     # (时刻, 模板号, x)
        self.card_seen = collections.deque(maxlen=200)     # (时刻, 模板号)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._on_amcl)
        rospy.Subscriber('/odom', Odometry, self._on_odom)
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._on_ar)
        rospy.Subscriber('/object_position', Point, self._on_object)
        rospy.Subscriber('/objects', Float32MultiArray, self._on_objects)
        self.nav = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print('等 move_base ...')
        self.nav.wait_for_server(rospy.Duration(60))
        self._set_nav_speed()

    # ---------- 回调：只存数据 ----------
    def _on_amcl(self, msg):
        q = msg.pose.pose.orientation
        self.amcl_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.odom_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def _on_ar(self, msg):
        frame = {}
        for m in msg.markers:
            p = m.pose.pose.position
            if p.z > 0:
                frame[m.id] = (self.fx * p.x / p.z + self.cx, self.fy * p.y / p.z + self.cy)
        if frame:
            self.ar_frames.append((time.time(), frame))

    def _on_object(self, msg):
        if msg.z != 255 and msg.x > 0:
            self.ring_seen.append((time.time(), int(msg.z), msg.x))

    def _on_objects(self, msg):
        data = msg.data
        now = time.time()
        for i in range(0, len(data), 12):
            self.card_seen.append((now, int(data[i])))

    # ---------- 走 ----------
    def _set_nav_speed(self):
        """把 move_base 的最高速度改成 game.yaml 的 speed。"""
        speed = float(self.cfg['speed'])
        try:
            from dynamic_reconfigure.client import Client
            client = Client('/move_base/DWAPlannerROS', timeout=5)
            client.update_configuration({'max_vel_x': speed, 'min_vel_x': -speed, 'max_vel_y': speed,
                                         'min_vel_y': -speed, 'max_vel_trans': speed,
                                         'xy_goal_tolerance': float(self.cfg['goal_xy_tolerance']),
                                         'yaw_goal_tolerance': float(self.cfg['goal_yaw_tolerance'])})
            print('move_base 最高速度设为 %.2f m/s' % speed)
        except Exception as error:   # noqa: BLE001
            print('改 move_base 速度失败（%s），用 launch 里写的速度' % error)

    def set_start_pose(self, point):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x, msg.pose.pose.position.y = point[0], point[1]
        q = quaternion_from_euler(0, 0, math.radians(point[2]))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        self.init_pub.publish(msg)
        rospy.sleep(1.0)

    def go_to(self, point, timeout_s):
        """move_base 走到一个点。超时就取消，返回 False。"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = point[0], point[1]
        q = quaternion_from_euler(0, 0, math.radians(point[2]))
        o = goal.target_pose.pose.orientation
        o.x, o.y, o.z, o.w = q[0], q[1], q[2], q[3]
        self.nav.send_goal(goal)
        if not self.nav.wait_for_result(rospy.Duration(timeout_s)):
            self.nav.cancel_goal()
            self.stop()
            return False
        return True

    def move(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)

    def stop(self, settle_s=0.0):
        for _ in range(3):
            self.move(0.0, 0.0, 0.0)
            rospy.sleep(0.03)
        if settle_s > 0:
            rospy.sleep(settle_s)

    def turn_by(self, degrees, timeout_s=4.0):
        """原地转 degrees 度（左正），用里程计朝向反馈。"""
        if self.odom_yaw is None or abs(degrees) < 0.5:
            return
        target = self.odom_yaw + math.radians(degrees)
        sign = 1.0 if degrees > 0 else -1.0
        end = time.time() + timeout_s
        rate = rospy.Rate(20)
        while time.time() < end and not rospy.is_shutdown():
            left = angle_diff(target, self.odom_yaw)
            if abs(left) < math.radians(0.7) or left * sign < 0:
                break
            speed = min(0.6, max(0.3, 1.5 * abs(left)))   # 垫子上小于 0.3 转不动
            self.move(0.0, 0.0, math.copysign(speed, left))
            rate.sleep()
        self.stop(0.5)

    def face(self, yaw_deg, tolerance_deg, max_deg=15.0):
        """按 AMCL 朝向把车头转准到 yaw_deg。差得太多（定位可能还没更新）就不转。"""
        self.stop(0.5)                                   # 停稳，等 AMCL 更新
        if self.amcl_yaw is None:
            return
        diff = math.degrees(angle_diff(math.radians(yaw_deg), self.amcl_yaw))
        if tolerance_deg < abs(diff) <= max_deg:
            self.turn_by(diff)

    def shift(self, meters, speed):
        """横移 meters 米（左正），按时间走，走完停稳。"""
        duration = abs(meters) / speed / 0.92 + 0.3       # 0.92：横移实际速度比例；0.3：起步延迟
        end = time.time() + duration
        rate = rospy.Rate(20)
        while time.time() < end and not rospy.is_shutdown():
            self.move(0.0, math.copysign(speed, meters), 0.0)
            rate.sleep()
        self.stop(0.6)

    def fire(self):
        self.stop()                                      # 车本来就停着，不再等：等一下码就走开了
        self.shoot_pub.publish(String(data='shoot'))
        print('>>> 开枪')
        rospy.sleep(float(self.cfg['after_shot_s']))

    # ---------- 看 ----------
    def ring_x(self, look_s, ids):
        """看 look_s 秒，返回环形靶靶心 x 的中位数（没看到返回 None）。"""
        start = time.time()
        rospy.sleep(look_s)
        xs = [x for (t, i, x) in self.ring_seen if t >= start and i in ids]
        return median(xs)

    def codes_since(self, since):
        return [(t, f) for (t, f) in self.ar_frames if t >= since]

    def latest_code(self, code, max_age=0.3):
        for t, frame in reversed(self.ar_frames):
            if time.time() - t > max_age:
                return None
            if code in frame:
                return frame[code]
        return None


class Game(object):
    def __init__(self, car, cfg):
        self.car = car
        self.cfg = cfg
        self.t0 = None
        self.targets = list(cfg['cards']['default'])
        self.here = cfg['points']['start']          # 现在在哪个点（估剩余路程用）
        self.drive_s = 0.0                          # 这一局已经开了多少秒、多少米、几个点（估速度用）
        self.drive_m = 0.0
        self.goals = 0

    def elapsed(self):
        return time.time() - self.t0

    def say(self, name):
        v = self.cfg['voice']
        path = os.path.join(v['dir'], name)
        self.car.stop()
        try:
            proc = subprocess.Popen(list(v['player']) + [path])
            end = time.time() + 6.0
            while proc.poll() is None and time.time() < end:
                time.sleep(0.05)
            if proc.poll() is None:
                proc.kill()
        except OSError as error:
            print('播报失败：%s' % error)

    def drive(self, route_name):
        points = self.cfg['points']
        for name in self.cfg['routes'][route_name]:
            start = time.time()
            ok = self.car.go_to(points[name], float(self.cfg['goal_timeout_s']))
            self.drive_s += time.time() - start
            self.drive_m += math.hypot(points[name][0] - self.here[0], points[name][1] - self.here[1])
            self.here = points[name]
            self.goals += 1
            print('[%5.1f s] 到 %s %s' % (self.elapsed(), name, '' if ok else '（超时，接着走）'))
        last = self.cfg['routes'][route_name][-1]
        if self.cfg.get('final_turn') and last in ('p1', 'p2', 'p3'):
            self.car.face(points[last][2], float(self.cfg['turn_tolerance_deg']))

    def rest_of_route(self, after):
        """从现在的点走完 after 之后所有路段：返回 (直线距离 米, 点数)。"""
        order = ['recog', 'p1', 'p2', 'p3', 'end']
        points = self.cfg['points']
        here, total, count = self.here, 0.0, 0
        for route in order[order.index(after) + 1:]:
            for name in self.cfg['routes'][route]:
                total += math.hypot(points[name][0] - here[0], points[name][1] - here[1])
                here = points[name]
                count += 1
        return total, count

    def time_for(self, target, route_name):
        """这个靶最多能等几秒；时间不够返回 None（跳过）。"""
        overhead = float(self.cfg['goal_overhead_s'])
        nominal = 1.0 / float(self.cfg['speed'])
        pace = 1.5 * nominal
        if self.drive_m > 0.5:
            pace = (self.drive_s - overhead * self.goals) / self.drive_m   # 这一局实际每米花几秒
            pace = max(nominal, min(3.0 * nominal, pace))
        meters, count = self.rest_of_route(route_name)
        left = (float(self.cfg['match_time_s']) - self.elapsed() - pace * meters - overhead * count
                - float(self.cfg['end_reserve_s']))
        allowed = min(float(self.cfg[target]['timeout_s']), left)
        if allowed < float(self.cfg['min_target_s']):
            print('[%5.1f s] 时间不够，跳过 %s' % (self.elapsed(), target))
            return None
        return allowed

    # ---------- 识别 ----------
    def recognize(self):
        c = self.cfg['cards']
        answers = dict((int(k), v) for k, v in c['answers'].items())
        start = time.time()
        while time.time() - start < float(c['look_s']) and not rospy.is_shutdown():
            votes = collections.Counter(i for (t, i) in self.car.card_seen if t >= start and i in answers)
            if votes:
                best, count = votes.most_common(1)[0]
                if count >= int(c['min_votes']):
                    self.targets = list(answers[best])
                    print('认出卡片 %d → 旋转靶 %d 号、移动靶 %d 号' % (best, self.targets[0], self.targets[1]))
                    return
            rospy.sleep(0.1)
        print('没认出卡片，用默认靶号 %s' % self.targets)

    # ---------- 环形靶 ----------
    def shoot_ring(self, allowed_s):
        r = self.cfg['ring']
        ids = [int(i) for i in r['ids']]
        end = time.time() + allowed_s
        shifted = 0.0
        for _ in range(int(r['tries']) + 1):
            if time.time() > end:
                break
            x = self.car.ring_x(float(r['look_s']), ids)
            if x is None:
                print('没看到环形靶')
                continue
            error = x - float(r['aim_x'])
            print('环形靶偏差 %.1f 像素' % error)
            if abs(error) <= float(r['tolerance_px']):
                break
            want = -error / float(r['px_per_m'])                 # 靶心偏右 → 车往右移（负）
            limit = float(r['max_shift_m'])
            want = max(-limit - shifted, min(limit - shifted, want))
            if abs(want) < 0.004:
                break                                            # 到横移上限了
            self.car.shift(want, float(r['shift_speed']))
            shifted += want
        self.car.fire()

    # ---------- 旋转靶 ----------
    def wheel(self, look_s=1.0):
        """看 look_s 秒，每帧用 ≥3 个码拟合转轮，返回圆心和半径的中位数 (u, v, r)。"""
        since = time.time()
        rospy.sleep(look_s)
        fits = []
        for t, frame in self.car.codes_since(since):
            pts = [uv for code, uv in frame.items() if 1 <= code <= 5]
            if len(pts) >= 3:
                fit = fit_circle(pts)
                if fit is not None:
                    fits.append(fit)
        if not fits:
            return None
        return median([f[0] for f in fits]), median([f[1] for f in fits]), median([f[2] for f in fits])

    def shoot_rotating(self, allowed_s):
        r = self.cfg['rotating']
        aim_u, aim_v = float(r['aim'][0]), float(r['aim'][1])
        code = int(self.targets[0])
        end = time.time() + allowed_s
        # 1. 转车头，让转轮的圆经过弹着点
        for _ in range(int(r['align_tries'])):
            if time.time() > end - 3.0:
                break
            w = self.wheel()
            if w is None:
                print('看不清转轮')
                break
            uc, vc, radius = w
            dv = aim_v - vc
            half = math.sqrt(radius * radius - dv * dv) if abs(dv) < radius else 0.0
            # 圆和"弹着点那条水平线"的左右交点：让离得近的那个落到弹着点上
            target_uc = min((aim_u - half, aim_u + half), key=lambda u: abs(u - uc))
            move_px = target_uc - uc
            print('转轮圆心 (%.0f, %.0f) 半径 %.0f，要移 %.1f 像素' % (uc, vc, radius, move_px))
            if abs(move_px) <= float(r['tolerance_px']):
                break
            self.car.turn_by(move_px / float(r['px_per_deg']))  # 左转画面右移
        # 2. 等目标码转到弹着点附近
        rate = rospy.Rate(30)
        while time.time() < end and not rospy.is_shutdown():
            uv = self.car.latest_code(code)
            if uv is not None and math.hypot(uv[0] - aim_u, uv[1] - aim_v) <= float(r['tolerance_px']):
                print('%d 号码到了 (%.0f, %.0f)' % (code, uv[0], uv[1]))
                break
            rate.sleep()
        else:
            print('等太久，直接开枪')
        self.car.fire()

    # ---------- 移动靶 ----------
    def shoot_moving(self, allowed_s):
        m = self.cfg['moving']
        aim_u = float(m['aim_u'])
        code = int(self.targets[1])
        end = time.time() + allowed_s
        rate = rospy.Rate(30)
        while time.time() < end and not rospy.is_shutdown():
            uv = self.car.latest_code(code)
            if uv is not None and abs(uv[0] - aim_u) <= float(m['tolerance_px']):
                print('%d 号码到了 u=%.0f' % (code, uv[0]))
                break
            rate.sleep()
        else:
            print('等太久，直接开枪')
        self.car.fire()

    # ---------- 一整局 ----------
    def run(self):
        self.t0 = time.time()
        v = self.cfg['voice']
        self.say(v['start'])
        self.drive('recog')
        self.recognize()
        self.say(v['rotating'].format(self.targets[0]))
        self.say(v['moving'].format(self.targets[1]))
        self.drive('p1')
        allowed = self.time_for('ring', 'p1')
        if allowed:
            self.shoot_ring(allowed)
        self.drive('p2')
        allowed = self.time_for('rotating', 'p2')
        if allowed:
            self.shoot_rotating(allowed)
        self.drive('p3')
        allowed = self.time_for('moving', 'p3')
        if allowed:
            self.shoot_moving(allowed)
        self.drive('end')
        self.say(v['end'])
        print('总用时 %.1f 秒' % self.elapsed())


def main():
    with open(CONFIG) as handle:
        cfg = yaml.safe_load(handle)
    rospy.init_node('jinghong_game')
    car = Car(cfg)
    car.set_start_pose(cfg['points']['start'])
    print('车放在起点，回车开始：')
    input('')
    try:
        Game(car, cfg).run()
    finally:
        car.stop()


if __name__ == '__main__':
    main()
