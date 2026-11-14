# -*- coding: utf-8 -*-
"""运动控制的纯计算部分：输入传感器数据，输出速度指令 (vx, vy, wz)。

这里不发布任何 ROS 消息。速度最终由 ros_io.RosIO.send_velocity() 发出，
全程序只有这一个出口，避免旧程序里多个线程同时往 /cmd_vel 发指令互相打架。

本底盘是麦克纳姆轮，可以同时前后、左右、旋转（vx, vy, wz 三个自由度）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import math

import numpy as np

from .geometry import SIDE_DIRECTION, clamp, map_to_base, wrap_angle


class VelocityCommand(object):
    """一条速度指令（机器人坐标系）：vx 前进(m/s)，vy 左移(m/s)，wz 逆时针转(rad/s)。"""

    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.vx = vx
        self.vy = vy
        self.wz = wz

    def is_zero(self):
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0

    def __repr__(self):
        return 'Cmd(vx=%.3f, vy=%.3f, wz=%.3f)' % (self.vx, self.vy, self.wz)


STOP = VelocityCommand(0.0, 0.0, 0.0)


def speed_with_minimum(error, gain, max_speed, min_speed, tolerance):
    """比例控制 + 最小速度。

    误差在容差内 → 输出 0（并且主动发 0，让车真正停住）。
    误差在容差外 → 输出 gain*error，限幅到 max_speed，且绝对值不低于 min_speed。

    为什么要最小速度：底盘驱动把速度乘 100 后取整（cm/s、0.01rad/s），
    0.009 m/s 会被截成 0，再加上电机死区，太小的指令车根本不动。
    """
    if abs(error) <= tolerance:
        return 0.0
    speed = clamp(gain * error, -max_speed, max_speed)
    if abs(speed) < min_speed:
        speed = math.copysign(min_speed, speed)
    return speed


def hold_heading(drift_rad, gain=1.5, max_rate=0.2, deadband_rad=math.radians(0.15)):
    """横移时稳车头：车头相对横移开始时偏了 drift_rad（正 = 偏左），返回反向转回来的角速度（rad/s）。

    为什么要：垫子上横移时车头会跟着转。2026-10-19 22:29 菜单 6（旋转靶演练）：激光量到车向右横移了 25 mm、20 mm，
    转轮圆心在画面里只动了 -2.2、+0.6 像素（理论各约 -11、-9 像素），说明车头同时向左转了约 1 度，横移等于白做，
    最后只能偏 8 像素"尽力而为"开火。菜单 22 横移时一直用这个（同一面侧墙的偏角做反馈），
    2026-10-19 录包两次横移（114 mm、27 mm）车头只偏了 0.06、0.12 度。
    """
    if drift_rad is None or abs(drift_rad) <= deadband_rad:
        return 0.0
    return max(-max_rate, min(max_rate, -gain * drift_rad))


class VelocitySmoother(object):
    """限制加速度：每个控制周期速度变化不超过 accel * dt，避免急启急停打滑、晃动相机。"""

    def __init__(self, linear_accel, angular_accel):
        self.linear_accel = linear_accel
        self.angular_accel = angular_accel
        self.current = VelocityCommand()

    def reset(self):
        self.current = VelocityCommand()

    def smooth(self, target, dt):
        def step_toward(now, goal, max_change):
            return now + clamp(goal - now, -max_change, max_change)

        lin_step = self.linear_accel * dt
        ang_step = self.angular_accel * dt
        self.current = VelocityCommand(
            step_toward(self.current.vx, target.vx, lin_step),
            step_toward(self.current.vy, target.vy, lin_step),
            step_toward(self.current.wz, target.wz, ang_step))
        return self.current


class ChassisModel(object):
    """底盘响应模型：预测"现在开始发零速度，车会滑到哪里停下"。

    2026-10-17 全流程录包（full_212336.bag）实测：发出速度指令后
      - 死区约 0.3 秒（0.35~0.5 秒才开始动），之后按一阶惯性跟上，时间常数约 0.3 秒；
      - 稳态只能到指令的一部分：后退约 0.9、横移约 0.8、前进 0.6~1.0（横移后再前进时最低）；
        原地转只有边走边转的 1/3 左右（2026-10-16 原地转速测试）。
    旧的路径跟随和精定位没考虑这些：拐角处指令已经换方向，车还沿原方向滑 1 秒（冲过 8 cm）；
    精定位冲过 4~5 cm 再倒回来。现在两者都按"预测停车点"控制：预测点到了就提前收油门。

    record() 每个控制周期调用一次（Context.publish_motion 里，记录真正发出去的指令）；
    能拿到轮速计时，用它修正 vx、wz 的估计（/wheel_odom 的横向速度恒为 0，vy 只能靠模型）。
    """

    def __init__(self, params, dt=0.05):
        self.dead = float(params.get('dead_time_s', 0.3))
        self.tau = max(0.02, float(params.get('time_constant_s', 0.3)))
        self.gain_forward = float(params.get('gain_forward', 1.0))
        self.gain_backward = float(params.get('gain_backward', 1.0))
        self.gain_lateral = float(params.get('gain_lateral', 1.0))
        self.gain_yaw = float(params.get('gain_yaw', 1.0))
        self.gain_yaw_inplace = float(params.get('gain_yaw_inplace', self.gain_yaw))
        self.wheel_blend = float(params.get('wheel_blend', 0.0))
        self.wheel_blend_yaw = float(params.get('wheel_blend_yaw', self.wheel_blend))
        self.dt = dt
        self.history = []            # [(时刻, vx, vy, wz)] 发出的指令
        self.velocity = [0.0, 0.0, 0.0]
        self.last_time = None

    def _gain(self, axis, command):
        if axis == 0:
            return self.gain_forward if command[0] >= 0 else self.gain_backward
        if axis == 1:
            return self.gain_lateral
        in_place = abs(command[0]) + abs(command[1]) < 1e-3     # 原地转比边走边转弱得多（2026-10-16 实测约 1/3）
        return self.gain_yaw_inplace if in_place else self.gain_yaw

    def _command_at(self, t):
        """t 时刻底盘"正在执行"的指令 = t - 死区 时刻发出的指令。"""
        target = t - self.dead
        chosen = (0.0, 0.0, 0.0)
        for item in self.history:
            if item[0] <= target + 1e-9:
                chosen = item[1:]
            else:
                break
        return chosen

    def _step(self, velocity, command, dt):
        k = min(1.0, dt / self.tau)
        return [v + (self._gain(i, command) * c - v) * k for i, (v, c) in enumerate(zip(velocity, command))]

    def record(self, now, command, wheel=None):
        if self.last_time is not None:
            t = self.last_time
            while t < now - 1e-9:
                step = min(self.dt, now - t)
                self.velocity = self._step(self.velocity, self._command_at(t), step)
                t += step
        if wheel is not None:
            self.velocity[0] += (wheel[0] - self.velocity[0]) * self.wheel_blend
            self.velocity[2] += (wheel[2] - self.velocity[2]) * self.wheel_blend_yaw
        self.last_time = now
        self.history.append((now, command.vx, command.vy, command.wz))
        oldest = now - self.dead - 0.5
        while len(self.history) > 2 and self.history[1][0] < oldest:
            self.history.pop(0)

    def reset(self):
        self.history = []
        self.velocity = [0.0, 0.0, 0.0]
        self.last_time = None

    def speed(self):
        return math.hypot(self.velocity[0], self.velocity[1])

    def is_stopped(self, hold_s=0.3):
        """最近 hold_s 秒发的都是零、估计速度也接近零。"""
        if self.last_time is None:
            return True
        recent = [h for h in self.history if h[0] >= self.last_time - hold_s]
        commands_zero = all(abs(h[1]) + abs(h[2]) + abs(h[3]) < 1e-6 for h in recent)
        moving = abs(self.velocity[0]) + abs(self.velocity[1]) > 0.01 or abs(self.velocity[2]) > 0.03
        return commands_zero and not moving

    def speed_after(self, seconds):
        """从现在起只发零速度，seconds 秒后车的平移速度（m/s）。
        用法：新方向的指令要过死区才起作用，"死区后速度已经接近 0"时发新指令，新方向一起步车就是停住的（分段档位 settle）。"""
        if self.last_time is None:
            return 0.0
        velocity = list(self.velocity)
        t = self.last_time
        while t < self.last_time + seconds - 1e-9:
            command = self._command_at(t + 1e-6)
            if t - self.dead > self.last_time:
                command = (0.0, 0.0, 0.0)
            velocity = self._step(velocity, command, self.dt)
            t += self.dt
        return math.hypot(velocity[0], velocity[1])

    def predict_stop(self):
        """从现在起只发零速度时，车还会走多远。

        返回 (dx, dy, dyaw, dyaw_dead)：dx/dy/dyaw 是最终停下时相对现在的位移（现在的车体坐标系），
        dyaw_dead 是死区结束时（现在发的指令开始起作用的时刻）车身已经转过的角度。
        """
        if self.last_time is None:
            return 0.0, 0.0, 0.0, 0.0
        velocity = list(self.velocity)
        x = y = yaw = 0.0
        yaw_dead = None
        t = self.last_time
        end = t + self.dead + 6.0 * self.tau
        while t < end:
            command = self._command_at(t + 1e-6)
            if t - self.dead > self.last_time:
                command = (0.0, 0.0, 0.0)          # 以后发的都是零
            velocity = self._step(velocity, command, self.dt)
            x += (velocity[0] * math.cos(yaw) - velocity[1] * math.sin(yaw)) * self.dt
            y += (velocity[0] * math.sin(yaw) + velocity[1] * math.cos(yaw)) * self.dt
            yaw += velocity[2] * self.dt
            t += self.dt
            if yaw_dead is None and t >= self.last_time + self.dead:
                yaw_dead = yaw
            if t > self.last_time + self.dead and abs(velocity[0]) + abs(velocity[1]) < 1e-3 and abs(velocity[2]) < 2e-3:
                break
        return x, y, yaw, (yaw if yaw_dead is None else yaw_dead)


def limit_wheel_speed(command, wheel_limit, wheel_k):
    """麦轮单个轮子的线速度约为 |vx| + |vy| + k|wz|。超过底盘能力时轮子饱和，平移会被"吃掉"、方向也会歪。
    2026-10-17 实测：边走边转 1.5 rad/s 时，实际只转到 1.3、前进只剩一半。
    超了就按比例缩小平移（方向不变），转向保留。"""
    if wheel_limit is None or wheel_limit <= 0:
        return command
    rotation = wheel_k * abs(command.wz)
    translation = abs(command.vx) + abs(command.vy)
    if translation + rotation <= wheel_limit or translation < 1e-9:
        return command
    scale = max(0.0, wheel_limit - rotation) / translation
    return VelocityCommand(command.vx * scale, command.vy * scale, command.wz)


def limit_wheel_speed_keep(keep, along, wheel_limit, wheel_k):
    """和 limit_wheel_speed 一样限制单轮速度，但先保住 keep（纠偏分量 + 转向），只缩小 along（沿路线的分量）。

    keep、along 都是车体坐标系的 VelocityCommand（along.wz 不用）。返回合成后的指令。
    用在通道横移：2026-10-20 16:19 录包，横移 0.48 m/s 时纠偏的 -0.07 m/s 后退被按比例一起缩掉、
    又被轮速饱和吃掉，车一直停在离挡板端头 0.6~4 cm 的地方横移过去。"""
    def wheel(f):
        return abs(keep.vx + f * along.vx) + abs(keep.vy + f * along.vy) + wheel_k * abs(keep.wz)
    if wheel_limit is None or wheel_limit <= 0 or wheel(1.0) <= wheel_limit:
        return VelocityCommand(keep.vx + along.vx, keep.vy + along.vy, keep.wz)
    if wheel(0.0) > wheel_limit:
        # 纠偏 + 转向本身就超了：只保留它们，再按比例缩
        return limit_wheel_speed(VelocityCommand(keep.vx, keep.vy, keep.wz), wheel_limit, wheel_k)
    low, high = 0.0, 1.0                     # wheel(f) 是 f 的凸函数，f=0 时满足、f=1 时超 → 二分找最大的 f
    for _ in range(20):
        mid = 0.5 * (low + high)
        if wheel(mid) <= wheel_limit:
            low = mid
        else:
            high = mid
    return VelocityCommand(keep.vx + low * along.vx, keep.vy + low * along.vy, keep.wz)


class PathFollower(object):
    """沿一串路径点开过去（不经过 move_base）。2026-10-17 按实测底盘响应重写。

    思路：
      1. 路径 = 起点 → 各路径点 的折线。每个周期先用 ChassisModel 预测"现在松油门车会停在哪"，
         后面的计算都用这个预测点（抵消底盘 0.3 秒死区 + 惯性，不再冲过拐角和终点）。
      2. 方向：预测点投影到折线上，沿折线往前 lookahead 处取"引导点"，速度指向引导点
         → 拐角自然变成圆弧，偏离路线也会被拉回来。
         急弯（转角 ≥ sharp_corner_deg，比如通道点处的 90 度弯）不切角：预测点离拐点不到 corner_reach_m 之前，
         引导点不越过拐点（2026-10-18 实测：通道两侧是围栏和挡板端头，圆弧切角会让车头蹭到挡板）。
      3. 速度：min(最高速度, 到终点的减速曲线 √(2·a·剩余路程), 每个拐角前按转角降速)。
      4. 朝向：每段终点规定的朝向，用预测朝向做比例控制，限速 max_yaw_rate，1 度以内不修；
         只剩原地转时见 turn_in_place（不低于 min_turn_rate，预测转到位就收）。
         要转的角度大时（去识别区、从识别区出来都要转 90 度）按"走到这个点时刚好转完"限制平移速度：
         原地转比边走边转慢得多（2026-10-18 垫子上实测：边走边转约 0.36、原地只有 0.15~0.3），
         先到点再原地转，又慢、又会在终点来回点动。
         离终点不到 turn_hold_m、朝向还没到：平移锁住只转向，离开 turn_release_m 才重新平移
         （2026-10-18 实测：到点时还差 27 度，位置误差 1 cm 时减速曲线仍给 0.1 m/s，车在终点前后左右点动 5 秒）。
      5. 速度换算到车体坐标系时，用"指令真正起作用时"的车身朝向（边走边转时差十几度）。
      6. 轮速饱和时按比例缩小平移（limit_wheel_speed）；输出加一阶低通（约 0.07 秒），位姿的小跳动不变成方向抖动。
      6b. 分段档位（2026-10-20）：segments 给出每一段的最高速度、减速度、末端速度。
         爆发段（离开任务框往后退、通道里横移）：最高速度高、加速猛（加速度上限由 DriveStep 按段设给平滑器），
         末端速度 = 0：按预测停车点刹到拐点（死区 0.3~0.5 秒里车还在滑，新方向的指令起作用时车已经基本停住，
         效果就是"停住再横移"，但不用干等）；settle = true 时到拐点还要等车完全停稳才开下一段。
         常规段（进任务框）：速度低、停得准，精定位剩得少。
         做法和 RoboCup 小型组 TIGERs 的 BangBangTrajectory1D 一样（加速 → 匀速 → 减速到 0 的时间最优三段），
         区别是减速按"预测停车点"算，抵消了底盘死区。
      7. 完成：离终点 goal_tolerance 内、朝向（现在和预测停下时）都在 handoff_yaw 内；或车已停住、离终点 close_finish_m 内
         （低速点动底盘常常不起步，不如直接交给贴墙精定位）。车停着时不做预测（同样的原因，模型会以为在动）。
         handoff_yaw（交接朝向容差，默认 6 度）比 yaw_tolerance 宽：每个行驶步骤后面都是贴墙精定位，它按墙转朝向，比 AMCL 准。
         2026-10-18 22:19 录包：去 1 号点时 AMCL 说还差 4.6 度（容差 4 度），按墙量其实只差 1 度；
         行驶在原地"发一下、停一下"地补这 0.6 度，补不动，干等 2.3 秒到超时。

    waypoints：[(x, y, yaw_rad), ...] 地图坐标。没有 chassis 模型时退化为不做预测。
    每个 drive 步骤新建一个 PathFollower（起点 = 第一次 update 时的位置）。
    """

    def __init__(self, waypoints, max_speed, corner_speed, decel, max_yaw_rate,
                 yaw_gain, goal_tolerance, yaw_tolerance,
                 lookahead_m=0.20, lookahead_s=0.4, wheel_limit=None, wheel_k=0.2,
                 min_turn_rate=0.45, straight_angle_deg=15.0, close_finish_m=0.08,
                 sharp_corner_deg=60.0, corner_reach_m=0.05, turn_hold_m=0.05, turn_release_m=0.10,
                 handoff_yaw=None, segments=None, cross_gain=2.5, cross_max=0.12, burst_switch_m=0.015,
                 yaw_sync=1.0, yaw_bandwidth=None):
        self.waypoints = list(waypoints)
        self.max_speed = max_speed
        self.corner_speed = corner_speed
        self.decel = decel
        self.max_yaw_rate = max_yaw_rate
        self.yaw_gain = yaw_gain
        self.goal_tolerance = goal_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.handoff_yaw = max(yaw_tolerance, handoff_yaw if handoff_yaw is not None else yaw_tolerance)
        # 每一段的运动参数（config.path_segments）：max_speed / decel / end_speed / wheel_limit / settle。
        # 不给就全程用上面的常规参数
        self.segments = [dict(seg) for seg in segments] if segments else None
        self.settled = set()                    # 已经停稳过的拐点（settle 段）
        self.lookahead_m = lookahead_m
        self.lookahead_s = lookahead_s
        self.wheel_limit = wheel_limit
        self.wheel_k = wheel_k
        self.min_turn_rate = min_turn_rate
        self.straight_angle = math.radians(straight_angle_deg)
        self.close_finish_m = close_finish_m
        self.sharp_angle = math.radians(sharp_corner_deg)
        self.corner_reach_m = corner_reach_m
        self.turn_hold_m = turn_hold_m
        self.turn_release_m = turn_release_m
        # 贴线段（爆发档 track: true）：沿线速度 + 横向纠偏 = -cross_gain × 偏离线的距离（不超过 cross_max），
        # 轮速超限时先缩沿线速度、保住纠偏（limit_wheel_speed_keep）
        self.cross_gain = cross_gain
        self.cross_max = cross_max
        # 末端速度为 0 的段（爆发段）：预测停车点沿路线离拐点不到 burst_switch_m 才切到下一段。
        # 2026-10-20 16:19 录包：原来按"离拐点 corner_reach_m（5 cm）以内"就切，横移时车离通道线还差 6~9 cm，
        # 车头离挡板端头只剩 0.6~4 cm，安全检查连停 5 次
        self.burst_switch_m = burst_switch_m
        # 大角度转向时按"走到这一段终点刚好转完"限制平移速度，假设能按 max_yaw_rate × gain_yaw × yaw_sync 转
        self.yaw_sync = yaw_sync
        # 朝向带宽（1/s）：给了就按"实际转速 = yaw_bandwidth × 朝向误差"发指令，即指令 = yaw_bandwidth / 底盘转向增益 × 误差。
        # 2026-10-21：原来 yaw_gain 3.0 是指令增益，垫子上边走边转只转出 0.36、原地 0.25，实际只有 1.1 / 0.75 每秒，
        # 最后 30 度拖 1 秒多、1 号点交接时还差 7~11 度，再交给精定位按 0.05 rad/s 慢慢蹭。不给就照旧用 yaw_gain
        self.yaw_bandwidth = yaw_bandwidth
        self.holding = False                    # 已到点、只剩转向：平移锁住（见 update）
        self.min_turn_speed = 0.08              # 为了边走边转而降速时不低于这个（底盘 0.05 m/s 以下起步很慢）
        self.output_smoothing = 0.5             # 一阶低通系数（20 Hz 下时间常数约 0.07 秒）
        self.last_command = None
        self.points = None                      # 折线顶点 [(x, y)]，第 0 个是出发点
        self.cum = None                         # 各顶点的累计路程
        self.segment = 0                        # 当前所在的折线段（顶点 segment → segment+1）

    def is_last(self):
        return self.segment >= len(self.waypoints) - 1

    def set_waypoint_x(self, index, x):
        """把第 index 个路径点（从 0 数）的 x 改成 x（通道点按激光量后围栏重新定位时用，见 DriveStep）。"""
        self._move_waypoint(index, x, self.waypoints[index][1])

    def set_waypoint_y(self, index, y):
        self._move_waypoint(index, self.waypoints[index][0], y)

    def _move_waypoint(self, index, x, y):
        self.waypoints[index] = (x, y, self.waypoints[index][2])
        if self.points is not None:
            self.points[index + 1] = (x, y)
            self.cum = [0.0]
            for a, b in zip(self.points, self.points[1:]):
                self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

    def turn_in_place(self, pose, chassis=None, target_yaw=None):
        """只转不平移：返回 (wz, 是否已转到位)。target_yaw 默认是终点朝向。

        底盘原地 0.45 rad/s 以下基本转不动，所以转速不低于 min_turn_rate；
        车在转时按"预测停下时的朝向"判断，预测到位就收（死区 + 惯性会再转一截，2026-10-17 审查仿真：
        不预测时去识别区转 90 度会多转 13 度）。车停着时按当前朝向，保证指令持续到车真正转起来。
        """
        tolerance = self.yaw_tolerance
        if target_yaw is None:
            target_yaw = self.waypoints[-1][2]
            tolerance = self.handoff_yaw            # 终点朝向：转到交接容差内就交给贴墙精定位
        now_error = wrap_angle(target_yaw - pose[2])
        stopped = chassis is None or chassis.is_stopped()
        predicted_error = now_error if stopped else wrap_angle(now_error - chassis.predict_stop()[2])
        if not stopped and abs(chassis.velocity[2]) < 0.03 and abs(now_error) >= tolerance \
                and abs(predicted_error) < tolerance:
            # 刚发出转向指令、模型里车还在死区没转起来：预测只多算了死区里那一点，不能据此就收。
            # 否则"发一个周期 → 预测到位收掉 → 停 0.3 秒又发"，加速度限幅下每次只发出 0.15 rad/s，车根本不转
            # （2026-10-18 22:19 去 1 号点：差 4.6 度、容差 4 度，这样空转了 2.3 秒）。
            predicted_error = math.copysign(tolerance, now_error)
        if abs(predicted_error) < tolerance:
            return 0.0, stopped and abs(now_error) < tolerance
        rate = max(min(abs(self._yaw_gain(chassis, True) * predicted_error), self.max_yaw_rate), self.min_turn_rate)
        return math.copysign(rate, predicted_error), False

    def _yaw_gain(self, chassis, in_place):
        """朝向比例增益（指令 rad/s / 误差 rad）。"""
        if self.yaw_bandwidth is None or chassis is None:
            return self.yaw_gain
        gain = chassis.gain_yaw_inplace if in_place else chassis.gain_yaw
        return self.yaw_bandwidth / max(gain, 0.1)

    def _build(self, x, y):
        self.points = [(x, y)] + [(w[0], w[1]) for w in self.waypoints]
        self.cum = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

    def _project(self, px, py):
        """预测点投影到折线：返回 (路程 s, 段号)。只往前找（当前段和下一段），进度不会倒退。"""
        best = None
        last = min(self.segment + 1, len(self.points) - 2)
        if self.segments and self._segment_params(self.segment)['end_speed'] is not None:
            last = self.segment         # 规定了末端速度的段：只能由 update 里的"到拐点"判断切到下一段（settle 要先等停住）
        for i in range(self.segment, last + 1):
            (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
            length = self.cum[i + 1] - self.cum[i]
            if length < 1e-9:
                t, d = 0.0, math.hypot(px - ax, py - ay)
            else:
                t = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / length
                t = clamp(t, 0.0, length)
                d = math.hypot(ax + (bx - ax) * t / length - px, ay + (by - ay) * t / length - py)
            # 下一段要明显更近才切过去（拐角处两段一样近时留在当前段）
            if best is None or d < best[0] - (1e-3 if i > self.segment else 0.0):
                best = (d, self.cum[i] + t, i)
        self.segment = best[2]
        return best[1], best[2]

    def _point_at(self, s):
        s = clamp(s, 0.0, self.cum[-1])
        for i in range(len(self.points) - 1):
            if s <= self.cum[i + 1] + 1e-9:
                length = self.cum[i + 1] - self.cum[i]
                t = 0.0 if length < 1e-9 else (s - self.cum[i]) / length
                (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
                return ax + (bx - ax) * t, ay + (by - ay) * t
        return self.points[-1]

    def _turn_angle(self, vertex):
        """顶点 vertex 处折线转过的角度（弧度）。"""
        (ax, ay), (bx, by), (cx, cy) = self.points[vertex - 1], self.points[vertex], self.points[vertex + 1]
        a1 = math.atan2(by - ay, bx - ax)
        a2 = math.atan2(cy - by, cx - bx)
        return abs(wrap_angle(a2 - a1))

    def _segment_params(self, index):
        """第 index 段（开到第 index 个路径点）的运动参数。"""
        if self.segments:
            seg = self.segments[max(0, min(index, len(self.segments) - 1))]
            return {'max_speed': float(seg.get('max_speed', self.max_speed)), 'decel': float(seg.get('decel', self.decel)),
                    'end_speed': seg.get('end_speed'), 'wheel_limit': seg.get('wheel_limit', self.wheel_limit),
                    'settle': bool(seg.get('settle', False)), 'settle_speed': seg.get('settle_speed', 0.03),
                    'accel': seg.get('accel'), 'track': bool(seg.get('track', False)),
                    'profile': seg.get('profile', 'normal')}
        return {'max_speed': self.max_speed, 'decel': self.decel, 'end_speed': None, 'wheel_limit': self.wheel_limit,
                'settle': False, 'accel': None, 'track': False, 'profile': 'normal'}

    def current_segment(self):
        """现在在走的那一段的运动参数（DriveStep 用它设加速度上限）。"""
        return self._segment_params(self.segment)

    def update(self, pose, chassis=None):
        """pose = (x, y, yaw)。返回 (VelocityCommand, finished, 到终点的直线距离)。"""
        x, y, yaw = pose
        if self.points is None:
            self._build(x, y)
        goal_x, goal_y, goal_yaw = self.waypoints[-1]
        distance = math.hypot(goal_x - x, goal_y - y)

        # 1. 预测停车点（地图坐标）。车停着时不预测：低速小指令底盘常常根本不起步，模型会以为在动
        stopped = chassis is not None and chassis.is_stopped()
        if chassis is not None and not stopped:
            dx, dy, dyaw, dyaw_dead = chassis.predict_stop()
        else:
            dx, dy, dyaw, dyaw_dead = 0.0, 0.0, 0.0, 0.0
        px = x + math.cos(yaw) * dx - math.sin(yaw) * dy
        py = y + math.sin(yaw) * dx + math.cos(yaw) * dy
        pyaw = yaw + dyaw

        # 完成判断：朝向要现在和预测停下时都在交接容差内（否则还在转，停下时会超出）。
        # 交接容差 handoff_yaw 比 yaw_tolerance 宽：最后几度交给贴墙精定位按墙转（见类说明第 7 条）
        yaw_now_error = abs(wrap_angle(goal_yaw - yaw))
        yaw_pred_error = abs(wrap_angle(goal_yaw - pyaw))
        yaw_ok = yaw_now_error < self.handoff_yaw and yaw_pred_error < self.handoff_yaw
        if distance < self.goal_tolerance and yaw_ok:
            return STOP, True, distance
        if stopped and self.points is not None and self.segment >= len(self.points) - 2 \
                and distance < self.close_finish_m and yaw_ok:
            # 车已经停住、离终点几厘米：剩下的交给贴墙精定位（低速点动底盘常常不起步，22:06 实测反复点动 2~3 秒）
            return STOP, True, distance

        # 到点后只剩转向：平移锁住（滞回：离开 turn_release_m 才重新平移）
        last_leg = self.segment >= len(self.points) - 2
        if last_leg and not yaw_ok and (distance < self.turn_hold_m or (self.holding and distance < self.turn_release_m)):
            self.holding = True
            wz, turned = self.turn_in_place(pose, chassis)
            if turned:
                return STOP, True, distance
            self.last_command = VelocityCommand(0.0, 0.0, wz)
            return self.last_command, False, distance
        self.holding = False

        # 2. 进度、剩余路程
        s, segment = self._project(px, py)
        # 规定了末端速度的段（爆发段，末端 = 0）：预测停车点到了 / 越过了本段终点，就（settle 段等车基本停住后）切到下一段。
        # 不主动切的话，到拐点时减速曲线给的速度正好是 0，车会停在拐点不动
        # （仿真：死区 0.5 秒时冲过拐点 6 cm，投影到两段一样近，停了 5.5 秒）
        while segment < len(self.points) - 2 and self._segment_params(segment)['end_speed'] is not None:
            vertex = segment + 1
            ending = self._segment_params(segment)
            if self.cum[vertex] - s > self.burst_switch_m:
                break                           # 预测停车点沿路线还没到拐点（见 burst_switch_m 说明）
            if ending['settle'] and vertex not in self.settled:
                # 先发零速度，等到"现在发新指令、过了死区起作用时车已经基本停住"再开下一段（停住再横移）。
                # 直接等车停稳再发会多耗一个死区（0.3~0.5 秒）
                if chassis is None or stopped or chassis.speed_after(chassis.dead) < float(ending['settle_speed']):
                    self.settled.add(vertex)
                else:
                    self.last_command = STOP
                    return STOP, False, distance
            self.segment = vertex
            s, segment = self._project(px, py)
            segment = max(segment, vertex)
        total = self.cum[-1]
        if segment >= len(self.points) - 2:
            remaining = math.hypot(goal_x - px, goal_y - py)          # 最后一段：直接看到终点的距离
            if s >= total - 1e-6 and (goal_x - px) * (goal_x - x) + (goal_y - py) * (goal_y - y) < 0:
                remaining = 0.0                                       # 预测点已经越过终点
        else:
            remaining = total - s

        # 3. 速度大小（每段可以有自己的最高速度、减速度：爆发段 / 常规段，见 config.path_segments）
        seg = self._segment_params(segment)
        speed = min(seg['max_speed'], math.sqrt(2.0 * self._segment_params(len(self.waypoints) - 1)['decel']
                                                * max(remaining, 0.0)))
        current = chassis.speed() if chassis is not None else 0.0
        lookahead = self.lookahead_m + self.lookahead_s * current
        carrot_limit = None                          # 引导点最远到哪（前方第一个急弯的拐点）
        for vertex in range(segment + 1, len(self.points) - 1):
            ending = self._segment_params(vertex - 1)       # 以这个拐点为终点的那一段
            angle = self._turn_angle(vertex)
            if ending['end_speed'] is not None:
                # 这一段规定了末端速度（爆发段 = 0）：按"开到拐点刚好降到 end_speed"减速，引导点不越过拐点
                end_speed, sharp = float(ending['end_speed']), True
            elif angle < self.straight_angle:
                continue
            else:
                end_speed, sharp = self.corner_speed, angle >= self.sharp_angle
            # 急弯要真正走到拐点再转，减速从拐点算起；缓弯按引导点提前减速、走圆弧
            to_corner = max(0.0, self.cum[vertex] - s - (0.0 if sharp else lookahead))
            speed = min(speed, math.sqrt(end_speed ** 2 + 2.0 * float(ending['decel']) * to_corner))
            if sharp and carrot_limit is None:
                cx_, cy_ = self.points[vertex]
                # 末端速度 0 的段：引导点一直停在拐点，直到 update 开头判定"到拐点"切段为止
                # （原来离拐点 5 cm 内引导点就越过拐点，车提前往下一段方向走——16:19 横移快结束时车头往挡板方向拐）
                if ending['end_speed'] is not None or math.hypot(px - cx_, py - cy_) > self.corner_reach_m:
                    carrot_limit = self.cum[vertex]
        if remaining < 0.005:
            speed = 0.0
        # 要转的角度大：平移慢一点，保证走到这一段终点时差不多转完（边走边转比原地转快得多）
        seg_yaw = self.waypoints[min(segment, len(self.waypoints) - 1)][2]
        turn_left = abs(wrap_angle(seg_yaw - pyaw))
        wheel_limit = seg['wheel_limit'] if seg.get('wheel_limit') is not None else self.wheel_limit
        wz_cap = self.max_yaw_rate
        if chassis is not None and turn_left > math.radians(10.0) and speed > 0.0:
            to_waypoint = max(0.0, self.cum[min(segment + 1, len(self.cum) - 1)] - s)
            gain = chassis.gain_yaw * self.yaw_sync           # 边走边转实际转出指令的多少
            speed = min(speed, max(self.min_turn_speed, to_waypoint * self.max_yaw_rate * gain / turn_left))
            margin = 1.2                                      # 转向按"提前 1/6 转完"安排
            if wheel_limit and self.wheel_k > 0.0:
                # 平移和转向共用轮速：单轮 ≈ v·f + k·wz（f = 运动方向在车体坐标里的 |cos|+|sin|，斜着走更费轮速）。
                # 走完 to_waypoint 正好转完 turn_left 要 wz = v·turn_left /(gain·to_waypoint)，
                # 代入 v·f + k·wz·margin ≤ 轮速上限，得到平移最多能多快。
                # 2026-10-21：原来转向一律按 max_yaw_rate（1.0）给，垫子上实际只转 0.36 rad/s，去识别区 90 度要 4 秒多；
                # 直接把 max_yaw_rate 提高又会把轮速全给转向、平移被缩到 0（10-17 那次"平移只剩一半"）。
                wx, wy = self.points[min(segment + 1, len(self.points) - 1)]
                heading = math.atan2(wy - py, wx - px) - pyaw
                f = abs(math.cos(heading)) + abs(math.sin(heading))
                need = self.wheel_k * margin * turn_left / (gain * max(to_waypoint, 1e-3))
                speed = min(speed, max(self.min_turn_speed, wheel_limit / (f + need)))
            wz_cap = min(self.max_yaw_rate, max(self.min_turn_rate,
                                                margin * speed * turn_left / (gain * max(to_waypoint, 0.05))))

        # 4. 方向：指向引导点；贴线段：沿线 + 横向纠偏（见 cross_gain 说明）
        track = None
        if speed > 0.0 and seg.get('track') and segment < len(self.points) - 1:
            (ax, ay), (bx, by) = self.points[segment], self.points[segment + 1]
            length = math.hypot(bx - ax, by - ay)
            if length > 1e-6:
                ux, uy = (bx - ax) / length, (by - ay) / length
                offset = (py - ay) * ux - (px - ax) * uy                # 预测点在线左边为正
                correction = clamp(-self.cross_gain * offset, -self.cross_max, self.cross_max)
                track = ((speed * ux, speed * uy), (-uy * correction, ux * correction))
        if track is not None:
            vx_map, vy_map = track[0][0] + track[1][0], track[0][1] + track[1][1]
        elif speed > 0.0:
            if segment >= len(self.points) - 2 and remaining <= lookahead:
                carrot = (goal_x, goal_y)
            elif carrot_limit is not None:
                carrot = self._point_at(min(s + lookahead, carrot_limit))
            else:
                carrot = self._point_at(s + lookahead)
            cx, cy = carrot[0] - px, carrot[1] - py
            norm = math.hypot(cx, cy)
            if norm < 1e-6:
                cx, cy, norm = goal_x - x, goal_y - y, max(distance, 1e-6)
            vx_map, vy_map = speed * cx / norm, speed * cy / norm
        else:
            vx_map = vy_map = 0.0

        # 5. 朝向：当前段终点的朝向，用预测朝向控制
        target_yaw = self.waypoints[min(segment, len(self.waypoints) - 1)][2]
        yaw_error = wrap_angle(target_yaw - pyaw)
        wz = clamp(self._yaw_gain(chassis, False) * yaw_error, -wz_cap, wz_cap)
        if speed == 0.0:
            wz, _ = self.turn_in_place(pose, chassis, target_yaw)     # 只剩原地转
        elif abs(yaw_error) < math.radians(1.0):
            wz = 0.0                                # 1 度以内不修（位姿朝向 0.5 度一跳，修了只会来回抖）

        # 6. 换算到车体坐标系：用指令真正起作用时的朝向
        tau = chassis.tau if chassis is not None else 0.0
        yaw_effect = yaw + dyaw_dead + 0.5 * wz * tau
        if track is not None:
            ax_, ay_ = map_to_base(track[0][0], track[0][1], yaw_effect)
            kx_, ky_ = map_to_base(track[1][0], track[1][1], yaw_effect)
            command = limit_wheel_speed_keep(VelocityCommand(kx_, ky_, wz), VelocityCommand(ax_, ay_, 0.0),
                                             wheel_limit, self.wheel_k)
        else:
            vx, vy = map_to_base(vx_map, vy_map, yaw_effect)
            command = limit_wheel_speed(VelocityCommand(vx, vy, wz), wheel_limit, self.wheel_k)
        # 输出低通（约 0.07 秒）：位姿每周期的小跳动不直接变成速度方向的抖动
        if self.last_command is not None and speed > 0.0:
            a = self.output_smoothing
            command = VelocityCommand(a * self.last_command.vx + (1 - a) * command.vx,
                                      a * self.last_command.vy + (1 - a) * command.vy,
                                      a * self.last_command.wz + (1 - a) * command.wz)
        self.last_command = command
        return command, False, distance


class WallAligner(object):
    """贴墙精定位：用激光测到的墙距和墙的角度，把车停到示教时记录的位置和朝向。

    这是旧程序 move_to_position() 的改进版：
      - 旧程序：取扇区里最近几个点求平均，前后/左右固定速度（0.13 m/s）来回挪，容差 2 cm；不管朝向。
      - 新程序：直线拟合求垂直距离和墙的角度，比例控制同时修正前后、左右和朝向，
                接近目标时自动减速，容差可以收到 1 cm 以内。

    wall_refs：[{'side': 'back', 'distance': 1.282}, ...]，distance 是示教记录的激光到墙距离。
    yaw_side：用哪面墙校正朝向（一般用最长、最远的后墙）；yaw_ref 为示教时记录的偏角。
    """

    def __init__(self, wall_refs, yaw_side, yaw_ref, params):
        self.wall_refs = wall_refs
        self.yaw_side = yaw_side
        self.yaw_ref = yaw_ref
        self.p = params

    def update(self, wall_fits, chassis=None, boost=1.0):
        """wall_fits：{side: WallFit 或 None}；chassis：ChassisModel（可选）；
        boost：最小速度放大倍数（车"发了最小速度也不动"时由 AlignStep 逐步加大，见 AlignStep 说明）。

        返回 (VelocityCommand, errors, aligned)：
            errors：{'back': 距离误差, ..., 'yaw': 角度误差}，看不到的墙不在里面；
            aligned：所有误差都在容差内（且需要的墙都看到了）。
        有 chassis 时，速度按"预测停车点"的误差算（2026-10-17：不预测时冲过 4~5 cm 再倒回）；
        完成与否仍按当前实测误差判断。
        """
        p = self.p
        vx = vy = 0.0
        errors = {}
        all_seen = True
        # 只在车确实跑得比较快时才按预测收油门（冲过头只发生在这时）；低速蠕动时底盘常常不起步，模型会以为在动
        fast = chassis is not None and (chassis.speed() > 0.06 or abs(chassis.velocity[2]) > 0.2)
        sx, sy, syaw, _ = chassis.predict_stop() if fast else (0.0, 0.0, 0.0, 0.0)
        for ref in self.wall_refs:
            fit = wall_fits.get(ref['side'])
            if fit is None:
                all_seen = False
                continue
            # 误差为正 = 离墙太远，应朝墙移动
            error = fit.distance - ref['distance']
            errors[ref['side']] = error
            direction_x, direction_y = SIDE_DIRECTION[ref['side']]
            predicted = error - (sx * direction_x + sy * direction_y)   # 滑行会让离墙误差再减小这么多
            speed = speed_with_minimum(predicted, p['linear_gain'], p['max_linear_speed'],
                                       p['min_linear_speed'] * boost, p['distance_tolerance'])
            vx += speed * direction_x
            vy += speed * direction_y

        wz = 0.0
        yaw_fit = wall_fits.get(self.yaw_side)
        if yaw_fit is not None:
            yaw_error = wrap_angle(yaw_fit.yaw_offset - self.yaw_ref)
            errors['yaw'] = yaw_error
            # yaw_offset 为正表示车偏左（逆时针），要往右转，所以取负号
            # boost 同时放大朝向增益（2026-10-20）：垫子上原地只转出指令的 0.17，最后 1~2 度按 0.04~0.06 rad/s 慢慢蹭；
            # 一开始就用大增益在硬地面（原地 0.4）上会来回摆，所以只在"蹭不动"时（AlignStep 判 boost）才加大
            yaw_gain = p['yaw_gain']
            if p.get('yaw_bandwidth') and chassis is not None:
                # 按"实际转速 = yaw_bandwidth × 误差"给指令（见 PathFollower.yaw_bandwidth），精定位基本是原地转
                yaw_gain = float(p['yaw_bandwidth']) / max(chassis.gain_yaw_inplace, 0.1)
            wz = -speed_with_minimum(yaw_error + syaw, yaw_gain * boost, p['max_yaw_rate'],
                                     p['min_yaw_rate'] * boost, p['yaw_tolerance'])
        else:
            all_seen = False

        aligned = all_seen and all(
            abs(e) <= (p['yaw_tolerance'] if k == 'yaw' else p['distance_tolerance'])
            for k, e in errors.items())
        return VelocityCommand(vx, vy, wz), errors, aligned


def _box_distance(px, py, x0, x1, y0, y1):
    """点到矩形 [x0,x1]×[y0,y1] 的距离（在矩形里为 0），支持 numpy 广播。"""
    dx = np.maximum(np.maximum(x0 - px, px - x1), 0.0)
    dy = np.maximum(np.maximum(y0 - py, py - y1), 0.0)
    return np.hypot(dx, dy)


def obstacle_in_path(base_xs, base_ys, command, front_length, rear_length, half_width, margin, stop_distance,
                     self_depth=0.02):
    """行驶安全检查：车沿当前速度方向再走 stop_distance 以内，会不会撞进激光点，或擦着它过去（间隙 < margin）。

    base_xs, base_ys：机器人坐标系下的激光点（numpy 数组）。
    车身是前后不对称的矩形：中心往前 front_length、往后 rear_length、左右各 half_width（前面有枪管）。
    只检查平移方向；原地转动不检查。
    返回 True 表示前方有障碍，应停车。

    2026-10-20 改成按"车身矩形沿运动方向平移"精确算。原来用"四个角里最靠前的那个角"当整条前沿：
    斜着走时（通道横移 + 一点后退纠偏），车尾后面十几厘米、几乎和运动方向平行的后围栏会被当成前方障碍。
      1. 撞进去：再走 stop_distance 以内，激光点陷进车身的深度比现在多 5 mm 以上 → 挡
         （正前方的点：离车头不到 stop_distance - 5 mm 就挡，和原来差不多）。
         按"陷进去多深"而不是"碰没碰到"算：贴着墙起步、或边走边转时车角已经挨着墙，往远离墙的方向走不算撞
         （整局仿真：去识别区转身时左前角挨着左围栏，按"碰到"算会被拦住，只停平移、转向还在滑，反而蹭上围栏）；
      2. 擦过：这个点撞不上（在路线旁边），但走 stop_distance 以内会离车身不到 margin，
         而且比现在离所有障碍物最近的距离还近 5 mm 以上 → 挡（挡板端头这种从旁边擦过去的）。
    陷进车身轮廓超过 self_depth 的点先去掉：那是激光扫到的车自己的零件，不是障碍。
    2026-10-20 17:57 整局：激光每帧都有 2~4 个点在车尾轮廓里 6.6 cm 深（激光坐标 -165 度、0.18 米，以前的录包里也有）。
    去 1 号点第一条指令是右移带一点后退，这些点"越陷越深"被当成障碍，原地等满 8 秒超时，
    接着 1 号点精定位在识别区按错的墙对位，车撞上卡片墙。真的墙最多因为车身尺寸估计偏大"陷进去" 1 cm 左右（起点框后围栏 1.1 cm）。
    """
    speed = math.hypot(command.vx, command.vy)
    if speed < 1e-3 or len(base_xs) == 0:
        return False
    dir_x = command.vx / speed
    dir_y = command.vy / speed
    xs = np.asarray(base_xs, dtype=float)
    ys = np.asarray(base_ys, dtype=float)
    x0, x1, y0, y1 = -rear_length, front_length, -half_width, half_width

    def depth(px, py):
        """点陷进车身多深（米，在车身外面为负）。"""
        return np.minimum(np.minimum(px - x0, x1 - px), np.minimum(py - y0, y1 - py))

    real = depth(xs, ys) <= self_depth                 # 去掉车自己的零件
    xs, ys = xs[real], ys[real]
    if len(xs) == 0:
        return False
    depth_now = max(0.0, float(depth(xs, ys).max()))
    t_lo, t_hi = _box_entry(xs, ys, dir_x, dir_y, x0, x1, y0, y1)
    a = np.maximum(t_lo, 0.0)
    b = np.minimum(t_hi, stop_distance)
    inside = a <= b                                    # 走 stop_distance 以内会进到车身里的点
    if inside.any():
        frac = np.linspace(0.0, 1.0, 5)
        ts = a[inside][:, None] + (b[inside] - a[inside])[:, None] * frac
        deepest = float(depth(xs[inside][:, None] - ts * dir_x, ys[inside][:, None] - ts * dir_y).max())
        if deepest > depth_now + 0.005:
            return True
    now = _box_distance(xs, ys, x0, x1, y0, y1)
    side = (t_lo > t_hi) & (now < margin + stop_distance)          # 撞不上、但离得不远的点
    if not side.any():
        return False
    steps = np.linspace(0.0, stop_distance, 9)
    dist = _box_distance(xs[side][:, None] - steps * dir_x, ys[side][:, None] - steps * dir_y, x0, x1, y0, y1)
    closest_now = float(now.min())
    return bool(((dist.min(axis=1) < margin) & (dist.min(axis=1) < closest_now - 0.005)).any())


def _box_entry(px, py, dir_x, dir_y, x0, x1, y0, y1):
    """点 (px, py) 相对车身以 -t·dir 移动（车沿 dir 走 t 米）时，进入矩形 [x0,x1]×[y0,y1] 和离开它的 t（slab 法）。
    返回 (t_enter, t_exit) 两个数组；t_enter > t_exit 表示永远碰不到。"""
    inf = np.inf
    t_lo = np.full(px.shape, -inf)
    t_hi = np.full(px.shape, inf)
    for p, d, lo, hi in ((px, dir_x, x0, x1), (py, dir_y, y0, y1)):
        if abs(d) < 1e-9:
            outside = (p < lo) | (p > hi)
            t_lo = np.where(outside, inf, t_lo)
            t_hi = np.where(outside, -inf, t_hi)
        else:
            a = (p - hi) / d
            b = (p - lo) / d
            t_lo = np.maximum(t_lo, np.minimum(a, b))
            t_hi = np.minimum(t_hi, np.maximum(a, b))
    return t_lo, t_hi
