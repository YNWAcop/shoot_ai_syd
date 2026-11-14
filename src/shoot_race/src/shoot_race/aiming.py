# -*- coding: utf-8 -*-
"""瞄准与开火判断的纯计算部分（不依赖 ROS，可单独测试）。

三个靶三种打法（车都不转向，只做横向平移：这台底盘原地转向死区很大，平移才准）：
  RingAimer        环形靶（静止）：停稳看 → 按偏差横向平移过去 → 停稳再看，偏差够小就开火。
  RotatingPlanner  旋转靶（转轮，2025 场地实测 43 秒转一圈）：
                   用转轮上全部 5 个码拟合出转轮的圆心、半径、转速，算出目标码下一次经过"枪口高度"
                   （圆和枪口高度那条水平线的左右两个交点）的时刻；左右交点哪个先到打哪个，
                   原地扭车头（2026-10-23 起，YawTwister）或横向平移把交点对准枪口，到时刻开火。
  MovingPredictor  移动靶（滑块左右往返）：车不动，预测目标到达枪口的时刻开火；
                   刚换向、快到行程端点时不开火（换向时速度突变，预测会错）。

坐标统一用像素（u 向右，v 向下）：
  环形靶  find_object 给的靶心像素横坐标；
  AR 码   ar_track_alvar 的位置 (x, y, z) 用相机内参换算：u = fx·x/z + cx，v = fy·y/z + cy。
          不直接用"米"：车上 ar_track_alvar 的 marker_size 参数和码的实际尺寸不一致，
          米制坐标整体放大约 2.9 倍（深度读 4.3 m，实际约 1.5 m），但 x/z、y/z 不受影响。
平移和像素的换算：车向左平移 Δy 米，距离 D 处的目标在画面里向右移 fx·Δy/D 像素。

旧程序对动靶是"追着打"：车一直转着追 AR 码，开枪前不刹车，也没有提前量。
车在动、画面在拖影、弹丸到达时目标已经走开，这是动靶打不准的主要原因之一。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import collections
import math

import numpy as np

from .geometry import wrap_angle
from .motion import hold_heading, speed_with_minimum


# ============================================================
# 横向平移（三个靶共用）
# ============================================================
class LateralShifter(object):
    """把车横向平移到指定的横向位置，用激光测的侧墙距离做反馈。

    为什么用激光、不用里程计（2026-10-16 平移测试 step_test.py）：
      低速平移时底盘有明显起步延迟（0.05 m/s 要 0.4 s 才动，0.03 m/s 要 0.7 s，0.02 m/s 不动），
      而且麦轮侧滑，里程计把横向位移高估约 3 倍；底盘驱动的里程计位置也只有 1 cm 分辨率。
      激光测侧墙距离是真实位移，和贴墙精定位同一个原理。
    所以最小速度取 0.05 m/s，卡住判定放宽到 2 秒，让底盘有时间"攒劲"起步。

    横向位置：米，左为正，原点任意（只用差值），由 steps.LateralTracker 提供。
    """

    def __init__(self, params):
        self.gain = params['gain']                  # 剩余距离（米）→ 速度（m/s）的比例
        self.max_speed = params['max_speed']
        self.min_speed = params['min_speed']        # 底盘能动起来的最小平移速度
        self.tolerance = params['tolerance_m']      # 到位容差（米）
        self.stall_s = params['stall_s']            # 剩余距离多久不减小就算"卡住"
        # 松油门后还会滑：实测横向速度 × coast_s。2026-10-17 21:24 录包：0.05 m/s 横移，指令 15 mm 实际走了 26 mm
        # （到位才收油门，底盘约 0.3 秒死区里继续滑）。只在车确实在动（≥ 2 cm/s）时算，车停着时照常发指令直到起步
        self.coast_s = params.get('coast_s', 0.0)
        # 小步平移用慢速（2026-10-22）：0.05 m/s 松油门后还滑 1~2 cm，要走 1~2 cm 的时候经常走过头、来回修正；
        # 0.03 m/s 起步慢（约 0.7 秒）但滑得少（2026-10-16 实测，菜单 22 移动靶细调一直这样做）
        self.fine_below = params.get('fine_below_m', 0.0)
        self.fine_speed = params.get('fine_speed', self.min_speed)
        self.fine = None                            # 本次平移是不是小步（第一次 update 时按剩余距离定；走过头后也改成小步）
        self.first_sign = 0.0                       # 本次平移开始时剩余距离的正负（判断走过头）
        self.target = None
        self.best = None                            # 本次平移见过的最小 |剩余距离|
        self.best_time = None
        self.history = []                           # 最近 0.3 秒的 (时刻, 横向位置)，算实测速度

    def start(self, target):
        """开始一次平移：目标横向位置 target（米，左为正）。"""
        self.target = target
        self.best = None
        self.best_time = None
        self.history = []
        self.fine = None
        self.first_sign = 0.0

    def coast(self, now, lateral):
        """按最近 0.3 秒的激光横向位置估计速度，返回松油门后还会滑多远（米，左为正）。"""
        self.history.append((now, lateral))
        while len(self.history) > 2 and now - self.history[1][0] >= 0.3:
            self.history.pop(0)
        t0, l0 = self.history[0]
        if self.coast_s <= 0 or now - t0 < 0.15:
            return 0.0
        velocity = (lateral - l0) / (now - t0)
        return velocity * self.coast_s if abs(velocity) >= 0.02 else 0.0

    def update(self, now, lateral):
        """返回 (横向速度 vy, 状态, 剩余距离)。状态：moving / done / stalled / no_wall。"""
        if lateral is None:
            return 0.0, 'no_wall', None
        remaining = self.target - lateral
        if self.fine is None:
            self.fine = abs(remaining) < self.fine_below
            self.first_sign = math.copysign(1.0, remaining) if remaining != 0.0 else 0.0
        elif self.fine_below > 0 and remaining * self.first_sign < -self.tolerance:
            self.fine = True                       # 走过头了：往回修正用慢速，免得来回晃
        coast = self.coast(now, lateral)
        if abs(remaining) <= self.tolerance and coast == 0.0:
            return 0.0, 'done', remaining
        # 卡住检测：剩余距离比之前的最小值再小 0.5 mm 才算有进展
        if self.best is None or abs(remaining) < self.best - 0.0005:
            self.best = abs(remaining)
            self.best_time = now
        elif now - self.best_time > self.stall_s:
            return 0.0, 'stalled', remaining
        predicted = remaining - coast                      # 松油门后还差多少
        if abs(predicted) <= self.tolerance or predicted * remaining < 0:
            return 0.0, 'moving', remaining                # 已经够了：松油门，靠滑行到位（停住后再判 done）
        if self.fine:
            speed = speed_with_minimum(predicted, self.gain, self.fine_speed, self.fine_speed, 0.0)
        else:
            speed = speed_with_minimum(predicted, self.gain, self.max_speed, self.min_speed, 0.0)
        return speed, 'moving', remaining


class YawTwister(object):
    """原地把车头扭到指定朝向（2026-10-23，旋转靶左右交点用），用激光侧墙偏角做反馈（同菜单 22 粗调 aim_test.turn_by）。

    为什么扭车头而不是平移：交点离弹着点约一个转轮半径（1.5 米处约 30 像素 ≈ 9 cm），平移最多 5 cm 不出框；
    扭车头 3~4 度就够，车头两角（离中心 0.23 m）横向只多出约 1.5 cm。车和枪、相机是一体的，扭完以后
    弹着点在画面里的像素不变，所以只要让交点在画面里落到弹着点上。
    垫子上原地转有死区、最小一下约 3~4 度，扭不准：一次扭完（走过头不往回修），停稳后重新看画面，剩下的再修。

    朝向：弧度，左转为正（steps.LateralTracker.yaw，侧墙拟合的偏角）。
    """

    def __init__(self, params):
        self.gain = params['gain']                          # 剩余角度（弧度）→ 角速度指令（rad/s）
        self.max_rate = params['max_rate']
        self.min_rate = params['min_rate']                  # 垫子上原地转能转起来的最小指令
        self.tolerance = math.radians(params['tolerance_deg'])
        self.coast_s = params.get('coast_s', 0.0)           # 松开后还会转：实测角速度 × coast_s（每扭一次按实测更新）
        self.coast_range = (0.2, 1.5)
        self.release = None                                 # 松开那一刻的 (车头, 实测角速度)，扭完后更新 coast_s 用
        self.stall_s = params.get('stall_s', 2.0)
        self.timeout_s = params.get('timeout_s', 4.0)
        self.target = None
        self.first_sign = 0.0
        self.started = None
        self.best = None
        self.best_time = None
        self.history = []

    def start(self, target, now=None):
        self.target = target
        self.first_sign = 0.0
        self.started = now
        self.best = None
        self.best_time = None
        self.history = []
        self.release = None

    def _learn_coast(self, yaw):
        """松开后实际又转了多少 / 松开时的角速度 → 更新 coast_s（和上次的平均，限 0.2~1.5 秒）。"""
        if self.release is None:
            return
        yaw_release, rate_release = self.release
        self.release = None
        if abs(rate_release) < math.radians(1.0):
            return
        measured = wrap_angle(yaw - yaw_release) / rate_release
        low, high = self.coast_range
        self.coast_s = min(high, max(low, 0.5 * self.coast_s + 0.5 * measured))

    def _rate(self, now, yaw):
        """最近 0.4 秒侧墙偏角的变化率（rad/s）；时间太短返回 0。"""
        self.history.append((now, yaw))
        while len(self.history) > 2 and now - self.history[1][0] >= 0.4:
            self.history.pop(0)
        t0, y0 = self.history[0]
        if now - t0 < 0.2:
            return 0.0
        return wrap_angle(yaw - y0) / (now - t0)

    def update(self, now, yaw):
        """返回 (角速度指令 wz, 状态, 剩余角度)。状态：moving / done / stalled / timeout / no_wall。"""
        if yaw is None:
            return 0.0, 'no_wall', None
        if self.started is None:
            self.started = now
        remaining = wrap_angle(self.target - yaw)
        if self.first_sign == 0.0:
            self.first_sign = math.copysign(1.0, remaining) if remaining != 0.0 else 1.0
        rate = self._rate(now, yaw)
        moving = abs(rate) >= math.radians(0.5)
        coast = rate * self.coast_s if moving else 0.0
        overshot = remaining * self.first_sign < 0
        if (abs(remaining) <= self.tolerance or overshot) and not moving:
            self._learn_coast(yaw)
            return 0.0, 'done', remaining                  # 到了，或者走过头了：停稳后交给画面重新看（不来回修）
        if now - self.started > self.timeout_s:
            return 0.0, 'timeout', remaining
        if self.best is None or abs(remaining) < self.best - math.radians(0.1):
            self.best = abs(remaining)
            self.best_time = now
        elif now - self.best_time > self.stall_s and not moving:
            return 0.0, 'stalled', remaining
        predicted = remaining - coast
        if overshot or abs(predicted) <= self.tolerance or predicted * remaining < 0:
            if self.release is None and moving:
                self.release = (yaw, rate)                 # 记下松开时的车头和角速度
            return 0.0, 'moving', remaining                # 松开，靠惯性转到位（停稳后再判 done）
        wz = speed_with_minimum(predicted, self.gain, self.max_rate, self.min_rate, 0.0)
        return wz, 'moving', remaining


class PixelAligner(object):
    """弹道测试用（scripts/aim_test.py）：反复"看码 → 横移 → 停稳再看"，直到目标码的横坐标对上弹着点。

    为什么要按实测更新换算系数：2026-10-18 22:19 整局实测，在垫子上横移 50 mm，转轮在画面里只动了 8.5 像素
    （理论 fx×0.05/1.5 ≈ 22 像素），因为横移时车头会跟着偏。所以第一次按理论值走，之后用
    "这次画面里动了多少像素 / 激光测的横向实际走了多少" 当换算系数（限制在理论值的 0.25~1.5 倍）。
    横向位置：米，左为正（车向左 → 画面里目标向右，u 增大）。
    """

    def __init__(self, aim_u, px_per_m, tolerance_px, max_total_m, max_iterations, max_flips=3):
        self.aim_u = aim_u
        self.theory = px_per_m
        self.gain = px_per_m                 # 当前用的换算系数（像素 / 米）
        self.tolerance = tolerance_px
        self.max_total = max_total_m
        self.max_iterations = int(max_iterations)
        self.max_flips = int(max_flips)      # 误差正负来回翻这么多次就停：底盘最小一步（约 1.5 cm ≈ 7 像素）比剩下的误差还大
        self.origin = None
        self.previous = None                 # 上一次规划时的 (目标码 u, 横向位置, 误差)
        self.iterations = 0
        self.flips = 0

    def plan(self, u_target, lateral):
        """返回 (状态, 目标横向位置, 说明)。状态：done（对准了）/ shift（去目标横向位置）/ give_up（到此为止）。"""
        if self.origin is None:
            self.origin = lateral
        error = self.aim_u - u_target         # 目标码要在画面里移动多少像素（右为正）
        if self.previous is not None:
            moved = lateral - self.previous[1]
            if abs(moved) > 0.005:           # 走了 5 mm 以上才更新，太小的话除出来全是噪声
                measured = (u_target - self.previous[0]) / moved
                low, high = 0.25 * self.theory, 1.5 * self.theory
                self.gain = min(high, max(low, measured))
            if error * self.previous[2] < 0:
                self.flips += 1
        info = {'err_px': error, 'gain_px_per_cm': self.gain / 100.0, 'iteration': self.iterations}
        if abs(error) <= self.tolerance:
            return 'done', lateral, info
        if self.iterations >= self.max_iterations or self.flips >= self.max_flips:
            info['reason'] = 'iterations' if self.iterations >= self.max_iterations else 'flips'
            return 'give_up', lateral, info
        # 来回翻过符号说明上一步走过头了（底盘松油门后还会滑），之后每步只走算出来的 7 成
        target = lateral + error / self.gain * (0.7 if self.flips else 1.0)
        low, high = self.origin - self.max_total, self.origin + self.max_total
        if (target > high and lateral >= high - 0.002) or (target < low and lateral <= low + 0.002):
            info['reason'] = 'at_limit'      # 已经顶到平移上限还要往外走：到此为止（不往回拉）
            return 'give_up', lateral, info
        clamped = max(low, min(high, target))
        info['shift_mm'] = (clamped - lateral) * 1000.0
        self.previous = (u_target, lateral, error)
        self.iterations += 1
        return 'shift', clamped, info


def pixels_to_shift(pixels, camera_fx, distance, image_gain=1.0):
    """要让目标在画面里向右移 pixels 像素，车要向左平移多少米（负数 = 向右）。

    image_gain：车平移时识别出的像素实际变化 / 理论变化（fx·Δy/D）。AR 码按位姿换算，取 1；
    环形靶 find_object 实测约 0.55（见 RingAimer）。
    """
    return pixels / (camera_fx * image_gain) * distance


# ============================================================
# 环形靶
# ============================================================
class RingAimer(object):
    """环形靶对准："停稳看 → 横向平移过去 → 停稳再看"，平移前后看到的结果按可信度合起来估计。

    为什么平移、不转向（2026-10-16 场地实测，原地转速测试 spin_test.py）：
      这台麦轮底盘原地转向死区很大：0.3 rad/s 以下的转向指令车完全不动，
      0.5~1.0 rad/s 也只转出指令的 25%~35%，做不了零点几度的精细转向。
      平移则很准：激光侧墙测的平移量和卷尺量的一致（42.4 mm vs 43 mm）。

    2026-10-16 晚 13 个环形靶录包（shoot_ring_*.bag）分析出的三件事，决定了现在的做法：
      1. find_object 只有约 2.6 帧/秒（0.38 秒一帧），不是 10 Hz；车完全静止时单帧靶心像素
         抖动标准差约 3 像素（5%~95% 分位 ±4.9）。旧做法"3 帧中位数、容差 2.5 像素"等于在抽签：
         录到 -2.8 → +2.8 → -6.0 像素来回修正直到 8 秒超时。
      2. 车横向平移时，find_object 的像素只变化理论值 fx·Δy/D 的约 0.55 倍（10 次平移回归，
         残差 2.3 像素；可能是模板里带了远处背景）。旁证：三次试射反推弹着点，按理论值换算
         得 370 / 358.5 / 364.8（差 11 像素），按 0.55 倍换算得 365.8 / 363.4 / 366.4（差 3 像素）。
         所以像素 → 平移量的换算要除以 image_gain，否则每次只修正一半多一点。
      3. 每次平移车身朝向会偏 0.1~0.4 度（激光侧墙偏角，多数偏同一方向），画面跟着偏 1~4 像素。
         所以平移前看到的结果换算到新位置后不完全可信，不能和新看到的帧同等对待。

    做法：
      - 车停稳、再等 vision_latency_s 之后收到的帧才用。同一位置的帧叫"一轮"，取中位数。
      - 估计当前位置的靶心像素：本轮中位数，加上之前各轮"换算到当前位置"的中位数
        （u + g·(lat_now - lat)，g = image_gain·fx/D 像素/米），按可信度加权平均：
          每轮的方差 = 识别噪声 (1.25·frame_noise_px)²/帧数
                     + 换算误差 (model_error × 换算的像素量)² + 朝向误差 shift_yaw_px² × 之后平移了几次。
        平移量小、换算准时旧帧能帮上忙（少看几帧）；平移量大、模型不准时自动以新帧为主。
      - 偏差 e = 估计 - aim_x_px：
          |e| ≤ tolerance_px                         → 开火；
          |e| ≤ look_more_px 且这一轮看得还不多       → 原地多看几帧（0.4 秒一帧，比平移 + 停稳约 2.5 秒便宜）；
          否则                                       → 平移 -e/g 米，平移完回到 look。
      - 新一轮和旧估计差超过 reset_px → 说明换算倍数不对或靶被挪了，丢掉旧的几轮。
      - 累计平移量限制在 max_shift_m 以内（防止识别错误时车跑出任务框）；已经顶到上限 → 直接开火（尽力而为，
        不干等到超时）。
    蒙特卡洛（tests/ring_monte_carlo.py，用录包的噪声、帧率、平移起步延迟）：旧做法用时中位 4.4 s、超时 18~36%；
    这里用时中位约 4 s、超时 1~3%，弹着偏差相当或更小。
    """

    def __init__(self, params, shift_params, camera_fx):
        self.aim_x_px = params['aim_x_px']                    # 弹着点在图像中的横坐标（标定得到）
        self.tolerance_px = params['tolerance_px']            # 允许的像素偏差
        self.look_more_px = params.get('look_more_px', 2.0 * self.tolerance_px)
        self.camera_fx = camera_fx                            # 相机焦距（像素）
        self.target_distance = params['target_distance_m']    # 车到环形靶的距离（米），用于像素 → 平移量换算
        self.image_gain = params.get('image_gain', 1.0)       # 平移时像素实际变化 / 理论变化
        self.px_per_m = self.image_gain * camera_fx / self.target_distance
        self.vision_latency_s = params['vision_latency_s']    # 检测结果相对拍摄时刻的延迟
        self.confirm_frames = int(params['confirm_frames'])   # 第一次看至少几帧
        self.recheck_frames = int(params.get('recheck_frames', self.confirm_frames))   # 平移后再看至少几帧
        self.max_look_frames = int(params.get('max_look_frames', 6))   # 同一位置最多看几帧（再不行就平移）
        # 估计误差要小到这个程度才开火（像素，None = 不卡）。2026-10-19 弹道测试：估计偏差 2.2 像素（在容差内）就开火了，
        # 5 枪实测靶心平均偏 4.3 像素、弹孔偏右 1.8 cm（9 环）——6 帧的估计误差约 ±1.8 像素，比容差的一半还大
        self.max_sigma_px = params.get('max_sigma_px')
        self.frame_noise_px = params.get('frame_noise_px', 3.0)        # 单帧识别噪声（标准差，像素）
        self.model_error = params.get('model_error', 0.3)              # 平移换算的相对误差
        self.shift_yaw_px = params.get('shift_yaw_px', 2.5)            # 每次平移朝向偏差带来的像素误差
        self.reset_px = params.get('reset_px', 8.0)
        self.max_shift = params['max_shift_m']                # 累计平移量上限（米）
        # 走过头（这次看到的偏差和平移前反号）之后，之后每次只走算出来的这么多倍，免得来回修正（1.0 = 不打折）。
        # 2026-10-22 找同心圆后识别噪声只剩 0.3 像素，剩下的误差主要是底盘：小步平移会滑过头、车头会偏 0.1~0.4 度
        self.overshoot_scale = params.get('overshoot_scale', 1.0)
        self.last_shift_error = None  # 上一次决定平移时的偏差（像素）
        self.overshoots = 0           # 走过头几次了
        self.shifter = LateralShifter(shift_params)
        self.shift_tolerance = self.shifter.tolerance
        self.state = 'look'
        self.look_since = None        # 本次"看"从什么时刻开始（车停稳的时刻）
        self.look_frames = []         # 本轮（当前位置）看到的帧 [(收到时刻, 像素, 当时横向位置或 None)]
        self.looks = []               # 之前各轮：[(中位数像素, 帧数, 横向位置, 这一轮结束时已平移次数)]
        self.reset_checked = False    # 本轮是否已经和旧估计比较过
        self.last_frame_t = None      # 已经收进来的最新一帧的时刻（避免重复收）
        self.shifts = 0               # 已经修正过几次
        self.total_shift = 0.0        # 累计要求的平移量（米，左为正）
        self.shift_target = 0.0       # 本次要平移的距离（米，左为正）

    def _noise_var(self, frames):
        """n 帧中位数的方差（中位数比平均数噪声大约 1.25 倍）。"""
        return (1.25 * self.frame_noise_px) ** 2 / max(1, frames)

    def _old_estimates(self, lat_now):
        """之前各轮换算到当前位置：[(像素, 方差)]。不知道当前横向位置时没法换算，不用。"""
        out = []
        if lat_now is None:
            return out
        for median, frames, lat, shifts_then in self.looks:
            if lat is None:
                continue
            moved_px = self.px_per_m * (lat_now - lat)
            var = (self._noise_var(frames) + (self.model_error * moved_px) ** 2
                   + self.shift_yaw_px ** 2 * (self.shifts - shifts_then))
            out.append((median + moved_px, var))
        return out

    def update(self, now, lateral, observations, robot_still):
        """一个控制周期的判断。

        lateral：车的横向位置（米，左为正，原点任意，只用差值），由激光侧墙距离算出；拿不到时为 None。
        observations：[(收到时刻, 靶心像素x), ...]，按时间排序。
        robot_still：车是否已经停稳。
        返回 (vy, ready_to_fire, info)：vy 是横向速度（m/s，左为正）。
        """
        if self.state == 'shift':
            vy, status, remaining = self.shifter.update(now, lateral)
            if status == 'no_wall':
                return 0.0, False, {'state': 'shift', 'reason': 'no_wall'}
            info = {'state': 'shift', 'left_mm': remaining * 1000}
            if status == 'moving':
                return vy, False, info
            # 到位（done）或卡住（stalled）都回到 look 重新看
            self.state = 'look'
            self.look_since = None
            info.update({'state': 'look', 'reason': 'shift_' + status})
            return 0.0, False, info

        # ---- look ----
        if not robot_still:
            self.look_since = None
            return 0.0, False, {'state': 'look', 'reason': 'settling'}
        if self.look_since is None:
            self.look_since = now
        for (t, x) in observations:
            if t >= self.look_since + self.vision_latency_s and (self.last_frame_t is None or t > self.last_frame_t):
                self.look_frames.append((t, x, lateral))
                self.last_frame_t = t
        need = self.recheck_frames if self.looks else self.confirm_frames
        if len(self.look_frames) < need:
            return 0.0, False, {'state': 'look', 'reason': 'waiting_frames', 'fresh': len(self.look_frames)}

        info = {'state': 'look', 'shifts': self.shifts}
        current = float(np.median([u for (_, u, _) in self.look_frames]))
        estimates = [(current, self._noise_var(len(self.look_frames)))]
        old = self._old_estimates(lateral)
        if old and not self.reset_checked:
            # 平移后第一次够帧：新一轮和旧估计对不上 → 旧的几轮作废
            self.reset_checked = True
            weights = [1.0 / v for (_, v) in old]
            old_mean = sum(w * u for w, (u, _) in zip(weights, old)) / sum(weights)
            if abs(current - old_mean) > self.reset_px:
                info['reset_px'] = current - old_mean
                self.looks = []
                old = []
        estimates += old
        weights = [1.0 / v for (_, v) in estimates]
        estimate = sum(w * u for w, (u, _) in zip(weights, estimates)) / sum(weights)
        error_px = estimate - self.aim_x_px
        info.update({'err_px': error_px, 'frames': len(self.look_frames), 'looks': len(estimates),
                     'sigma_px': (1.0 / sum(weights)) ** 0.5})
        sigma = info['sigma_px']
        enough = len(self.look_frames) >= self.max_look_frames
        if abs(error_px) <= self.tolerance_px and (self.max_sigma_px is None or sigma <= self.max_sigma_px or enough):
            return 0.0, True, info
        # 只差一点：多看几帧再定，免得被噪声带着来回平移。卡估计误差时，明显在容差外（差出 1.5 倍估计误差）就不再多看
        clearly_out = self.max_sigma_px is not None and abs(error_px) - self.tolerance_px > 1.5 * sigma
        if abs(error_px) <= self.look_more_px and not enough and not clearly_out:
            info['reason'] = 'look_more'
            return 0.0, False, info
        if lateral is None:
            info['reason'] = 'no_wall'
            return 0.0, False, info
        # 靶心在瞄准点右边（偏差为正）→ 要让靶心往左移 → 车要往右移（横向速度为负）
        if self.last_shift_error is not None and error_px * self.last_shift_error < 0:
            self.overshoots += 1
        self.last_shift_error = error_px
        scale = self.overshoot_scale if self.overshoots else 1.0
        wanted = pixels_to_shift(-error_px, self.camera_fx, self.target_distance, self.image_gain) * scale
        if self.overshoots:
            info['overshoots'] = self.overshoots
        # 累计平移量不超过 max_shift_m
        allowed = max(-self.max_shift - self.total_shift, min(self.max_shift - self.total_shift, wanted))
        if abs(allowed) < self.shift_tolerance:
            info['reason'] = 'max_shift_reached'  # 已经移到上限，再等也没用：照当前位置开火
            return 0.0, True, info
        # 这一轮结束，存起来（中位数、帧数、横向位置）
        lats = [lat for (_, _, lat) in self.look_frames if lat is not None]
        self.looks.append((current, len(self.look_frames), float(np.median(lats)) if lats else None, self.shifts))
        self.look_frames = []
        self.reset_checked = False
        self.shift_target = allowed
        self.total_shift += allowed
        self.shifter.start(lateral + allowed)
        self.state = 'shift'
        self.shifts += 1
        info['shift_mm'] = allowed * 1000
        return 0.0, False, info


# ============================================================
# AR 码数据的预处理
# ============================================================
def neighbor_median(values, half_width):
    """每个值前后各 half_width 个邻居（不含自己）的中位数；两端邻居少一些，不补值。

    不含自己、两端不补值：最新一帧是坏点时，它和前面几帧的中位数一比就露馅。
    （如果像普通滑动中位数那样用最后一个值补齐两端，最新的坏点会"自己证明自己"，正好漏掉最要紧的那一帧。）
    """
    values = np.asarray(values, dtype=float)
    n = len(values)
    pad = np.full(half_width, np.nan)
    padded = np.concatenate([pad, values, pad])
    columns = [padded[i:i + n] for i in range(2 * half_width + 1) if i != half_width]
    return np.nanmedian(np.array(columns), axis=0)


def drop_misreads(samples, outlier_px, half_width=3):
    """去掉 AR 误识别的样本：横坐标和前后邻居的中位数相差超过 outlier_px 像素。

    实测（2026-10-16 录包）：AR 码偶尔被认成旁边的码（码号对，位置却是另一个码的），
    横坐标会突然跳 27~55 像素（相邻码的间距）。单独一两帧，和邻居的中位数一比就能识别出来。
    samples：[(t, u, v), ...]；返回同格式的列表。
    """
    if len(samples) < 3:
        return list(samples)
    u = np.array([s[1] for s in samples])
    keep = np.abs(u - neighbor_median(u, half_width)) <= outlier_px
    return [s for s, k in zip(samples, keep) if k]


def fuse_slider_codes(samples_by_id, target, min_pairs=3, max_dev_px=3.0, prior_px=None):
    """移动靶 6/7/8 三个码装在同一个滑块上、一起动：用另外两个码补上目标码没认出来的帧（2026-10-25）。

    为什么：10-25 傍晚几局目标码时有时无（每秒只认出 8~16 次，断档 0.3~7 秒，多在弹着点附近），
    预测要"最近 0.3 秒至少 4 帧"，码经过弹着点那一下正好断档就不开火，等到 16 秒超时乱打一枪。
    同一时刻另外两个码多半认得出（三个码合起来断档 ≤ 0.34 秒），换算到目标码的位置就能接着预测。

    samples_by_id：{码号: [(图像时间戳, u, v), ...]}；target：目标码号。
    每个旁码相对目标码的偏移用"同一帧两个码都认出来"的帧实测：取中位数，偏离中位数 > max_dev_px 的
    （码认串了）不算；这样的帧不到 min_pairs 帧时这个旁码先不用。prior_px（{码号: 相对位置像素}）给了的话，
    帧数不够时改用先验间距（实测相邻两码约 26~27 像素，见 shooting.yaml）。
    同一帧目标码自己认出来了就用目标码自己的，没认出来时几个旁码换算出来的取平均。
    返回 [(t, u, v), ...]（目标码的位置），按时间排序，每帧一个。
    """
    def key(t):
        return int(round(t * 10000.0))

    own = samples_by_id.get(target) or []
    own_by_key = dict((key(s[0]), s) for s in own)
    borrowed = {}                     # 目标码没认出来的帧：{帧: [旁码换算出来的 (t, u, v), ...]}，几个旁码取平均
    for other in sorted(samples_by_id):
        samples = samples_by_id[other]
        if other == target or not samples:
            continue
        du, dv = [], []
        for s in samples:
            mine = own_by_key.get(key(s[0]))
            if mine is not None:
                du.append(s[1] - mine[1])
                dv.append(s[2] - mine[2])
        offset = None
        if len(du) >= min_pairs:
            du, dv = np.array(du), np.array(dv)
            keep = np.abs(du - np.median(du)) <= max_dev_px
            if keep.sum() >= min_pairs:
                offset = (float(np.median(du[keep])), float(np.median(dv[keep])))
        if offset is None and prior_px and other in prior_px and target in prior_px:
            offset = (float(prior_px[other]) - float(prior_px[target]), 0.0)
        if offset is None:
            continue
        for s in samples:
            k = key(s[0])
            if k not in own_by_key:
                borrowed.setdefault(k, []).append((s[0], s[1] - offset[0], s[2] - offset[1]))
    fused = dict(own_by_key)
    for k, items in borrowed.items():
        fused[k] = (items[0][0], sum(x[1] for x in items) / len(items), sum(x[2] for x in items) / len(items))
    return [fused[k] for k in sorted(fused)]


def line_fit(t, x):
    """x(t) = x0 + slope * t 的最小二乘拟合，返回 (slope, x0)。"""
    slope, x0 = np.polyfit(t, x, 1)
    return float(slope), float(x0)


# ============================================================
# 移动靶
# ============================================================
class MovingPredictor(object):
    """移动靶：车不动，预测目标到达枪口的时刻开火。

    数据（2026-10-16 录包 moving2.bag，2025 场地）：滑块匀速往返，速度约 18 像素/秒（约 4 cm/s），
    单程约 6.8 秒、行程约 121 像素（约 27 cm）；到端点立即换向，换向过程（速度低于一半）约 0.5 秒。

    每个控制周期：
      1. 去掉误识别（drop_misreads），取最近 fit_window_s 秒拟合直线 u(t)，得到当前位置和速度；
      2. 静止目标（速度 < static_speed_px_s 持续 run_window_s）：在容差内就开火（手扶着靶测试时就是这样）；
      3. 运动目标，以下任一条不开火：
           速度 > max_speed_px_s（数据有问题，比如漏掉的误识别）；
           速度 < min_speed_px_s（正在换向）；
           最近 run_window_s 前后两半的速度方向不一致（刚换过向）；
           前方的行程端点已经知道、且弹丸到达时目标离端点不到 end_margin_px（马上要换向）；
      4. 剩余时间 = (枪口 - 当前位置) / 速度。剩余时间 - latency_s ≤ 半个控制周期，
         且弹丸到达时刻的预测位置离枪口 ≤ hit_tolerance_px → 开火。
         （不是"一进容差就开火"：那样总是在容差边缘开火，系统性地早一点。）

    行程端点：历史里位置的最小值/最大值，如果不在历史的开头或结尾（说明目标到过那里又回来了），就是已知端点。

    latency_s（待实测标定）= 从程序决定开火到弹丸到达靶面的总时间：
        串口指令 + 发射机构动作 + 弹丸飞行。图像识别的延迟已经通过"用图像时间戳外推到现在"扣除了。
    """

    def __init__(self, params, control_period_s):
        self.aim_u = params['aim_u_px']               # 弹着点像素坐标（标定得到）
        self.aim_v = params['aim_v_px']
        self.use_v = bool(params['use_v'])            # 移动靶只水平运动，一般不卡高度
        self.tolerance_v = params['tolerance_v_px']
        self.hit_tolerance = params['hit_tolerance_px']
        self.latency_s = params['latency_s']
        self.fit_window_s = params['fit_window_s']
        self.run_window_s = params['run_window_s']
        self.min_samples = int(params['min_samples'])
        self.max_sample_age_s = params['max_sample_age_s']
        self.outlier_px = params['outlier_px']
        self.min_speed = params['min_speed_px_s']
        self.max_speed = params['max_speed_px_s']
        self.static_speed = params['static_speed_px_s']
        self.end_margin = params['end_margin_px']
        self.end_margin_s = params.get('end_margin_s', 0.15)   # 离端点的余量还要 ≥ 速度 × 这么多秒（滑块快时换向前减速的那段更长）
        self.edge_s = 0.3                              # 端点判断：极值离历史两端至少这么久才算"到过又回来"
        self.half_period = 0.5 * control_period_s
        self.period = control_period_s
        self.static_since = None

    def known_ends(self, t, u):
        """已知的行程端点 {'min': 像素, 'max': 像素}（只包含确认折返过的）。"""
        ends = {}
        for name, index in (('min', int(np.argmin(u))), ('max', int(np.argmax(u)))):
            if t[index] - t[0] > self.edge_s and t[-1] - t[index] > self.edge_s:
                ends[name] = float(u[index])
        return ends

    def check(self, now, samples):
        """samples：[(t, u, v), ...] 按时间排序（历史尽量长，用来找行程端点）。返回 (fire, info)。"""
        info = {'samples': len(samples)}
        if not samples:
            info['reason'] = 'no_target'
            return False, info
        age = now - samples[-1][0]
        info['age'] = age
        if age > self.max_sample_age_s:
            info['reason'] = 'stale'
            return False, info
        clean = drop_misreads(samples, self.outlier_px)
        t = np.array([s[0] for s in clean])
        u = np.array([s[1] for s in clean])
        v = np.array([s[2] for s in clean])
        latest = t[-1]
        window = t >= latest - self.fit_window_s
        if window.sum() < self.min_samples or t[window].max() - t[window].min() < 1e-3:
            # 窗口内样本不够（帧率低或中途丢帧）：不拿更早的旧样本凑数，否则速度会算错
            info['reason'] = 'few_samples'
            return False, info
        speed, u_latest = line_fit(t[window] - latest, u[window])
        u_now = u_latest + speed * (now - latest)
        u_hit = u_now + speed * self.latency_s                  # 弹丸到达时刻目标的位置
        v_now = float(np.median(v[window]))
        v_ok = (not self.use_v) or abs(v_now - self.aim_v) <= self.tolerance_v
        info.update({'u': u_now, 'speed': speed, 'err_px': u_hit - self.aim_u, 'v': v_now})
        if abs(speed) > self.max_speed:
            info['reason'] = 'bad_fit'                           # 速度不可能这么快：数据有问题，不信
            return False, info

        # ---- 静止目标 ----
        if abs(speed) < self.static_speed:
            if self.static_since is None:
                self.static_since = now
            if now - self.static_since >= self.run_window_s and abs(u_now - self.aim_u) <= self.hit_tolerance and v_ok:
                info['reason'] = 'static'
                return True, info
            info['reason'] = 'slow'
            return False, info
        self.static_since = None
        if abs(speed) < self.min_speed:
            info['reason'] = 'slow'                              # 正在换向
            return False, info

        # ---- 换向检查 1：最近 run_window_s 前后两半速度方向一致 ----
        run = t >= latest - self.run_window_s
        middle = latest - self.run_window_s / 2.0
        first, second = run & (t < middle), t >= middle
        if first.sum() < 3 or second.sum() < 3:
            info['reason'] = 'few_samples'
            return False, info
        speed1, _ = line_fit(t[first] - latest, u[first])
        speed2, _ = line_fit(t[second] - latest, u[second])
        if speed1 * speed < 0 or speed2 * speed < 0 or min(abs(speed1), abs(speed2)) < 0.5 * self.min_speed:
            info['reason'] = 'turning'
            return False, info

        # ---- 换向检查 2：前方的行程端点 ----
        ends = self.known_ends(t, u)
        end_ahead = ends.get('max' if speed > 0 else 'min')
        if end_ahead is not None:
            info['end'] = end_ahead
            margin = max(self.end_margin, abs(speed) * self.end_margin_s)
            if (end_ahead - u_hit) * math.copysign(1.0, speed) < margin:
                info['reason'] = 'near_end'
                return False, info

        # ---- 开火时机 ----
        # 2026-10-23：不在控制周期上取整。开火时刻落在下一个周期之前 → 返回 delay_s，步骤里精确等 delay_s 秒再开火。
        # 以前"lead ≤ 半个周期就开"，开火时刻最多差一个周期（0.05 秒）：18 像素/秒时差 1 像素，滑块快到 60 像素/秒时差 3 像素
        lead = (self.aim_u - u_now) / speed - self.latency_s     # 还要等多久再开火（秒）
        info['lead_s'] = lead
        if lead > self.period:
            info['reason'] = 'waiting'
            return False, info
        delay = max(0.0, lead)
        u_hit = u_now + speed * (delay + self.latency_s)        # 等 delay 秒再开火时，弹丸到达时目标的位置
        info['err_px'] = u_hit - self.aim_u
        if abs(u_hit - self.aim_u) > self.hit_tolerance or not v_ok:
            info['reason'] = 'missed' if v_ok else 'height'      # 已经过去了，等它回来
            return False, info
        info.update({'reason': 'fire', 'delay_s': delay})
        return True, info


class MovingInterceptor(object):
    """移动靶截击（2026-10-23，shooting.yaml moving.intercept）：码要很久才回到弹着点时，原地扭车头把弹着点挪到码更早经过的地方。

    为什么：10-23 实测滑块约 15 像素/秒、单程约 7.5 秒，6 号码行程在画面 u≈254~363，弹着点 350.5 离右端只有约 12 像素。
    码刚往左走过弹着点，要走到最左端再回来，等 9~12 秒（7 次里 4 次）；码正好往弹着点走时 1~2 秒就打了。
    车停着开火最准（不能边扭边打：底盘起步 0.3~0.5 秒、原地转只出指令的约 0.2 倍，追不上），所以只在开火前挪一次：
      1. 车停稳看 min_span_s 秒，拟合码的位置、速度、方向；行程两端先用 stroke_px（标定值，按车头朝向换算），
         看到码折返或者走出了标定范围就按实测修正（行程宽度不变）；
      2. 按"三角波往返（端点停 dwell_s 秒）"算码下一次经过每个候选点 p 的时刻；候选点在扭得到的范围内
         （车头相对侧墙不超过 max_twist_deg）、离两端至少 end_margin_px（换向前后速度不稳，不在那打）；
         把弹着点挪到 p 要的时间 = 起步 + 角度 / 转速 + 停稳后重新攒数据（settle_s）；
      3. 最早的候选点比"不动、等码回到弹着点"至少早 min_gain_s 秒，才扭；扭完停稳，照常由 MovingPredictor 按画面预测开火。
    扭 Δψ（左转为正）画面右移 k × Δψ：要把候选点 p 挪到弹着点，Δψ = (弹着点 − p) / k。
    """

    def __init__(self, params, target_id):
        ip = params.get('intercept') or {}
        self.enabled = bool(ip.get('enabled', False))
        strokes = ip.get('stroke_px') or {}
        stroke = strokes.get(target_id, strokes.get(str(target_id)))
        self.prior = (float(stroke[0]), float(stroke[1])) if stroke else None
        self.prior_heading = math.radians(ip.get('stroke_heading_deg', 0.0))   # 标定 stroke_px 时车头相对侧墙的朝向
        self.width = (self.prior[1] - self.prior[0]) if self.prior else ip.get('stroke_width_px', 107.0)
        self.k = math.degrees(1.0) * ip.get('twist_px_per_deg', 8.8)          # 像素 / 弧度
        self.max_twist = math.radians(ip.get('max_twist_deg', 6.0) - ip.get('twist_margin_deg', 0.8))
        self.margin = ip.get('end_margin_px', 15.0)
        self.min_gain = ip.get('min_gain_s', 2.5)
        self.max_turn = math.radians(ip.get('max_turn_deg', 8.0))         # 一次最多扭这么多
        self.done = 0                                                     # 已经扭过几次（第二次要求省得更多）
        self.start_s = ip.get('twist_start_s', 0.8)
        self.rate = math.radians(ip.get('twist_rate_deg_s', 4.0))
        self.coast_s = ip.get('twist_coast_s', 0.5)
        self.settle_s = ip.get('settle_s', 1.2)
        self.buffer_s = ip.get('buffer_s', 0.5)             # 码到截击点前至少还要留这么多秒（估得不准的余量）
        self.unsure_px = ip.get('unsure_margin_px', 8.0)    # 行程两端还没亲眼看到（用的是标定值）时，离端点再多留这么多
        self.dwell_s = ip.get('dwell_s', 0.4)
        self.min_span = ip.get('min_span_s', 0.5)
        self.min_move_px = ip.get('min_move_px', 4.0)
        self.aim = params['aim_u_px']
        self.min_speed = params['min_speed_px_s']
        self.fit_window = params['fit_window_s']
        self.outlier_px = params['outlier_px']
        self.fire_margin = params['end_margin_px']
        self.edge_s = 0.3
        self.ends_seen = False

    def move_time(self, turn):
        """扭 turn 弧度、停稳、攒够数据能开火要多久（规划用；每扭一次按实测修正 start_s）。"""
        return self.start_s + abs(turn) / self.rate + self.coast_s + self.settle_s + self.buffer_s

    def learn(self, turn, seconds):
        """一次扭车头从开始到停稳实际用了 seconds 秒：修正起步时间（和上次平均，限 0.4~2.5 秒）。"""
        if abs(turn) < math.radians(1.0):
            return
        measured = seconds - abs(turn) / self.rate - self.coast_s
        self.start_s = min(2.5, max(0.4, 0.5 * self.start_s + 0.5 * measured))

    def first_pass(self, u0, v, ends, p, t_min, horizon=40.0):
        """码从 u0 以速度 v（像素/秒，带方向）出发，在 ends=(L, R) 之间三角波往返，t_min 秒以后第一次经过 p 的时刻；没有返回 None。"""
        left, right = ends
        if not left < p < right or abs(v) < 1e-6:
            return None
        speed = abs(v)
        direction = 1.0 if v > 0 else -1.0
        pos = min(max(u0, left), right)
        t = 0.0
        for _ in range(12):
            end = right if direction > 0 else left
            if (p - pos) * direction >= 0 and (end - p) * direction >= 0:
                t_hit = t + (p - pos) * direction / speed
                if t_hit >= t_min:
                    return t_hit
            t += (end - pos) * direction / speed + self.dwell_s
            pos, direction = end, -direction
            if t > horizon:
                break
        return None

    def ends(self, t, u, heading):
        """行程两端（当前画面坐标）：标定值按车头朝向换算，再用这次看到的数据修正。拿不准返回 None。"""
        ends = None
        if self.prior is not None:
            shift = self.k * (heading - self.prior_heading) if heading is not None else 0.0
            ends = (self.prior[0] + shift, self.prior[1] + shift)
        i_min, i_max = int(np.argmin(u)), int(np.argmax(u))
        seen_min = t[i_min] - t[0] > self.edge_s and t[-1] - t[i_min] > self.edge_s   # 在左端折返过
        seen_max = t[i_max] - t[0] > self.edge_s and t[-1] - t[i_max] > self.edge_s
        self.ends_seen = seen_min or seen_max
        if seen_min:
            return float(u[i_min]), float(u[i_min]) + self.width
        if seen_max:
            return float(u[i_max]) - self.width, float(u[i_max])
        if ends is None:
            return None
        if u.min() < ends[0]:
            ends = (float(u.min()), float(u.min()) + self.width)
        if u.max() > ends[1]:
            ends = (float(u.max()) - self.width, float(u.max()))
        return ends

    def plan(self, now, samples, yaw, yaw_ref, heading, time_left=None):
        """车停着时调用。samples：停稳以后目标码的 [(t, u, v)]（当前画面坐标）；yaw：车头（里程计，弧度）；
        yaw_ref：车头正对侧墙时的里程计朝向（扭的上限从这算）；heading：车头相对侧墙的朝向（弧度，拿不到 None）。
        返回 (目标车头 或 None, info)。
        """
        info = {}
        if not self.enabled or yaw is None or yaw_ref is None or len(samples) < 8:
            return None, dict(info, intercept='no_data')
        clean = drop_misreads(samples, self.outlier_px)
        t = np.array([x[0] for x in clean])
        u = np.array([x[1] for x in clean])
        if t[-1] - t[0] < self.min_span:
            return None, dict(info, intercept='short')
        window = t >= t[-1] - self.fit_window
        if window.sum() < 4:
            return None, dict(info, intercept='few')
        v, u_last = line_fit(t[window] - t[-1], u[window])
        if abs(v) < self.min_speed:
            return None, dict(info, intercept='slow')       # 正在换向：方向不定，等一会儿再算
        u_now = u_last + v * (now - t[-1])
        ends = self.ends(t, u, heading)
        if ends is None:
            return None, dict(info, intercept='no_ends')
        margin = self.margin + (0.0 if self.ends_seen else self.unsure_px)
        info.update({'ends': '%.0f-%.0f' % ends, 'speed': v})
        # 不动：码下一次经过弹着点（离端点太近 MovingPredictor 不开火，当作经过不了）
        t_stay = None
        if ends[0] + self.fire_margin < self.aim < ends[1] - self.fire_margin:
            t_stay = self.first_pass(u_now, v, ends, self.aim, 0.0)
        best = None
        offset = wrap_angle(yaw - yaw_ref)
        p_low, p_high = ends[0] + margin, ends[1] - margin
        p = math.ceil(p_low)
        while p <= p_high:
            turn = (self.aim - p) / self.k                   # 要扭的角度（左转为正）
            if abs(offset + turn) <= self.max_twist and abs(turn) <= self.max_turn:
                move = abs(self.aim - p) >= self.min_move_px
                need = self.move_time(turn) if move else 0.0
                t_hit = self.first_pass(u_now, v, ends, p, need)
                if time_left is not None and t_hit is not None and t_hit > time_left:
                    t_hit = None                             # 这一步剩下的时间里等不到
                if t_hit is not None and (best is None or t_hit < best[0]):
                    best = (t_hit, p, turn if move else 0.0)
            p += 2.0
        info['stay_s'] = t_stay if t_stay is not None else -1.0
        if best is None:
            return None, dict(info, intercept='no_reach')
        info.update({'intercept_s': best[0], 'p': best[1], 'turn_deg': math.degrees(best[2])})
        gain = self.min_gain * (1.5 if self.done else 1.0)                  # 第二次扭要省得更多才值得
        stay_ok = t_stay is not None and (time_left is None or t_stay <= time_left)
        if best[2] == 0.0 or (stay_ok and best[0] + gain > t_stay):
            return None, dict(info, intercept='stay')
        self.done += 1
        return yaw + best[2], dict(info, intercept='twist')


# ============================================================
# 旋转靶
# ============================================================
def fit_circle(u, v):
    """代数法拟合圆：返回 (圆心 u, 圆心 v, 半径)。"""
    a = np.c_[2 * u, 2 * v, np.ones(len(u))]
    b = u * u + v * v
    (uc, vc, c), _, _, _ = np.linalg.lstsq(a, b, rcond=None)
    return float(uc), float(vc), math.sqrt(max(c + uc * uc + vc * vc, 0.0))


def circular_mean(angles):
    return math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))


class WheelModel(object):
    """转轮模型：圆心 (uc, vc)、每个码的半径和相位、共同的角速度 omega。

    角度 = atan2(v - vc, u - uc)。v 向下，所以角度增大在画面里是顺时针。
    码 k 在时刻 t 的角度 = phases[k] + omega * (t - t_ref)。
    """

    def __init__(self, uc, vc, omega, phases, radii, t_ref, static):
        self.uc, self.vc = uc, vc
        self.omega = omega
        self.phases = phases
        self.radii = radii
        self.t_ref = t_ref
        self.static = static            # 转轮没在转（或转得太慢测不出来）

    @property
    def period(self):
        return 2 * math.pi / abs(self.omega) if not self.static else float('inf')

    def angle(self, marker_id, t):
        return self.phases[marker_id] + (0.0 if self.static else self.omega * (t - self.t_ref))

    def position(self, marker_id, t):
        angle = self.angle(marker_id, t)
        radius = self.radii[marker_id]
        return self.uc + radius * math.cos(angle), self.vc + radius * math.sin(angle)

    def time_until(self, marker_id, angle, t):
        """从 t 起，码转到 angle 还要多久（0 ~ 一圈）。"""
        turn = (angle - self.angle(marker_id, t)) * math.copysign(1.0, self.omega)
        return (turn % (2 * math.pi)) / abs(self.omega)

    def signed_time_until(self, marker_id, angle, t):
        """码转到 angle 的时间，范围 (-半圈, +半圈]：负数表示刚转过去。"""
        return wrap_angle(angle - self.angle(marker_id, t)) / self.omega

    def crossings(self, marker_id, aim_v):
        """码经过枪口高度 v = aim_v 的位置：[{'side', 'u', 'angle', 'v_miss'}, ...]。

        一般有左右两个交点。枪口高度在圆外（打不到圆周）时只给最近的一点（圆的最高或最低点），
        v_miss = 差多少像素，尽力而为。
        """
        radius = self.radii[marker_id]
        dv = aim_v - self.vc
        if abs(dv) < radius:
            du = math.sqrt(radius * radius - dv * dv)
            return [{'side': 'left', 'u': self.uc - du, 'angle': math.atan2(dv, -du), 'v_miss': 0.0},
                    {'side': 'right', 'u': self.uc + du, 'angle': math.atan2(dv, du), 'v_miss': 0.0}]
        side = 'bottom' if dv > 0 else 'top'
        return [{'side': side, 'u': self.uc, 'angle': math.copysign(math.pi / 2, dv), 'v_miss': abs(dv) - radius}]


def fit_wheel(samples_by_id, min_markers=3, outlier_px=3.0, min_omega=math.radians(0.5),
              omega_lag_s=0.5, max_phase_dev=math.radians(8.0)):
    """用转轮上所有码的观测拟合 WheelModel。返回 (model, 原因)，失败时 model 为 None。

    samples_by_id：{码号: [(t, u, v), ...]}，只放车停稳以后拍的帧。
    实测（rotating.bag）：5 个码大多同时可见，单帧拟圆的圆心误差约 0.5 像素；码之间相位差约 72 度。
    误识别处理：
      码被认成旁边的码时，位置仍在圆周上（拟圆剔除不掉），但相位和自己的轨迹差几十度
      → 角速度用"相隔 omega_lag_s 的两点角度差"的中位数（不受个别坏点影响），
        每个码的相位再剔除偏离超过 max_phase_dev 的点。
    """
    ids, ts, us, vs = [], [], [], []
    for marker_id, samples in samples_by_id.items():
        if len(samples) < 3:
            continue
        for t, u, v in samples:
            ids.append(marker_id)
            ts.append(t)
            us.append(u)
            vs.append(v)
    if len(set(ids)) < min_markers:
        return None, 'few_markers'
    ids, ts, us, vs = np.array(ids), np.array(ts), np.array(us), np.array(vs)
    uc, vc, radius = fit_circle(us, vs)
    keep = np.abs(np.hypot(us - uc, vs - vc) - radius) <= outlier_px
    if keep.sum() < 3 * min_markers:
        return None, 'bad_circle'
    ids, ts, us, vs = ids[keep], ts[keep], us[keep], vs[keep]
    uc, vc, radius = fit_circle(us, vs)
    angles = np.arctan2(vs - vc, us - uc)
    t_ref = float(ts.max())

    # ---- 角速度：每个码相隔 omega_lag_s 的两点角度差 / 时间差，全部取中位数 ----
    rates = []
    for marker_id in set(ids.tolist()):
        sel = ids == marker_id
        order = np.argsort(ts[sel])
        mt, ma = ts[sel][order], angles[sel][order]
        later = np.searchsorted(mt, mt + omega_lag_s)
        valid = later < len(mt)
        if valid.any():
            i = np.nonzero(valid)[0]
            j = later[valid]
            dt = mt[j] - mt[i]
            rates.extend((np.arctan2(np.sin(ma[j] - ma[i]), np.cos(ma[j] - ma[i])) / dt).tolist())
    if len(rates) < 5:
        return None, 'short_observation'
    omega = float(np.median(rates))
    static = abs(omega) < min_omega
    if static:
        omega = 0.0

    # ---- 每个码的相位（t_ref 时刻的角度）和半径 ----
    phases, radii = {}, {}
    for marker_id in set(ids.tolist()):
        sel = ids == marker_id
        rel = angles[sel] - omega * (ts[sel] - t_ref)
        phase = circular_mean(rel)
        deviation = np.abs(np.arctan2(np.sin(rel - phase), np.cos(rel - phase)))
        good = deviation <= max_phase_dev
        if good.sum() < 2:
            continue
        phases[marker_id] = circular_mean(rel[good])
        radii[marker_id] = float(np.mean(np.hypot(us[sel][good] - uc, vs[sel][good] - vc)))
    return WheelModel(uc, vc, omega, phases, radii, t_ref, static), 'ok'


class RotatingPlanner(object):
    """旋转靶：拟合转轮，预测目标码经过"打得到的点"的时刻，到时刻开火。

    打得到的点（候选点）：
      closest  圆周上离弹着点最近的点。车不用动，偏差 = |弹着点到圆心距离 - 半径|。
      left / right  圆和"弹着点高度水平线"的两个交点。车横向平移把交点挪到弹着点上。
    优先不平移（2026-10-17 晚 5 次实测）：横移开环误差大——底盘死区约 0.3 秒，指令 15 mm 实际走了 26 mm，
      横移时车身还会偏 0.2~0.4 度（1.5 m 外 3~5 像素），修正用完只能偏 4~9 像素开火；
      而精定位靠墙闭环，那 5 次如果一步不横移、在最近点开火，偏差是 0.1 / 2.6 / 3.0 / 3.9 / 3.9 像素。
      所以 closest 偏差 ≤ hit_tolerance_px 就直接等它；不行才平移到交点（最多 max_corrections 次）。
    fire_side（2026-10-18）：只在转轮的哪一侧开火。right = 只用右交点、以及落在圆心右边的最近点（画面里码在转轮右半边时开火）；
      left 同理；any = 不限。实测弹着点几乎落在轮毂上，左右两侧都要平移约一个半径，按场上要求固定打右侧。
      没有符合的候选点（弹着点高度在圆外，只剩最高/最低点）时退回不限，保证这一步不会卡住。
    correction（2026-10-23）：交点离弹着点太远时怎么修。
      shift  横向平移（旧做法，最多 max_shift_m，够不着一个半径）；
      twist  原地扭车头（YawTwister）：要修的像素 ≥ twist_min_px 时扭 Δψ = Δu / twist_px_per_deg 度（左转为正），
             相对这一步开始时最多扭 max_twist_deg；小于 twist_min_px 的仍用平移（垫子上扭的最小一下约 3~4 度）。
             扭过一次后用"转轮圆心在画面里移了多少 / 侧墙偏角实际变了多少"更新换算系数。
             左右交点都瞄：按"算上扭车头的准备时间，目标码哪个交点先到"选（fire_side: any），
             动过一次车（扭或平移）以后就盯住那一侧，不再来回换。

    状态：
      observe  车停稳后观察 observe_s 秒，拟合转轮 → 选候选点（_plan）→ 需要平移就进 shift，否则进 wait。
      shift    平移（LateralShifter），完了回到 observe 重新看。
      wait     每个周期用最新观测重新拟合，弹丸到达时刻目标离弹着点 ≤ hit_tolerance_px、且到点时间 ≈ latency_s → 开火。
               静止满 keepalive_still_s 秒、离开火还早 → 后退再前进各 keepalive_nudge_s 秒（nudge），
               防止"20 秒状态不变"被判结束。前后挪只改距离，画面里转轮几乎不动（1.5 m 外 1.5 cm 只差约 0.2 像素），
               不像左右挪会把位置带偏。回来后重新观察一次。
      nudge    keepalive 前后挪（按时间开环：先后退再前进，前后增益不同会净退几毫米，只改距离，不影响瞄准）。
    等待时间超过截止时刻（deadline）→ 放弃（give_up），把时间留给后面的任务。

    update() 返回 (vx, vy, fire, info)。
    """

    def __init__(self, params, shift_params, camera_fx, target_id, control_period_s, twist_params=None):
        p = params
        self.aim_u = p['aim_u_px']
        self.aim_v = p['aim_v_px']
        self.hit_tolerance = p['hit_tolerance_px']
        self.latency_s = p['latency_s']
        self.fire_side = p.get('fire_side', 'any')
        self.camera_fx = camera_fx
        self.distance = p['target_distance_m']
        self.fresh_after_s = p['fresh_after_s']
        self.observe_s = p['observe_s']
        self.fit_window_s = p['fit_window_s']
        self.min_markers = int(p['min_markers'])
        self.outlier_px = p['outlier_px']
        self.min_omega = math.radians(p['min_omega_deg_s'])
        self.max_shift = p['max_shift_m']
        self.max_corrections = int(p['max_corrections'])
        self.plan_overhead_s = p['plan_overhead_s']
        self.plan_speed = p['plan_speed_mps']
        self.keepalive_still_s = p['keepalive_still_s']
        self.keepalive_nudge_s = p['keepalive_nudge_s']
        self.keepalive_speed = p['keepalive_speed_mps']
        self.keepalive_lead_s = p['keepalive_min_lead_s']
        # 至少看这么久才允许放弃（不能超过 fit_window_s：拟合最多只用这么长的数据）
        self.give_up_observe_s = min(p.get('give_up_observe_s', 0.0), self.fit_window_s)
        self.observed_s = 0.0
        self.shifter = LateralShifter(shift_params)
        self.shift_tolerance = self.shifter.tolerance
        # ---- 扭车头（correction: twist）----
        self.correction = p.get('correction', 'shift')
        self.twister = YawTwister(twist_params) if twist_params else None
        self.px_per_rad = math.degrees(1.0) * p.get('twist_px_per_deg', 8.8)
        self.px_per_rad_theory = self.px_per_rad
        self.max_twist = math.radians(p.get('max_twist_deg', 6.0))
        self.twist_min_px = p.get('twist_min_px', 10.0)
        self.plan_twist_rate = math.radians(p.get('plan_twist_deg_s', 3.0))   # 规划时按这个扭转速度估算用时
        # 扭车头的固定准备时间（扭完停稳 + 重新观察 + 可能还要补一次小平移），比平移的 plan_overhead_s 留得多：
        # 留少了，扭得不准再补一次就赶不上这一圈，要多等 43 秒
        self.plan_twist_overhead = p.get('plan_twist_overhead_s', self.plan_overhead_s + 1.5)
        # 已经在等某个点时，偏差在这个范围内就接着等（不重新规划），免得在容差边上来回跳、错过这一圈
        self.keep_px = p.get('wait_keep_px', self.hit_tolerance + 1.5)
        self.yaw_origin = None          # 步骤开始时的车头（侧墙偏角），扭的上限从这里算
        self.twist_before = None        # 上一次扭之前的 (转轮圆心 u, 车头)，扭完后更新换算系数用
        self.committed_side = None      # 动过一次车（扭或平移）以后盯住的那一侧
        self.wz = 0.0                   # 扭车头时的角速度指令（步骤读它）
        self.yaw = None                 # 最近一次的车头（侧墙偏角）
        self.twists = 0
        self.target = target_id
        self.half_period = 0.5 * control_period_s
        self.state = 'observe'
        self.look_since = None
        self.origin = None              # 步骤开始时的横向位置（平移上限从这里算）
        self.corrections = 0
        self.nudge_phase = None         # keepalive：'back'（后退）→ 'forward'（前进回来）
        self.nudge_until = None
        self.model = None

    # ---------------- 主入口 ----------------
    def update(self, now, lateral, samples_by_id, robot_still, still_for_s, deadline, yaw=None):
        """一个控制周期。

        lateral：横向位置（米，左为正），拿不到为 None；samples_by_id：转轮上各码的像素观测；
        robot_still：车已停稳；still_for_s：车已经连续静止多久；deadline：最晚开火时刻（绝对时间）；
        yaw：车头朝向（侧墙偏角，弧度，左转为正），拿不到为 None（就不能扭车头）。
        返回 (vx, vy, fire, info)；扭车头时的角速度指令在 self.wz。info['give_up'] 为 True 表示等不起了，步骤应结束。
        """
        self.wz = 0.0
        if self.origin is None and lateral is not None:
            self.origin = lateral
        if self.yaw_origin is None and yaw is not None:
            self.yaw_origin = yaw
        self.yaw = yaw
        if self.state == 'twist':
            return self._twist(now, yaw)
        if self.state == 'shift':
            vy, fire, info = self._shift(now, lateral)
            return 0.0, vy, fire, info
        if self.state == 'nudge':
            return self._nudge(now)
        if not robot_still:
            self.look_since = None
            return 0.0, 0.0, False, {'state': self.state, 'reason': 'settling'}
        if self.look_since is None:
            self.look_since = now
        fresh_from = self.look_since + self.fresh_after_s
        if now - fresh_from < self.observe_s:
            return 0.0, 0.0, False, {'state': self.state, 'reason': 'observing'}
        window_from = max(fresh_from, now - self.fit_window_s)
        self.observed_s = now - window_from              # 这次拟合用了多长时间的数据
        fresh = dict((k, [s for s in v if s[0] >= window_from]) for k, v in samples_by_id.items())
        model, why = fit_wheel(fresh, self.min_markers, self.outlier_px, self.min_omega)
        if model is None:
            return 0.0, 0.0, False, {'state': self.state, 'reason': why}
        if self.target not in model.phases:
            return 0.0, 0.0, False, {'state': self.state, 'reason': 'target_not_seen'}
        self.model = model
        if self.state == 'observe':
            self._learn_twist(model, yaw)
            vy, fire, info = self._plan(now, lateral, model, deadline)
            return 0.0, vy, fire, info
        return self._wait(now, lateral, model, still_for_s)

    # ---------------- 各状态 ----------------
    def _shift(self, now, lateral):
        vy, status, remaining = self.shifter.update(now, lateral)
        info = {'state': 'shift'}
        if remaining is not None:
            info['left_mm'] = remaining * 1000
        if status == 'moving':
            return vy, False, info
        if status == 'no_wall':
            info['reason'] = 'no_wall'
            return 0.0, False, info
        self.state = 'observe'
        self.look_since = None
        info.update({'state': 'observe', 'reason': 'shift_' + status})
        return 0.0, False, info

    def _twist(self, now, yaw):
        wz, status, remaining = self.twister.update(now, yaw)
        info = {'state': 'twist'}
        if remaining is not None:
            info['left_deg'] = math.degrees(remaining)
        if status == 'moving':
            self.wz = wz
            return 0.0, 0.0, False, info
        self.state = 'observe'
        self.look_since = None
        info.update({'state': 'observe', 'reason': 'twist_' + status})
        return 0.0, 0.0, False, info

    def _learn_twist(self, model, yaw):
        """扭完第一次重新拟合转轮时：转轮圆心在画面里移了多少像素 / 侧墙偏角实际变了多少 → 更新换算系数。"""
        if self.twist_before is None or yaw is None:
            return
        uc_before, yaw_before = self.twist_before
        self.twist_before = None
        turned = wrap_angle(yaw - yaw_before)
        if abs(turned) < math.radians(1.0):
            return                                          # 转得太少，除出来全是噪声
        measured = (model.uc - uc_before) / turned          # 左转（turned > 0）→ 转轮右移（u 增大）
        low, high = 0.5 * self.px_per_rad_theory, 1.6 * self.px_per_rad_theory
        self.px_per_rad = min(high, max(low, measured))

    def _shift_fits(self, lateral, du):
        """平移能不能修这么多像素（不超过平移上限）。能平移就平移：比扭准（垫子上扭的最小一下约 3~4 度）。"""
        if lateral is None or self.origin is None or self.corrections >= self.max_corrections:
            return False
        shift = pixels_to_shift(du, self.camera_fx, self.distance)
        return abs(lateral + shift - self.origin) <= self.max_shift

    def _can_twist(self):
        return (self.correction == 'twist' and self.twister is not None and self.yaw is not None
                and self.yaw_origin is not None and self.corrections < self.max_corrections)

    def _nudge(self, now):
        info = {'state': 'nudge', 'phase': self.nudge_phase}
        if now < self.nudge_until:
            sign = -1.0 if self.nudge_phase == 'back' else 1.0
            return sign * self.keepalive_speed, 0.0, False, info
        if self.nudge_phase == 'back':
            self.nudge_phase = 'forward'
            self.nudge_until = now + self.keepalive_nudge_s
            return self.keepalive_speed, 0.0, False, info
        self.state = 'observe'
        self.look_since = None
        info.update({'state': 'observe', 'reason': 'nudge_done'})
        return 0.0, 0.0, False, info

    def _candidates(self, model, now):
        """候选点：[{'side', 'u', 'angle', 'offset'}]。

        u：这一点的横坐标（平移用）；angle：目标码转到这一点时的角度；
        offset：车不动时，目标码到这一点时离弹着点多少像素。
        """
        if model.static:
            u, v = model.position(self.target, now)
            return [{'side': 'static', 'u': u, 'angle': model.angle(self.target, now),
                     'offset': math.hypot(u - self.aim_u, v - self.aim_v)}]
        radius = model.radii[self.target]
        du, dv = self.aim_u - model.uc, self.aim_v - model.vc
        angle = math.atan2(dv, du)
        candidates = [{'side': 'closest', 'u': model.uc + radius * math.cos(angle), 'angle': angle,
                       'offset': abs(math.hypot(du, dv) - radius)}]
        for c in model.crossings(self.target, self.aim_v):
            candidates.append({'side': c['side'], 'u': c['u'], 'angle': c['angle'],
                               'offset': math.hypot(c['u'] - self.aim_u, c['v_miss'])})
        allowed = [c for c in candidates if self._side_ok(c)]
        return allowed or candidates

    def _side_ok(self, candidate):
        """候选点是否在 fire_side 指定的一侧（closest 按它在圆心左边还是右边算；最高/最低点不分左右）。"""
        if self.fire_side not in ('left', 'right'):
            return True
        side = candidate['side']
        if side == 'closest':
            side = 'right' if math.cos(candidate['angle']) > 0 else 'left'
        if side not in ('left', 'right'):
            return True
        return side == self.fire_side

    def _plan(self, now, lateral, model, deadline):
        options = []
        candidates = self._candidates(model, now)
        if self.committed_side is not None:
            # 已经为这一侧动过车：盯住它（还在原地打得到的最近点也算），不再来回换
            same = [c for c in candidates if c['side'] == self.committed_side
                    or (c['side'] == 'closest' and c['offset'] <= self.hit_tolerance)]
            candidates = same or candidates
        for c in candidates:
            twist = 0.0
            if c['offset'] <= self.hit_tolerance:
                shift, need_shift = 0.0, False                       # 不用动就打得到
            elif (c['side'] in ('left', 'right') and self._can_twist()
                    and abs(self.aim_u - c['u']) >= self.twist_min_px
                    and not self._shift_fits(lateral, self.aim_u - c['u'])):
                # 扭车头：交点要在画面里移 Δu 像素 → 车头转 Δu / 换算系数（左转为正，画面里转轮右移）
                twist = (self.aim_u - c['u']) / self.px_per_rad
                shift, need_shift = 0.0, False
                total = self.yaw - self.yaw_origin + twist
                excess_rad = max(0.0, abs(total) - self.max_twist)
                prep = self.plan_twist_overhead + abs(twist) / self.plan_twist_rate
                wait = 0.0 if model.static else model.time_until(self.target, c['angle'], now)
                if not model.static and wait < prep + self.latency_s:
                    wait += model.period
                options.append(dict(c, shift=0.0, need_shift=False, twist=twist, excess=excess_rad * self.distance,
                                    wait=wait))
                continue
            elif c['side'] in ('left', 'right'):
                shift = pixels_to_shift(self.aim_u - c['u'], self.camera_fx, self.distance)
                # 测不到横向位置（看不到侧墙）就不能平移
                need_shift = self.corrections < self.max_corrections and lateral is not None
                if not need_shift:
                    continue
            else:
                continue                                              # 最近点偏太多、又没法靠平移修正
            excess = 0.0
            if need_shift and self.origin is not None:
                excess = max(0.0, abs(lateral + shift - self.origin) - self.max_shift)
            prep = (self.plan_overhead_s + abs(shift) / self.plan_speed) if need_shift else 0.0
            wait = 0.0 if model.static else model.time_until(self.target, c['angle'], now)
            if not model.static and wait < prep + self.latency_s:
                wait += model.period                        # 来不及，等下一圈
            options.append(dict(c, shift=shift, need_shift=need_shift, twist=0.0, excess=excess, wait=wait))
        info = {'state': 'observe', 'omega_deg_s': math.degrees(model.omega),
                'center': '%.1f,%.1f' % (model.uc, model.vc), 'radius': model.radii[self.target]}
        if lateral is None:
            info['no_wall'] = True
        if not options:
            # 不平移打不到、也没法平移：直接等，_wait 里会按最近点尽力而为
            self.state = 'wait'
            info['reason'] = 'wait_best_effort'
            return 0.0, False, info
        in_time = [o for o in options if o['excess'] == 0.0 and now + o['wait'] <= deadline]
        still = [o for o in in_time if not o['need_shift'] and not o['twist']]
        if self.correction == 'twist' and in_time:
            # 左右都瞄：算上准备时间，哪边先到打哪边（一样早时优先不动车）
            best = min(in_time, key=lambda o: (o['wait'], bool(o['need_shift'] or o['twist'])))
        elif still:
            best = min(still, key=lambda o: o['wait'])               # 优先不平移
        elif in_time:
            best = min(in_time, key=lambda o: o['wait'])
        else:
            best = min(options, key=lambda o: (o['excess'], o['wait']))   # 都超出平移上限或截止时刻：尽力而为
            if best['excess'] > 0:
                info['clamped_mm'] = best['excess'] * 1000
        info.update({'side': best['side'], 'wait_s': best['wait'], 'shift_mm': best['shift'] * 1000,
                     'offset_px': best['offset']})
        if best['twist']:
            info['twist_deg'] = math.degrees(best['twist'])
            info['k_px_deg'] = self.px_per_rad / math.degrees(1.0)        # 现在用的换算系数（扭过一次后是实测的）
            info['coast_s'] = self.twister.coast_s
        if self.yaw is not None and self.yaw_origin is not None:
            info['twisted_deg'] = math.degrees(self.yaw - self.yaw_origin)  # 相对这一步开始已经扭了多少
        if now + best['wait'] > deadline:
            if self.observed_s < self.give_up_observe_s - 1e-6:
                # 转速只看了 1 秒左右，误差可能有 20~30%（2026-10-16 实测同一转轮三次估出 8.3/10.7/8.0 度/秒）。
                # 放弃前多看一会儿再确认，免得因为转速估错白白放弃一个靶
                info['reason'] = 'confirm_give_up'
                return 0.0, False, info
            info.update({'reason': 'wait_too_long', 'give_up': True})
            return 0.0, False, info
        if best['twist']:
            total = self.yaw - self.yaw_origin + best['twist']
            total = max(-self.max_twist, min(self.max_twist, total))     # 不超过扭的上限（尽力而为）
            self.twister.start(self.yaw_origin + total, now)
            self.twist_before = (model.uc, self.yaw)
            self.state = 'twist'
            self.corrections += 1
            self.twists += 1
            self.committed_side = best['side']
            info['reason'] = 'twist'
            return 0.0, False, info
        if best['need_shift']:
            self.committed_side = best['side']
            target = lateral + best['shift']
            if self.origin is not None:
                target = max(self.origin - self.max_shift, min(self.origin + self.max_shift, target))
            self.shifter.start(target)
            self.state = 'shift'
            self.corrections += 1
            info['reason'] = 'shift'
            return 0.0, False, info
        self.state = 'wait'
        info['reason'] = 'wait'
        return 0.0, False, info

    def _wait(self, now, lateral, model, still_for_s):
        info = {'state': 'wait'}
        pred_u, pred_v = model.position(self.target, now + self.latency_s)    # 弹丸到达时刻目标的位置
        error = math.hypot(pred_u - self.aim_u, pred_v - self.aim_v)
        info['err_px'] = error
        candidates = self._candidates(model, now)
        aligned = [c for c in candidates if c['offset'] <= self.hit_tolerance]
        if not aligned:
            # 刚规划时在容差内、现在稍差一点（拟合噪声 0.5 像素左右）：接着等，不重新规划
            aligned = [c for c in candidates if c['offset'] <= self.keep_px]
        if not aligned:
            # 候选点都没对准（平移不准、或观测变了）
            if self.corrections < self.max_corrections and (lateral is not None or self._can_twist()):
                self.state = 'observe'                                # 还有修正次数：重新规划
                info['reason'] = 'replan'
                return 0.0, 0.0, False, info
            # 修正次数用完、或看不到侧墙：在最近点尽力打（总比不打强）
            best = min(candidates, key=lambda c: c['offset'])
            aligned = [best]
            info['best_effort_px'] = best['offset']
        next_hit = None
        for c in aligned:
            tolerance = max(self.hit_tolerance, c['offset'] + 0.5)
            lead = (0.0 if model.static else model.signed_time_until(self.target, c['angle'], now)) - self.latency_s
            # 只在目标码正好转到候选点时开火：-1 个控制周期 ≤ lead ≤ 半个周期（晚一个周期约差 0.4 像素，循环抖动时不至于错过一圈）。
            # 以前只判 lead ≤ 半个周期：尽力而为（容差放宽到 offset + 0.5）时，码已经转过去几秒也会开火
            # （2026-10-18 22:19 整局实测 lead = -6.2 s，等于随便挑了个时刻开枪）
            if -2 * self.half_period <= lead <= self.half_period and error <= tolerance:
                info.update({'reason': 'fire', 'side': c['side'], 'lead_s': lead})
                return 0.0, 0.0, True, info
            if lead > self.half_period:
                ahead = lead
            else:
                ahead = model.time_until(self.target, c['angle'], now) - self.latency_s   # 刚错过：等下一圈
            next_hit = ahead if next_hit is None else min(next_hit, ahead)
        info['next_s'] = next_hit
        if (still_for_s >= self.keepalive_still_s and next_hit >= self.keepalive_lead_s
                and self.keepalive_nudge_s > 0):
            self.state = 'nudge'
            self.nudge_phase = 'back'
            self.nudge_until = now + self.keepalive_nudge_s
            info['reason'] = 'keepalive'
            return -self.keepalive_speed, 0.0, False, info
        info['reason'] = 'waiting'
        return 0.0, 0.0, False, info


# ============================================================
# 旋转靶：边看边对（2026-10-23 v30，rotating.correction: servo）
# ============================================================
class WheelTracker(object):
    """转轮跟踪，车在扭、在平移时也能用（fit_wheel 要求车停稳）。

    fit_wheel 把一段时间的观测当成同一个圆心，车一动整个画面平移，拟合就乱了，所以 v29 每动一次车都要
    "停稳 → 重新看 1 秒"。这里每一帧（同一拍摄时刻的几个码）单独定圆心：
      - 这一帧看到 ≥ 3 个码：直接拟合圆（个别码误识别、偏离圆周的，去掉最差的一个再拟合）；
      - 不够 3 个：用已有模型反推（码位置 − 半径 × 这个码此刻的方向）。
    码相对"这一帧圆心"的角度不受车扭、平移影响，用它算转速和相位（同 fit_wheel：相隔 omega_lag_s 的角度差取中位数），
    所以车一边动，转速和相位一边越看越准；圆心取最近几帧的，就是"现在"转轮在画面里的位置。
    每帧圆心算一次就缓存，控制周期里只算新来的帧。
    """

    def __init__(self, min_markers=3, outlier_px=3.0, min_omega=math.radians(0.5), window_s=6.0,
                 omega_lag_s=0.5, max_phase_dev=math.radians(8.0)):
        self.min_markers = min_markers
        self.outlier_px = outlier_px
        self.min_omega = min_omega
        self.window_s = window_s
        self.omega_lag_s = omega_lag_s
        self.max_phase_dev = max_phase_dev
        self.model = None             # 最近一次成功的 WheelModel（uc, vc = 最近 3 帧圆心的中位数）
        self.cache = {}               # 帧时间 → (uc, vc)；None = 这一帧不能用
        self.recent = []              # 最近 12 帧 [(t, uc, vc)]
        self.span_s = 0.0             # 这次拟合用的数据跨了多少秒
        self.latest_t = None          # 最新一帧的拍摄时刻

    def _fit_frame(self, pts):
        """一帧里 ≥ 3 个码拟合圆心；不像一个圆（误识别）返回 None。"""
        keys = list(pts)
        while len(keys) >= 3:
            us = np.array([pts[k][0] for k in keys])
            vs = np.array([pts[k][1] for k in keys])
            uc, vc, radius = fit_circle(us, vs)
            residual = np.abs(np.hypot(us - uc, vs - vc) - radius)
            if self.model is not None:
                ref = float(np.median(list(self.model.radii.values())))
                if abs(radius - ref) > 2 * self.outlier_px and len(keys) > 3:
                    keys.pop(int(np.argmax(residual)))
                    continue
                if abs(radius - ref) > 2 * self.outlier_px:
                    return None
            if residual.max() <= self.outlier_px:
                return uc, vc
            if len(keys) == 3:
                return None
            keys.pop(int(np.argmax(residual)))
        return None

    def _frame_center(self, t, pts):
        if t in self.cache:
            return self.cache[t]
        if len(pts) >= 3:
            center = self._fit_frame(pts)
            self.cache[t] = center
            return center
        if self.model is None:
            return None                 # 先不缓存：有模型以后再反推
        est = [(u - self.model.radii[k] * math.cos(self.model.angle(k, t)),
                v - self.model.radii[k] * math.sin(self.model.angle(k, t)))
               for k, (u, v) in pts.items() if k in self.model.radii]
        center = (float(np.mean([e[0] for e in est])), float(np.mean([e[1] for e in est]))) if est else None
        self.cache[t] = center
        return center

    def update(self, samples_by_id, now):
        """用最近 window_s 秒的观测更新模型。返回 (model, 原因)，失败时 model 为 None。"""
        oldest = now - self.window_s
        frames = {}
        for marker_id, samples in samples_by_id.items():
            for t, u, v in samples:
                if t >= oldest:
                    frames.setdefault(round(t, 3), {})[marker_id] = (u, v)
        for t in [t for t in self.cache if t < oldest]:
            del self.cache[t]
        if not frames:
            return None, 'few_markers'
        self.latest_t = max(frames)
        centers = {}
        for t in sorted(frames):
            center = self._frame_center(t, frames[t])
            if center is not None:
                centers[t] = center
        if len(centers) < 3:
            return None, 'bad_circle' if len(frames) >= 3 else 'few_markers'
        # ---- 每个码相对那一帧圆心的角度、半径 ----
        per_id = {}
        for t, (uc, vc) in centers.items():
            for marker_id, (u, v) in frames[t].items():
                per_id.setdefault(marker_id, []).append((t, math.atan2(v - vc, u - uc), math.hypot(u - uc, v - vc)))
        if len([k for k, s in per_id.items() if len(s) >= 3]) < self.min_markers:
            return None, 'few_markers'
        rates = []
        for marker_id, samples in per_id.items():
            samples.sort()
            mt = np.array([s[0] for s in samples])
            ma = np.array([s[1] for s in samples])
            later = np.searchsorted(mt, mt + self.omega_lag_s)
            valid = later < len(mt)
            if valid.any():
                i = np.nonzero(valid)[0]
                j = later[valid]
                rates.extend((np.arctan2(np.sin(ma[j] - ma[i]), np.cos(ma[j] - ma[i])) / (mt[j] - mt[i])).tolist())
        if len(rates) < 5:
            return None, 'short_observation'
        omega = float(np.median(rates))
        static = abs(omega) < self.min_omega
        if static:
            omega = 0.0
        t_ref = max(centers)
        phases, radii = {}, {}
        for marker_id, samples in per_id.items():
            ts = np.array([s[0] for s in samples])
            rel = np.array([s[1] for s in samples]) - omega * (ts - t_ref)
            phase = circular_mean(rel)
            good = np.abs(np.arctan2(np.sin(rel - phase), np.cos(rel - phase))) <= self.max_phase_dev
            if good.sum() < 2:
                continue
            phases[marker_id] = circular_mean(rel[good])
            # 半径只用最近 1 秒的：进这一步之前车可能还在往框里开，离得远时转轮在画面里小
            dist = np.array([s[2] for s in samples])
            near = good & (ts >= t_ref - 1.0)
            radii[marker_id] = float(np.mean(dist[near] if near.sum() >= 2 else dist[good]))
        if not phases:
            return None, 'few_markers'
        order = sorted(centers)
        last3 = [centers[t] for t in order[-3:]]
        uc = float(np.median([c[0] for c in last3]))
        vc = float(np.median([c[1] for c in last3]))
        self.recent = [(t, centers[t][0], centers[t][1]) for t in order[-12:]]
        self.span_s = order[-1] - order[0]
        self.model = WheelModel(uc, vc, omega, phases, radii, t_ref, static)
        return self.model, 'ok'

    def center_rate(self, span_s=0.3):
        """最近 span_s 秒转轮圆心横坐标的变化率（像素/秒，右为正）：车在扭/平移时画面移多快。"""
        if len(self.recent) < 3:
            return 0.0
        t_last = self.recent[-1][0]
        pts = [(t, u) for t, u, _ in self.recent if t >= t_last - span_s]
        if len(pts) < 3 or pts[-1][0] - pts[0][0] < 0.08:
            return 0.0
        t = np.array([p[0] for p in pts])
        u = np.array([p[1] for p in pts])
        return float(np.polyfit(t - t[0], u, 1)[0])


class RotatingServo(object):
    """旋转靶"边看边对"（2026-10-23 v30，shooting.yaml rotating.correction: servo，参数在 rotating.servo）。

    为什么：v29（RotatingPlanner + 扭车头）每动一次车都要"停稳 → 重新看 1 秒 → 规划"，平移按换算开环走、走不准还要再来一轮，
    准备时间要留 5 秒以上，近的那个交点常常赶不上、只能等另一边或下一圈，实测一个旋转靶 20~35 秒。
    老程序（robot_slam/scripts/shoot_final.py）是一直看着 AR 码转车头追（angular.z = -1.4 × 码的横向偏差），
    对上就开火：不停下来看，但不预测、车还在转就开枪、只能等码自己转到枪口高度。
    这里取老程序"一直看着画面修"的做法，加上预测：
      1. 观察：转轮跟踪（WheelTracker）车动着也能用，进这一步之前（精定位最后那段）的画面也算，一般一进来就能规划；
      2. 选交点：左右交点都瞄，按"现在偏差要修多久"估准备时间，目标码哪边先到打哪边；
      3. 对准：每个控制周期都看偏差 e = 交点横坐标 − 弹着点横坐标，偏得多（> coarse_px）原地扭车头，偏得少横移；
         按"现在偏差 + 画面移动速度 × 松开后还会滑的时间"提前松开，停下来马上看（不用重新观察 1 秒），不准接着修。
         松开后滑了多少每次都实测更新（yaw_coast_s / lateral_coast_s），扭过头一次，下一次就多用横移细修。
         横移时稳车头（车头偏了画面就跟着偏）；车头相对这一步开始最多偏 max_twist_deg、横向最多 max_shift_m（里程计测），不出框；
      4. 等：对准后停稳等目标码转到交点（提前 latency_s 开火）；偏差变大、离开火还早就再对一次；
         码到了还没对准就不开这一枪，改打另一侧交点（约半圈后），不白打。
    update() 返回 (vx, vy, wz, fire, info)。
    """

    def __init__(self, params, servo_params, target_id, control_period_s):
        p = params
        s = servo_params or {}
        self.aim_u = p['aim_u_px']
        self.aim_v = p['aim_v_px']
        self.hit_tolerance = p['hit_tolerance_px']
        self.keep_px = p.get('wait_keep_px', self.hit_tolerance + 1.5)
        self.latency_s = p['latency_s']
        self.fire_side = p.get('fire_side', 'any')
        self.max_twist = math.radians(p.get('max_twist_deg', 6.0))
        self.max_shift = p['max_shift_m']
        self.keepalive_still_s = p['keepalive_still_s']
        self.keepalive_nudge_s = p['keepalive_nudge_s']
        self.keepalive_speed = p['keepalive_speed_mps']
        self.keepalive_lead_s = p['keepalive_min_lead_s']
        self.give_up_observe_s = p.get('give_up_observe_s', 0.0)
        self.corner_lever = s.get('corner_lever_m', 0.23)     # 车角离转动中心多远（扭车头时车角横向多出 = 这个 × sin 角度）
        self.max_corner = s.get('max_corner_m', 0.06)         # 扭 + 横移合起来，车角横向最多多出这么多（不出框）
        # ---- 观察 / 规划 ----
        self.observe_s = s.get('observe_s', 0.8)              # 至少有这么长的数据才规划（车动着拍的也算）
        self.prep_base_s = s.get('prep_base_s', 1.5)          # 准备时间 = prep_base_s + 偏差 / servo_px_s + hold_s
        self.servo_px_s = s.get('servo_px_s', 15.0)
        # 够不够得着（规划用）：扭 1 度、横移 1 米画面移多少像素。开始用理论值，每扭/移一下按实测更新（限理论值 0.5~1.6 倍）
        self.k_theory = math.degrees(1.0) * s.get('twist_px_per_deg', p.get('twist_px_per_deg', 8.8))
        self.k_px_rad = self.k_theory
        self.ppm_theory = s.get('lateral_px_per_m', 313.0)
        self.lat_px_m = self.ppm_theory
        # ---- 对准 ----
        self.tolerance = s.get('tolerance_px', 2.5)           # 对准到这以内就停
        self.release_px = s.get('release_px', 1.0)            # 预计停下来时离目标还差这么多就松开
        self.coarse_px = s.get('coarse_px', 10.0)             # 偏差大于这个扭车头，小于这个横移
        self.coarse_max_px = s.get('coarse_max_px', 14.0)     # 扭过头以后粗细分界最多放宽到这里（横移 5 cm 约 15 像素）
        self.yaw_gain = s.get('yaw_gain', 0.01)               # rad/s 每像素
        self.yaw_min = s.get('yaw_min_rate', 0.25)            # 垫子上原地转指令小于约 0.25 转不动
        self.yaw_max = s.get('yaw_max_rate', 0.4)
        self.yaw_coast = s.get('yaw_coast_s', 0.8)            # 松开后画面还会移 = 松开时画面速度 × 这么多秒（含识别延迟）
        self.lat_gain = s.get('lateral_gain', 0.004)          # m/s 每像素
        self.lat_min = s.get('lateral_min_speed', 0.03)
        self.lat_max = s.get('lateral_max_speed', 0.05)
        self.lat_coast = s.get('lateral_coast_s', 0.4)
        self.bump_yaw = s.get('yaw_bump', 0.05)               # 发了指令 stall_s 秒画面还不动：加大这么多
        self.bump_lat = s.get('lateral_bump', 0.01)
        self.start_s = s.get('start_s', 0.4)                  # 起步死区：松开后这么久内不判断停没停
        self.stall_s = s.get('stall_s', 1.0)
        self.max_move_s = s.get('max_move_s', 4.0)            # 一次扭/平移最多多久
        self.stable_px_s = s.get('stable_px_s', 3.0)          # 画面移动慢于这个算停稳
        self.hold_s = s.get('hold_s', 0.3)                    # 对准后停稳这么久才进入等待
        self.max_flips = int(s.get('max_flips', 3))           # 扭过头这么多次以后不再扭（只横移）
        self.max_moves = int(s.get('max_moves', 8))           # 一次对准最多动几下
        self.max_align_s = s.get('max_align_s', 8.0)
        self.hold_heading = s.get('hold_heading', True)       # 横移时稳车头
        self.hold_gain = s.get('hold_gain', 3.0)              # 稳车头：偏 1 度 → 回转 3 × 0.0175 rad/s（横移时车在动，小指令也转得动）
        # ---- 等待 ----
        self.realign_px = s.get('realign_px', 4.0)            # 等的时候偏差超过这个、离开火还早 → 再对一次
        self.realign_min_s = s.get('realign_min_s', 3.0)
        self.fire_max_rate = s.get('fire_max_rate_px_s', 8.0) # 画面还在移得比这快不开火
        self.late_s = s.get('late_s', 0.3)                    # 码过了交点这么久以内还能开火（miss 在容差内）
        self.tracker = WheelTracker(int(p['min_markers']), p['outlier_px'], math.radians(p['min_omega_deg_s']),
                                    s.get('window_s', 6.0))
        self.target = target_id
        self.half_period = 0.5 * control_period_s
        self.state = 'observe'
        self.side = None              # 选定的交点那一侧（left / right / top / bottom / static）
        self.yaw0 = None              # 这一步开始时的车头、横向位置（扭/平移上限从这里算）
        self.lateral0 = None
        self.yaw = None
        self.lateral = None
        self.move = None              # 对准中：None / 'yaw' / 'lat' / 'coast'
        self.move_t = None
        self.move_sign = 0.0          # 开始这一下时偏差的符号
        self.level = 0                # 加大了几次指令
        self.last_bump = None
        self.heading_ref = None       # 横移时要稳住的车头
        self.release = None           # 松开时的 (时刻, 偏差, 画面速度, 方式)，停稳后用来更新"滑多久"
        self.move_from = None         # 这一下开始时的 (偏差, 车头, 横向位置)，停稳后用来更新换算系数
        self.poses = collections.deque(maxlen=12)   # 最近的 (时刻, 车头, 横向位置)，算实际转速/横移速度
        self.release_t = None
        self.hold_since = None
        self.align_since = None
        self.aligned = False          # 这次对准是否停稳在容差内了
        self.best_effort = False      # 选的交点扭 + 横移都够不着（或已经顶到上限）：到点就开火，不再来回修
        self.moves = 0
        self.flips = 0
        self.aligns = 0
        self.misses = 0
        self.nudge_phase = None
        self.nudge_until = None
        self.model = None

    def set_heading_reference(self, yaw_ref):
        """车头"正对侧墙"时的朝向（和 update 的 yaw 同一坐标系）。扭的上限按它算（绝对朝向），不按这一步开始时的车头：
        车停歪了（2026-10-23 18:58 故意摆歪约 15 度，转轮圆心偏到 u≈216）也能扭回来，往回扭不受限、只限制往外扭。"""
        self.yaw0 = yaw_ref

    # ---------------- 主入口 ----------------
    def update(self, now, lateral, yaw, samples_by_id, still_for_s, deadline):
        """一个控制周期。lateral：横向位置（米，左为正）；yaw：车头朝向（弧度，左转为正）；拿不到为 None。"""
        if self.yaw0 is None and yaw is not None:
            self.yaw0 = yaw
        if self.lateral0 is None and lateral is not None:
            self.lateral0 = lateral
        self.yaw, self.lateral = yaw, lateral
        self.poses.append((now, yaw, lateral))
        if self.state == 'nudge':
            return self._nudge(now)
        model, why = self.tracker.update(samples_by_id, now)
        if model is not None and self.target not in model.phases:
            model, why = None, 'target_not_seen'
        if model is not None and now - self.tracker.latest_t > 0.5:
            model, why = None, 'stale'
        if model is None:
            if self.move in ('yaw', 'lat'):
                self._release(now, None, 0.0)          # 看不清就先停下
            return 0.0, 0.0, 0.0, False, self._info(reason=why)
        self.model = model
        if self.state == 'observe':
            if self.tracker.span_s < self.observe_s:
                return 0.0, 0.0, 0.0, False, self._info(reason='observing')
            return self._plan(now, model, deadline)
        if self.state == 'align':
            return self._align(now, model)
        return self._wait(now, model, still_for_s)

    # ---------------- 小工具 ----------------
    def _info(self, **extra):
        info = {'state': self.state}
        if self.side is not None:
            info['side'] = self.side
        if self.yaw is not None and self.yaw0 is not None:
            info['twisted_deg'] = math.degrees(wrap_angle(self.yaw - self.yaw0))
        if self.lateral is not None and self.lateral0 is not None:
            info['shifted_mm'] = 1000.0 * (self.lateral - self.lateral0)
        info.update(extra)
        return info

    def _points(self, model, now):
        """能开火的点：[{'side', 'u', 'angle', 'v_miss'}]（转轮不转时就是码现在的位置）。"""
        if model.static:
            u, v = model.position(self.target, now)
            return [{'side': 'static', 'u': u, 'angle': model.angle(self.target, now), 'v_miss': abs(v - self.aim_v)}]
        points = model.crossings(self.target, self.aim_v)
        if self.fire_side in ('left', 'right'):
            points = [c for c in points if c['side'] in (self.fire_side, 'top', 'bottom')] or points
        return points

    def _point(self, model, now):
        for c in self._points(model, now):
            if c['side'] == self.side:
                return c
        return None

    def _lead(self, model, point, now):
        """目标码转到这一点、减去开火延迟还要多久：-1 秒 ~ 一圈减 1 秒（负 = 刚过去不到 1 秒）。"""
        if model.static:
            return 0.0
        lead = model.time_until(self.target, point['angle'], now) - self.latency_s
        # 刚过去不到 late_s + 0.2 秒的算负数（还在开火窗口里 / 刚错过），再早的就是下一圈
        return lead - model.period if lead > model.period - (self.late_s + 0.2) else lead

    def _room(self, mode, direction, margin=0.0):
        """往 direction（指令符号）扭/平移还有没有余量（留 margin 给松开后还会滑的）；测不到车头/横向位置就不能动。往回走总是允许。

        单项上限：车头 max_twist_deg、横向 max_shift_m；合起来：车角横向多出 = |横移| + 车角半径 × sin|扭的角度| ≤ max_corner_m。
        """
        if mode == 'yaw':
            if self.yaw is None or self.yaw0 is None:
                return False
            offset, limit = wrap_angle(self.yaw - self.yaw0), self.max_twist
        else:
            if self.lateral is None or self.lateral0 is None:
                return False
            offset, limit = self.lateral - self.lateral0, self.max_shift
        growing = offset * direction > 0 or offset == 0.0
        if abs(offset) >= limit - margin and growing:
            return False
        if self.yaw is not None and self.yaw0 is not None and self.lateral is not None and self.lateral0 is not None:
            corner = abs(self.lateral - self.lateral0) + self.corner_lever * abs(math.sin(wrap_angle(self.yaw - self.yaw0)))
            margin_m = margin * self.corner_lever if mode == 'yaw' else margin
            if corner >= self.max_corner - margin_m and growing:
                return False
        return True

    def _reach_px(self, error):
        """从现在的车头、横向位置出发，往修正 error 的方向还能让画面移多少像素（扭 + 横移，按上限和换算系数估）。"""
        direction = -math.copysign(1.0, error)          # 指令符号
        reach = 0.0
        if self.yaw is not None and self.yaw0 is not None:
            offset = wrap_angle(self.yaw - self.yaw0)
            reach += max(0.0, self.max_twist - direction * offset) * self.k_px_rad
        if self.lateral is not None and self.lateral0 is not None:
            offset = self.lateral - self.lateral0
            # 车头扭到头时车角已经多出 corner_lever × sin(max_twist)，横移只剩 max_corner 减掉这些
            room = min(self.max_shift, self.max_corner - self.corner_lever * math.sin(self.max_twist))
            reach += max(0.0, room - direction * offset) * self.lat_px_m
        return reach

    # ---------------- 观察 → 选交点 ----------------
    def _plan(self, now, model, deadline):
        options = []
        points = self._points(model, now)
        if self.yaw is None and self.lateral is None:
            # 测不到车头和横向位置，车不能动：只选不动就打得到的点（都不行就尽力而为）
            points = [c for c in points if abs(c['u'] - self.aim_u) <= self.hit_tolerance] or points
        for c in points:
            error = c['u'] - self.aim_u
            need = (self.prep_base_s + abs(error) / self.servo_px_s if abs(error) > self.tolerance else 0.0) + self.hold_s
            wait = 0.0 if model.static else model.time_until(self.target, c['angle'], now)
            if not model.static and wait < need + self.latency_s:
                wait += model.period                    # 来不及，等下一圈
            excess = max(0.0, abs(error) - self.tolerance - self._reach_px(error))
            options.append((excess > 0.0, wait, abs(error), excess, c))
        info = self._info(omega_deg_s=math.degrees(model.omega), center='%.1f,%.1f' % (model.uc, model.vc),
                          radius=model.radii[self.target], k_px_deg=self.k_px_rad / math.degrees(1.0))
        if not options:
            return 0.0, 0.0, 0.0, False, dict(info, reason='no_crossing')
        # 够得着的里面哪边先到打哪边；都够不着（扭、横移都到上限）就挑差得最少的尽力而为
        reachable = [o for o in options if not o[0]]
        if reachable:
            _, wait, _, _, best = min(reachable, key=lambda o: (o[1], o[2]))
            self.best_effort = False
        else:
            _, wait, _, excess, best = min(options, key=lambda o: (o[3], o[1]))
            info['short_px'] = excess
            self.best_effort = True
        info.update({'side': best['side'], 'wait_s': wait, 'err_px': best['u'] - self.aim_u,
                     'reach_px': self._reach_px(best['u'] - self.aim_u)})
        if now + wait > deadline:
            if self.tracker.span_s < self.give_up_observe_s:
                return 0.0, 0.0, 0.0, False, dict(info, reason='confirm_give_up')
            return 0.0, 0.0, 0.0, False, dict(info, reason='wait_too_long', give_up=True)
        self.side = best['side']
        self._start_align(now)
        info.update({'state': 'align', 'reason': 'plan'})
        return 0.0, 0.0, 0.0, False, info

    def _start_align(self, now):
        self.state = 'align'
        self.aligned = False
        self.align_since = now
        self.hold_since = None
        self.move = None
        self.moves = 0
        self.aligns += 1

    # ---------------- 对准 ----------------
    def _align(self, now, model):
        point = self._point(model, now)
        if point is None:                                # 这一侧的交点没了（不该发生）：重新选
            self.state = 'observe'
            return 0.0, 0.0, 0.0, False, self._info(reason='lost_side')
        error = point['u'] - self.aim_u                  # 正 = 交点在弹着点右边 → 画面要左移（右转 / 右移）
        rate = self.tracker.center_rate()
        lead = self._lead(model, point, now)
        info = self._info(err_px=error, rate_px_s=rate, lead_s=lead)
        if not model.static and -self.late_s <= lead <= self.half_period:
            # 码已经到了：不管对没对准都交给"等"判断（对准了开火，没对准改打另一侧）
            if self.move in ('yaw', 'lat'):
                self._release(now, None, rate)
            self.state = 'wait'
            return 0.0, 0.0, 0.0, False, dict(info, state='wait', reason='arrived')
        if now - self.align_since > self.max_align_s or self.moves > self.max_moves:
            if self.move in ('yaw', 'lat'):
                self._release(now, None, rate)
            self.state = 'wait'
            return 0.0, 0.0, 0.0, False, dict(info, state='wait', reason='align_timeout')
        if self.move in ('yaw', 'lat'):
            command = self._drive(now, error, rate, info)
            if command is not None:
                return command
        if self.move == 'coast':
            if now - self.release_t < self.start_s or abs(rate) >= self.stable_px_s:
                return 0.0, 0.0, 0.0, False, dict(info, reason='coasting')
            self._learn(error)
            self.move = None
        if abs(error) <= self.tolerance:
            if self.hold_since is None or abs(rate) >= self.stable_px_s:
                self.hold_since = now
            if now - self.hold_since >= self.hold_s:
                self.state = 'wait'
                self.aligned = True
                return 0.0, 0.0, 0.0, False, dict(info, state='wait', reason='aligned')
            return 0.0, 0.0, 0.0, False, dict(info, reason='holding')
        self.hold_since = None
        return self._start_move(now, error, rate, info)

    def _start_move(self, now, error, rate, info):
        direction = -math.copysign(1.0, error)          # 指令符号（扭、平移都是这个方向让偏差变小）
        coarse = min(self.coarse_px * (1 + self.flips), max(self.coarse_px, self.coarse_max_px))
        can_yaw = self._room('yaw', direction) and self.flips < self.max_flips
        can_lat = self._room('lat', direction)
        if abs(error) > coarse and can_yaw:
            mode = 'yaw'
        elif can_lat:
            mode = 'lat'
        elif can_yaw:
            mode = 'yaw'                                 # 横移到头了（或测不到横向位置）：小偏差也只能扭
        else:
            # 都到上限了：按现在的位置等，到点尽力开火（不再来回"对准 / 到上限"）
            self.state = 'wait'
            self.best_effort = True
            return 0.0, 0.0, 0.0, False, dict(info, state='wait', reason='at_limit')
        self.move = mode
        self.move_t = now
        self.move_sign = math.copysign(1.0, error)
        self.level = 0
        self.last_bump = now
        self.moves += 1
        self.move_from = (error, self.yaw, self.lateral, mode)
        self.heading_ref = self.yaw if mode == 'lat' else None
        command = self._drive(now, error, rate, info)
        if command is not None:
            return command
        return 0.0, 0.0, 0.0, False, dict(info, reason='coasting')

    def _drive(self, now, error, rate, info):
        """正在扭/平移：返回这一周期的指令；该松开了返回 None（进入 coast）。"""
        mode = self.move
        coast = self.yaw_coast if mode == 'yaw' else self.lat_coast
        predicted = error + rate * coast                 # 现在松开，停下来时的偏差
        elapsed = now - self.move_t
        reason = None
        if self.move_sign * predicted <= self.release_px:
            reason = 'release'
        elif not self._room(mode, -self.move_sign, self._coast_margin(mode, elapsed)) and elapsed > 0.1:
            reason = 'limit'
            info['limit'] = mode
        elif elapsed > self.max_move_s:
            reason = 'move_timeout'
        if reason is not None:
            self._release(now, error, rate)
            return None
        # 发了 stall_s 秒画面还没动（地面阻力大）：加大指令
        if (elapsed > self.start_s + self.stall_s and abs(rate) < self.stable_px_s
                and now - self.last_bump > self.stall_s):
            self.level += 1
            self.last_bump = now
        info['predicted_px'] = predicted
        if mode == 'yaw':
            speed = max(self.yaw_min, min(self.yaw_max, self.yaw_gain * abs(predicted))) + self.level * self.bump_yaw
            speed = min(speed, self.yaw_max + 3 * self.bump_yaw)
            return 0.0, 0.0, -self.move_sign * speed, False, dict(info, reason='twist', coast_s=coast)
        speed = max(self.lat_min, min(self.lat_max, self.lat_gain * abs(predicted))) + self.level * self.bump_lat
        speed = min(speed, self.lat_max + 3 * self.bump_lat)
        speed = math.floor(speed * 100.0 + 1e-6) / 100.0 + 0.0004     # 底盘驱动按整数 cm/s 截断：发整数 cm/s
        wz = 0.0
        if self.hold_heading and self.heading_ref is not None and self.yaw is not None:
            wz = hold_heading(wrap_angle(self.yaw - self.heading_ref), gain=self.hold_gain,
                              deadband_rad=math.radians(0.15))
        return 0.0, -self.move_sign * speed, wz, False, dict(info, reason='shift', coast_s=coast)

    def _pose_rate(self, index, span_s=0.25):
        """最近 span_s 秒车头（index=1，rad/s）或横向位置（index=2，m/s）的实际变化速度；拿不到返回 0。"""
        pts = [(t, x[index]) for t, x in ((q[0], q) for q in self.poses) if x[index] is not None]
        if len(pts) < 2:
            return 0.0
        t_last, v_last = pts[-1]
        older = [q for q in pts if q[0] <= t_last - span_s] or pts[:1]
        t0, v0 = older[-1]
        if t_last - t0 < 0.05:
            return 0.0
        dv = wrap_angle(v_last - v0) if index == 1 else v_last - v0
        return dv / (t_last - t0)

    def _coast_margin(self, mode, elapsed):
        """松开后还会滑多少（弧度或米）= 实际转速/横移速度（里程计）× 滑行时间，留出来不超上限。

        2026-10-23 实车：以前按"最大指令 × 比例 × 滑行时间"估，留了 5.4 度（上限才 6 度），扭过 0.6 度以后每次一扭就判"到上限"、
        车根本没动就松开，车头卡在 -3.8 度够不着右交点，码转到了也不开火、只能等另一侧（17 / 22 / 43 秒的三次都是这个）。
        """
        # 松开后底盘死区里还在按原指令加速，滑的比"现在速度 × 滑行时间"多一点：多算 0.3 秒，再加固定余量
        # （2026-10-23 18:58 车摆得很偏、一口气扭 6 度时冲到了 8.3 度：起步那一下速度量得偏小）
        if mode == 'yaw':
            return abs(self._pose_rate(1)) * (self.yaw_coast + 0.3) + math.radians(0.8)
        return abs(self._pose_rate(2)) * (self.lat_coast + 0.3) + 0.005

    def _release(self, now, error, rate):
        if error is not None:
            self.release = (now, error, rate, self.move)
        else:
            self.release = None
        self.release_t = now
        self.move = 'coast'
        self.hold_since = None

    def _learn(self, error_now):
        """停稳后更新：
          滑多久 = 松开那一刻到现在偏差变了多少 / 松开时的画面速度（只用松开时画面确实在动、之后确实滑了一段的）；
          换算系数 = 这一下偏差变了多少 / 车头（或横向位置）实际变了多少（规划时判断够不够得着用）。扭过头记一次。
        """
        moved_from, self.move_from = self.move_from, None
        if moved_from is not None:
            error_a, yaw_a, lat_a, mode_a = moved_from
            if mode_a == 'yaw' and yaw_a is not None and self.yaw is not None:
                turned = wrap_angle(self.yaw - yaw_a)
                if abs(turned) >= math.radians(1.0):
                    k = (error_now - error_a) / turned
                    k = max(0.5 * self.k_theory, min(1.6 * self.k_theory, k))
                    self.k_px_rad = 0.5 * self.k_px_rad + 0.5 * k
            elif mode_a == 'lat' and lat_a is not None and self.lateral is not None:
                moved = self.lateral - lat_a
                if abs(moved) >= 0.008:
                    turned = wrap_angle(self.yaw - yaw_a) if (yaw_a is not None and self.yaw is not None) else 0.0
                    ppm = (error_now - error_a - self.k_px_rad * turned) / moved   # 扣掉横移时车头偏的那部分
                    ppm = max(0.5 * self.ppm_theory, min(1.6 * self.ppm_theory, ppm))
                    self.lat_px_m = 0.5 * self.lat_px_m + 0.5 * ppm
        if self.release is None:
            return
        _, error0, rate0, mode = self.release
        self.release = None
        if mode == 'yaw' and error_now * error0 < 0 and abs(error_now) > self.tolerance:
            self.flips += 1
        if abs(rate0) < 10.0 or abs(error_now - error0) < 3.0:
            return                                       # 松开时画面几乎没动、或之后没怎么滑：除出来是噪声
        measured = max(0.2, min(1.5, (error_now - error0) / rate0))
        if mode == 'yaw':
            self.yaw_coast = 0.5 * self.yaw_coast + 0.5 * measured
        else:
            self.lat_coast = 0.5 * self.lat_coast + 0.5 * measured

    # ---------------- 等 ----------------
    def _wait(self, now, model, still_for_s):
        point = self._point(model, now)
        if point is None:
            self.state = 'observe'
            return 0.0, 0.0, 0.0, False, self._info(reason='lost_side')
        error = point['u'] - self.aim_u
        rate = self.tracker.center_rate()
        lead = self._lead(model, point, now)
        pred_u, pred_v = model.position(self.target, now + self.latency_s)
        pred_u += rate * self.latency_s                  # 画面还在移的话，弹丸到的时候转轮又移了一点
        miss = math.hypot(pred_u - self.aim_u, pred_v - self.aim_v)
        # 对准过（停稳在容差内）的，等的时候稍微漂一点（≤ wait_keep_px）照样开火；码到了还没对准完的，偏差 ≤ hit_tolerance_px 才开火
        tolerance = (self.keep_px if self.aligned else self.hit_tolerance) + point.get('v_miss', 0.0)
        if self.best_effort:
            tolerance = max(tolerance, abs(error) + point.get('v_miss', 0.0) + 2.0)   # 够不着：到点照样开一枪
        info = self._info(err_px=error, miss_px=miss, lead_s=lead, rate_px_s=rate)
        # 开火窗口：码到交点前半个周期 ~ 过了 late_s 秒（交点处码竖直方向约 4 像素/秒，晚 0.3 秒约差 1 像素，miss 里已经算上）。
        # 只给 1 个周期的窗口会被拟合抖动跳过去（相位抖 0.01 弧度 ≈ 0.07 秒），白等半圈
        if -self.late_s <= lead <= self.half_period:
            if miss <= tolerance and abs(rate) <= self.fire_max_rate:
                # 记下这次学到的值，调参用：松开后滑多久（扭/横移）、扭过头几次、对准了几轮、放过几次交点
                if self.best_effort:
                    info['best_effort'] = True
                return 0.0, 0.0, 0.0, True, dict(info, reason='fire', yaw_coast_s=self.yaw_coast,
                                                 lat_coast_s=self.lat_coast, flips=self.flips, aligns=self.aligns,
                                                 misses=self.misses, k_px_deg=self.k_px_rad / math.degrees(1.0),
                                                 lat_px_cm=self.lat_px_m / 100.0)
            if model.static:
                self._start_align(now)                   # 转轮不转：没对准就接着对
                return 0.0, 0.0, 0.0, False, dict(info, state='align', reason='realign')
        if lead < -self.late_s:
            # 码转过去了还没开火（没对准）：不白打，重新选交点（一般是另一侧，约半圈后）
            self.misses += 1
            self.state = 'observe'
            return 0.0, 0.0, 0.0, False, dict(info, state='observe', reason='missed')
        if abs(error) > self.realign_px and lead > self.realign_min_s and not self.best_effort:
            self._start_align(now)
            return 0.0, 0.0, 0.0, False, dict(info, state='align', reason='realign')
        if (still_for_s >= self.keepalive_still_s and lead >= self.keepalive_lead_s
                and self.keepalive_nudge_s > 0):
            self.state = 'nudge'
            self.nudge_phase = 'back'
            self.nudge_until = now + self.keepalive_nudge_s
            return -self.keepalive_speed, 0.0, 0.0, False, dict(info, state='nudge', reason='keepalive')
        return 0.0, 0.0, 0.0, False, dict(info, reason='waiting')

    def _nudge(self, now):
        """keepalive 前后挪（只改距离，转轮在画面里几乎不动），回来后接着对准（画面一直在看，不用重新观察）。"""
        info = {'state': 'nudge', 'phase': self.nudge_phase}
        if now < self.nudge_until:
            sign = -1.0 if self.nudge_phase == 'back' else 1.0
            return sign * self.keepalive_speed, 0.0, 0.0, False, info
        if self.nudge_phase == 'back':
            self.nudge_phase = 'forward'
            self.nudge_until = now + self.keepalive_nudge_s
            return self.keepalive_speed, 0.0, 0.0, False, info
        self._start_align(now)
        self.move = 'coast'                              # 先等停稳再判断偏差
        self.release = None
        self.release_t = now
        info.update({'state': 'align', 'reason': 'nudge_done'})
        return 0.0, 0.0, 0.0, False, info
