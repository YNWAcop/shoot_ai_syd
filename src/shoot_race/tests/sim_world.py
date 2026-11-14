# -*- coding: utf-8 -*-
"""离线仿真：一个简化的 2026 场地 + 车辆 + 传感器，用来检查程序逻辑。

!!! 重要：这里的场地尺寸、靶子运动（转速、移动速度）、系统延迟都是假设值，
!!! 和真实场地/真实靶子无关。仿真只用来检查：
!!!   流程能否走完、会不会卡住、正负号对不对、预测开火时机的计算是否自洽、总用时是否在预算内。
!!! 不能用来估计真实命中率，也不能用来定参数。

SimIO 实现了和 shoot_race.ros_io.RosIO 相同的接口，主程序代码不需要任何修改就能在仿真里跑。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import collections
import math

import numpy as np

# ------------------------------------------------------------
# 假设的场地（地图坐标，米）。车头朝 +x 为"朝向靶子"，+y 为场地左侧。
# ------------------------------------------------------------
FENCE = (-0.30, 3.30, -3.30, 0.30)          # x_min, x_max, y_min, y_max（3.6 m × 3.6 m）
PARTITIONS = [((0.55, -0.95), (1.75, -0.95)),  # 任务点之间的挡板（长 1.2 m）
              ((0.55, -2.15), (1.75, -2.15))]
START_POSE = (-0.10, 0.10, 0.0)
BOX_CENTERS = {'p1': (1.05, -0.33), 'p2': (1.05, -1.55), 'p3': (1.05, -2.73), 'end': (0.10, -3.10)}
RECOG_POSE = (0.50, -0.12, math.pi / 2)      # 识别区：车头朝左墙

LASER_OFFSET_X = 0.06
# 激光扫到的车自己的零件（激光坐标：角度 度, 距离 米），/scan_filtered 里每帧都有。
# 2026-10-20 17:57 / 16:19 / 10-19 22:19 三个录包：-165 度 0.18 米（车尾右侧，陷进车身轮廓 6.6 cm）、158 度 0.22 米。
# 原来仿真把 0.16 米以内全去掉、也没有这些点，新安全检查把它们当障碍的问题仿真里看不出来
SELF_POINTS = [(-165.5, 0.181), (-165.0, 0.181), (-164.5, 0.180), (-164.0, 0.180), (157.5, 0.218), (158.0, 0.219)]
CAMERA_FX = 671.0
CAMERA_CX = 332.7

# AR 换算像素用的相机内参（与 system.yaml camera 一致）
AR_FX, AR_FY, AR_CX, AR_CY = 671.39, 668.23, 332.72, 274.54

# 靶子。转轮、移动靶的运动参数按 2026-10-16 在 2025 场地录的包设定（rotating.bag / moving2.bag），
# 几何位置是假设的（今年场地未知）：
# 环形靶：find_object 的像素模型按 2026-10-16 录包实测（shoot_ring_*.bag）：
#   约 2.6 帧/秒、延迟 0.1~0.4 秒、静止时单帧噪声标准差约 3 像素（10-19 新靶 3.6）；
#   车平移时像素只变化理论值的 0.55 倍（10-19 新靶 0.71，RING_IMAGE_GAIN）。
# 识别像素 = RING_AIM_PX + RING_IMAGE_GAIN × (真实像素 - GUN_RING_PIXEL) + 噪声，
# 即"靶心正好在枪线上"时识别像素等于标定好的 aim_x_px。
GUN_RING_PIXEL = 370.0                       # 枪线在真实光学像素里的位置（仿真内部用，任意值）
RING_AIM_PX = 363.0                          # 与 shooting.yaml ring.aim_x_px 一致（test_mission_sim 会检查）
RING_IMAGE_GAIN = 0.70                       # 2026-10-19 现在的靶实测 0.71（10-16 旧场地 0.55）
RING_NOISE_PX = 3.6
# 环形靶放在"车在框中心时识别偏差 +8 像素"（真实约 4 cm）的位置：要先平移一次再开火
RING_INITIAL_ERROR_PX = 8.0
RING_POS = (BOX_CENTERS['p1'][0] + 1.90,
            BOX_CENTERS['p1'][1] - (GUN_RING_PIXEL + RING_INITIAL_ERROR_PX / RING_IMAGE_GAIN - CAMERA_CX)
            / CAMERA_FX * 1.90)
WHEEL_CENTER_DEPTH = BOX_CENTERS['p2'][0] + 1.50
WHEEL_RADIUS = 0.064                         # 实测约 29 像素 @1.5 m
WHEEL_CENTER_RIGHT = 0.046                   # 转轮中心在相机右侧多少米（车在框中心时）。实测圆心 u≈353
WHEEL_CENTER_Y_CAM = -0.179                  # 转轮中心在相机坐标中的高度。实测圆心 v≈195
WHEEL_PERIOD_S = 43.0                        # 实测 43 秒一圈（画面里顺时针）
WHEEL_PHASE0 = 0.0                           # 初始相位（弧度），测试不同等待时间用
# 弹着点用 shooting.yaml 的值 (366, 214)，在圆心右下方：右交点 u≈375 要车右移约 2 cm，左交点要左移约 8 cm（超出上限）
RAIL_DEPTH = BOX_CENTERS['p3'][0] + 1.50
RAIL_AMPLITUDE = 0.135                       # 实测行程约 121 像素 ≈ 27 cm
RAIL_PERIOD_S = 13.6                         # 实测单程约 6.8 秒（速度约 4 cm/s），端点立即换向
RAIL_CENTER_RIGHT = 0.02                     # 滑块中点在相机右侧多少米（假设）
RAIL_Y_CAM = -0.183                          # 实测码 v≈193
TRUE_LATENCY_S = 0.15                        # 假设的真实延迟（与 shooting.yaml 的 latency_s 相同）
RING_RATE_HZ = 2.6                           # 实测 find_object 约 2.6 帧/秒
RING_DELAY_S = 0.3                           # find_object 结果比拍摄时刻晚约 0.3 秒
# 找同心圆（ring_finder.py，shooting.yaml ring.detector = rings）：像素和 find_object 不通用，弹着点单独标定。
# 2026-10-22 车不动时同一组 0.1~0.2 像素；按每秒 5 帧、处理 0.15 秒（车上每帧几十到一两百毫秒）、噪声 0.3 像素模拟。
# 像素随车平移的变化和 find_object 一样按 RING_IMAGE_GAIN（相机真实焦距约 470 像素 = 671 × 0.70）。
RINGS_AIM_PX = 350.5                         # 与 shooting.yaml ring.rings.aim_x_px 一致（test_mission_sim 会检查）
RINGS_RATE_HZ = 5.0
RINGS_DELAY_S = 0.15
RINGS_NOISE_PX = 0.3
AR_RATE_HZ = 25.0                            # 实测每个码 15~25 Hz
AR_DELAY_S = 0.04                            # 实测 AR 识别延迟约 0.036 秒
AR_NOISE_M = 0.0008                          # 位置噪声（米，1.5 m 处约 0.4 像素）
AR_MISREAD = 0.02                            # 2% 的检测把码号认成旁边的码（实测会发生）


def wall_segments():
    x0, x1, y0, y1 = FENCE
    segments = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    return segments + PARTITIONS


def raycast(origin, angles, segments, max_range=8.0):
    """从 origin 沿 angles（世界坐标角度）发射射线，返回到最近墙段的距离。"""
    ox, oy = origin
    dx = np.cos(angles)
    dy = np.sin(angles)
    best = np.full(len(angles), np.inf)
    for (ax, ay), (bx, by) in segments:
        ex, ey = bx - ax, by - ay
        denom = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((ax - ox) * ey - (ay - oy) * ex) / denom     # 射线参数
            u = ((ax - ox) * dy - (ay - oy) * dx) / denom     # 线段参数
        hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    best[best > max_range] = np.inf
    return best


SIM_DEAD_TIME = 0.35
SIM_TAU = 0.30
SIM_GAIN_FORWARD = 0.75
SIM_GAIN_BACKWARD = 0.90
SIM_GAIN_LATERAL = 0.80
SIM_GAIN_YAW = 0.90
SIM_GAIN_YAW_INPLACE = 0.40     # 原地转弱得多（2026-10-16 原地转速测试）


class SimIO(object):
    """仿真版 RosIO。"""

    def __init__(self, card_image, dt=0.05, seed=0, hide_ring=False, hide_markers=()):
        self.hide_ring = hide_ring            # True：模拟环形靶一直识别不到
        self.hide_markers = set(hide_markers)  # 模拟这些编号的 AR 码一直识别不到
        self.dt = dt
        self.t = 100.0
        self.rng = np.random.RandomState(seed)
        self.pose = list(START_POSE)          # 真实位姿
        self.velocity = [0.0, 0.0, 0.0]       # 真实车体速度
        self.command = [0.0, 0.0, 0.0]        # 收到的速度指令
        self.command_history = collections.deque()
        self.segments = wall_segments()
        self.card_image = card_image
        self.ar_buffer = collections.defaultdict(lambda: collections.deque(maxlen=600))
        self.ring_buffer = collections.deque(maxlen=60)
        self.ring_pending = collections.deque()   # 已拍摄、还没"处理完"的环形靶检测结果
        self.rings_buffer = collections.deque(maxlen=120)  # 找同心圆的结果 (可用时刻, 拍摄时刻, 像素)
        self.next_rings_frame = self.t
        self.next_ar_frame = self.t
        self.next_ring_frame = self.t
        self.collisions = 0
        self.max_speed_seen = 0.0
        self.move_base_client = None
        self.log_pose = []

    # ---------- 时间 ----------
    def now(self):
        return self.t

    def is_shutdown(self):
        return False

    def wait_next_tick(self):
        self._advance(self.dt)

    def sleep(self, seconds):
        end = self.t + seconds
        while self.t < end - 1e-9:
            self._advance(min(self.dt, end - self.t))

    def _advance(self, dt):
        """推进仿真：车辆运动（一阶惯性）+ 产生传感器数据。"""
        # 底盘响应（2026-10-17 全流程录包实测）：死区约 0.35 秒 + 一阶惯性 0.3 秒，
        # 稳态只到指令的一部分（前进 0.75、后退 0.9、横移 0.8、边走边转 0.9、原地转 0.4）。故意和程序里的模型参数不完全一样。
        self.command_history.append((self.t, list(self.command)))
        delayed = [0.0, 0.0, 0.0]
        for t_cmd, cmd in self.command_history:
            if t_cmd <= self.t - SIM_DEAD_TIME:
                delayed = cmd
        while len(self.command_history) > 2 and self.command_history[1][0] <= self.t - SIM_DEAD_TIME:
            self.command_history.popleft()
        in_place = abs(delayed[0]) + abs(delayed[1]) < 1e-3
        gains = [SIM_GAIN_FORWARD if delayed[0] >= 0 else SIM_GAIN_BACKWARD, SIM_GAIN_LATERAL,
                 SIM_GAIN_YAW_INPLACE if in_place else SIM_GAIN_YAW]
        for i in range(3):
            self.velocity[i] += (gains[i] * delayed[i] - self.velocity[i]) * min(1.0, dt / SIM_TAU)
        x, y, yaw = self.pose
        vx, vy, wz = self.velocity
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw = math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt))
        self.pose = [x, y, yaw]
        self.t += dt
        self.max_speed_seen = max(self.max_speed_seen, math.hypot(vx, vy))
        if self._robot_hits_wall():
            self.collisions += 1
        self._produce_camera_data()
        self.log_pose.append((self.t, x, y, yaw))

    def _robot_hits_wall(self, half_length=0.17, half_width=0.15):
        x, y, yaw = self.pose
        for (ax, ay), (bx, by) in self.segments:
            n = int(max(2, math.hypot(bx - ax, by - ay) / 0.01))
            px = np.linspace(ax, bx, n) - x
            py = np.linspace(ay, by, n) - y
            fx = px * math.cos(yaw) + py * math.sin(yaw)
            fy = -px * math.sin(yaw) + py * math.cos(yaw)
            if ((np.abs(fx) < half_length) & (np.abs(fy) < half_width)).any():
                return True
        return False

    # ---------- 相机类数据：环形靶、AR 码 ----------
    def ring_true_pixel(self):
        """环形靶靶心在当前位姿下的真实光学像素（和 GUN_RING_PIXEL 比较就是弹着偏差）。"""
        forward, left = self._world_to_robot(*RING_POS)
        return CAMERA_CX - CAMERA_FX * left / forward

    def ring_miss_cm(self, pose):
        """车在 pose 时开火，弹着点离环形靶靶心多少厘米（右为正）。"""
        saved = self.pose
        self.pose = list(pose)
        forward, _ = self._world_to_robot(*RING_POS)
        miss = (GUN_RING_PIXEL - self.ring_true_pixel()) / CAMERA_FX * forward * 100.0
        self.pose = saved
        return miss

    def _world_to_robot(self, wx, wy):
        x, y, yaw = self.pose
        dx, dy = wx - x, wy - y
        return dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)

    def rotating_marker_world(self, marker_id, t):
        """转轮上第 marker_id 个码在时刻 t 的位置：(深度方向 x, 横向 y, 相机坐标高度 y_cam)。

        角度 θ 按画面坐标（u 向右、v 向下）定义，θ 增大 = 画面里顺时针。
        """
        angle = WHEEL_PHASE0 + 2 * math.pi * t / WHEEL_PERIOD_S + 2 * math.pi * (marker_id - 1) / 5.0
        center_lateral = BOX_CENTERS['p2'][1] - WHEEL_CENTER_RIGHT      # 相机右侧 = 世界 -y
        lateral = center_lateral - WHEEL_RADIUS * math.cos(angle)
        height_cam = WHEEL_CENTER_Y_CAM + WHEEL_RADIUS * math.sin(angle)
        return WHEEL_CENTER_DEPTH, lateral, height_cam

    def moving_marker_world(self, marker_id, t):
        """移动靶：滑块三角波往返（端点立即换向），三个码间距 6 cm。"""
        phase = (t / RAIL_PERIOD_S) % 1.0
        triangle = 4 * phase - 1 if phase < 0.5 else 3 - 4 * phase      # -1 → 1 → -1
        right = RAIL_CENTER_RIGHT + RAIL_AMPLITUDE * triangle + (marker_id - 7) * 0.06
        return RAIL_DEPTH, BOX_CENTERS['p3'][1] - right, RAIL_Y_CAM

    def marker_camera_xy(self, world):
        depth_x, lateral_y, height_cam = world
        forward, left = self._world_to_robot(depth_x, lateral_y)
        return -left, height_cam, forward       # 相机 x 向右 = 机器人 -y

    def _produce_camera_data(self):
        # AR：图像时间戳 = 拍摄时刻
        while self.t >= self.next_ar_frame:
            stamp = self.next_ar_frame
            self.next_ar_frame += 1.0 / AR_RATE_HZ
            for marker_id in range(1, 9):
                world = (self.rotating_marker_world(marker_id, stamp) if marker_id <= 5
                         else self.moving_marker_world(marker_id, stamp))
                cx, cy, cz = self.marker_camera_xy(world)
                if marker_id in self.hide_markers:
                    continue
                if cz > 0.3 and abs(cx) < 0.6 * cz:          # 在视野内
                    reported = marker_id
                    if self.rng.rand() < AR_MISREAD:            # 误识别成旁边的码
                        group = range(1, 6) if marker_id <= 5 else range(6, 9)
                        reported = int(self.rng.choice([k for k in group if k != marker_id]))
                    self.ar_buffer[reported].append(
                        (stamp, cx + self.rng.normal(0, AR_NOISE_M), cy + self.rng.normal(0, AR_NOISE_M), cz))
        # 环形靶：按拍摄时刻的位姿算像素，RING_DELAY_S 秒后才"收到"（模拟 find_object 的处理延迟）
        while self.t >= self.next_ring_frame:
            self.next_ring_frame += 1.0 / RING_RATE_HZ
            forward, left = self._world_to_robot(*RING_POS)
            if forward > 0.5 and abs(left) < 0.5 * forward and not self.hide_ring:
                pixel = (RING_AIM_PX + RING_IMAGE_GAIN * (self.ring_true_pixel() - GUN_RING_PIXEL)
                         + self.rng.normal(0, RING_NOISE_PX))
                obj_id = 31 if self.rng.rand() > 0.2 else 30      # 偶尔匹配成另一张模板
            else:
                pixel, obj_id = 0.0, 255
            self.ring_pending.append((self.t + RING_DELAY_S, pixel, obj_id))
        while self.ring_pending and self.ring_pending[0][0] <= self.t:
            self.ring_buffer.append(self.ring_pending.popleft())
        # 找同心圆：按拍摄时刻的位姿算像素，带拍摄时间戳，处理完（RINGS_DELAY_S）才能用
        while self.t >= self.next_rings_frame:
            self.next_rings_frame += 1.0 / RINGS_RATE_HZ
            forward, left = self._world_to_robot(*RING_POS)
            if forward > 0.5 and abs(left) < 0.5 * forward and not self.hide_ring:
                pixel = (RINGS_AIM_PX + RING_IMAGE_GAIN * (self.ring_true_pixel() - GUN_RING_PIXEL)
                         + self.rng.normal(0, RINGS_NOISE_PX))
                self.rings_buffer.append((self.t + RINGS_DELAY_S, self.t, pixel))

    # ---------- RosIO 接口 ----------
    def topic_age(self, key):
        return 0.05

    def scan_points(self):
        x, y, yaw = self.pose
        laser = (x + LASER_OFFSET_X * math.cos(yaw), y + LASER_OFFSET_X * math.sin(yaw))
        angles = -math.pi + np.arange(720) * (2 * math.pi / 720)
        ranges = raycast(laser, angles + yaw, self.segments)
        ranges = ranges + self.rng.normal(0, 0.005, len(ranges))
        valid = np.isfinite(ranges) & (ranges > 0.16)       # /scan_filtered 已去掉车身附近的点
        angles, ranges = angles[valid], ranges[valid]
        if SELF_POINTS:
            angles = np.concatenate([angles, np.radians([a for a, r in SELF_POINTS])])
            ranges = np.concatenate([ranges, [r for a, r in SELF_POINTS]])
        return angles, ranges * np.cos(angles), ranges * np.sin(angles)

    def robot_pose(self):
        x, y, yaw = self.pose
        # 位置噪声按 AMCL 量级；朝向来自轮速计+IMU 融合，短时间内很平稳，噪声取很小
        return (x + self.rng.normal(0, 0.01), y + self.rng.normal(0, 0.01), yaw + self.rng.normal(0, 0.0005))

    def odom_pose(self):
        # 仿真里里程计没有漂移，就是真值加一点噪声（位置 0.3 mm，朝向 0.03 度）
        x, y, yaw = self.pose
        return (x + self.rng.normal(0, 0.0003), y + self.rng.normal(0, 0.0003), yaw + self.rng.normal(0, 0.0005))

    def odom_yaw(self):
        return self.odom_pose()[2]

    def wheel_speed(self):
        return tuple(self.velocity)

    def ring_observations(self, max_age, accept_ids):
        return [(t, px) for (t, px, obj) in self.ring_buffer if t >= self.t - max_age and obj in accept_ids]

    def ring_finder_observations(self, max_age):
        """仿真的"找同心圆"结果 [(拍摄时刻, u)]（ring_finder.RingSource 发现有这个方法就直接用，不渲染画面）。"""
        return [(shot, px) for (ready, shot, px) in self.rings_buffer if ready <= self.t and shot >= self.t - max_age]

    def ar_pixels(self, marker_ids, window_s):
        # 模拟识别处理延迟：只有拍摄时刻早于 now-AR_DELAY_S 的帧才"已经收到"
        result = {}
        for marker_id in marker_ids:
            result[marker_id] = [(t, AR_FX * x / z + AR_CX, AR_FY * y / z + AR_CY)
                                 for (t, x, y, z) in self.ar_buffer[marker_id]
                                 if self.t - window_s <= t <= self.t - AR_DELAY_S]
        return result

    @staticmethod
    def camera_to_pixel(cam):
        x, y, z = cam
        return AR_FX * x / z + AR_CX, AR_FY * y / z + AR_CY

    def camera_frames(self, since):
        return [(self.t, self.card_image)]

    def send_velocity(self, command):
        self.command = [command.vx, command.vy, command.wz]

    def set_initial_pose(self, x, y, yaw, std_xy=0.05, std_yaw=0.05):
        pass

    def move_base_ready(self, timeout_s):
        return False


class SimShooter(object):
    def __init__(self, io):
        self.io = io
        self.port = 'sim'
        self.fire_times = []

    def is_ready(self):
        return True

    def fire_burst(self, burst_s):
        self.fire_times.append(self.io.now())
        return True

    def safe_stop(self):
        pass


class SimVoice(object):
    """每段音频假设播放 1.5 秒；记录播放时车是否静止。"""

    def __init__(self, io):
        self.io = io
        self.end_time = 0.0
        self.played = []

    def missing_files(self, names):
        return []

    def start(self, files):
        speed = max(abs(v) for v in self.io.velocity)
        self.played.append((self.io.now(), list(files), speed))
        self.end_time = self.io.now() + 1.5 * len(files)

    def is_playing(self):
        return self.io.now() < self.end_time

    def stop(self):
        self.end_time = 0.0
