# -*- coding: utf-8 -*-
"""纯几何计算，不依赖 ROS，可以单独测试。

坐标约定（和 ROS 一致）：
    机器人坐标系 base：x 朝前，y 朝左，z 朝上；角度逆时针为正。
    地图坐标系 map：AMCL 定位输出的坐标系。
    激光数据：ranges[i] 对应角度 angle_min + i * angle_increment，
              0 度朝前、+90 度朝左、±180 度朝后（rplidar_ros 的默认输出就是这样）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import math

import numpy as np

# 四个方向墙面在机器人坐标系中的"理想方位角"（墙在机器人哪一侧）
SIDE_ANGLE = {
    'front': 0.0,
    'left': math.pi / 2.0,
    'back': math.pi,
    'right': -math.pi / 2.0,
}

# 机器人朝某侧墙移动时的单位方向（机器人坐标系），贴墙精定位时使用
SIDE_DIRECTION = {
    'front': (1.0, 0.0),
    'left': (0.0, 1.0),
    'back': (-1.0, 0.0),
    'right': (0.0, -1.0),
}


def wrap_angle(angle):
    """把角度规整到 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    """把数值限制在 [low, high] 之间。"""
    return max(low, min(high, value))


def is_finite(value):
    """判断是否为有限数（Python2 没有 math.isfinite）。"""
    return not (math.isnan(value) or math.isinf(value))


def map_to_base(vx_map, vy_map, robot_yaw):
    """把地图坐标系下的速度向量转换到机器人坐标系。

    robot_yaw：机器人在地图中的朝向（弧度）。
    """
    cos_y = math.cos(robot_yaw)
    sin_y = math.sin(robot_yaw)
    vx_base = cos_y * vx_map + sin_y * vy_map
    vy_base = -sin_y * vx_map + cos_y * vy_map
    return vx_base, vy_base


def scan_to_points(angle_min, angle_increment, ranges, min_range=0.05, max_range=8.0):
    """把激光数据转换成点坐标。

    返回 (angles, xs, ys) 三个 numpy 数组，坐标在激光坐标系中，
    已去掉 NaN、inf 和超出 [min_range, max_range] 的无效点。
    """
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(len(ranges))
    valid = np.isfinite(ranges) & (ranges >= min_range) & (ranges <= max_range)
    angles = angles[valid]
    ranges = ranges[valid]
    return angles, ranges * np.cos(angles), ranges * np.sin(angles)


class WallFit(object):
    """一次墙线拟合的结果。

    distance：激光中心到墙线的垂直距离（米）。
    yaw_offset：机器人相对"正对这面墙"的偏转角（弧度，逆时针为正）。
                例如机器人向左转了 2 度，那么不论测哪一面墙，yaw_offset 都约为 +2 度。
    num_points：参与拟合的点数，越多越可信。
    """

    def __init__(self, side, distance, yaw_offset, num_points):
        self.side = side
        self.distance = distance
        self.yaw_offset = yaw_offset
        self.num_points = num_points

    def __repr__(self):
        return 'WallFit(%s, d=%.3f m, yaw=%.2f deg, n=%d)' % (
            self.side, self.distance, math.degrees(self.yaw_offset), self.num_points)


def _fit_line(xs, ys):
    """最小二乘直线拟合（主成分方向）。返回 (中心点, 单位方向向量, 每个点到直线的距离)。"""
    center = np.array([xs.mean(), ys.mean()])
    points = np.stack([xs - center[0], ys - center[1]], axis=1)
    # 协方差矩阵最大特征值对应的特征向量就是直线方向
    cov = points.T.dot(points)
    eigen_values, eigen_vectors = np.linalg.eigh(cov)
    direction = eigen_vectors[:, np.argmax(eigen_values)]
    normal = np.array([-direction[1], direction[0]])
    residuals = np.abs(points.dot(normal))
    return center, direction, residuals


def _ransac_inliers(xs, ys, threshold, iterations=60):
    """RANSAC 找"最多点共线"的那条线，返回属于这条线的点的布尔掩码。

    做法：反复随机取两个点连成直线，数一数有多少点离这条线不超过 threshold，
    保留点数最多的那次。这样即使扇区里混进了相邻墙角、挡板端头的点，也不会把墙线拉歪。
    随机数种子固定，保证同样的数据每次结果一样。
    """
    count = len(xs)
    rng = np.random.RandomState(12345)
    best_mask = None
    best_count = -1
    for _ in range(iterations):
        i, j = rng.randint(0, count, size=2)
        if i == j:
            continue
        dx = xs[j] - xs[i]
        dy = ys[j] - ys[i]
        length = math.hypot(dx, dy)
        if length < 0.05:      # 两点太近，算出的方向不可靠
            continue
        # 每个点到直线 (i, j) 的距离
        dist = np.abs((xs - xs[i]) * dy - (ys - ys[i]) * dx) / length
        mask = dist <= threshold
        if mask.sum() > best_count:
            best_count = mask.sum()
            best_mask = mask
    return best_mask


def fit_wall(angles, xs, ys, side, sector_half_angle=math.radians(30.0),
             expected_distance=None, distance_band=0.4, min_points=8,
             outlier_threshold=0.02, max_yaw_offset=math.radians(20.0)):
    """在指定一侧拟合墙线。

    参数：
        angles, xs, ys：scan_to_points 的输出。
        side：'front' / 'left' / 'back' / 'right'。
        sector_half_angle：只使用该侧 ±sector_half_angle 范围内的点。
        expected_distance：已知的大致墙距（米）。给出时只保留距离在
                           expected_distance ± distance_band 内的点，排除挡板端头等干扰。
        min_points：点数少于这个值时认为没看到墙，返回 None。
        outlier_threshold：RANSAC 判定"点在墙上"的距离阈值（米）。
        max_yaw_offset：拟合出的偏角超过这个值（弧度）就认为拟合错了（多半是把相邻的墙
                        当成了这面墙，比如车停在角落时），返回 None。

    返回 WallFit，看不到墙时返回 None。
    """
    side_angle = SIDE_ANGLE[side]
    angle_diff = np.abs(np.arctan2(np.sin(angles - side_angle), np.cos(angles - side_angle)))
    keep = angle_diff <= sector_half_angle
    if expected_distance is not None:
        # 点到"理想墙线"的投影距离：前/后墙看 x，左/右墙看 y
        if side in ('front', 'back'):
            projected = np.abs(xs)
        else:
            projected = np.abs(ys)
        keep &= np.abs(projected - expected_distance) <= distance_band
    wall_xs = xs[keep]
    wall_ys = ys[keep]
    if len(wall_xs) < min_points:
        return None

    # RANSAC 选出墙上的点 → 对这些点做最小二乘拟合
    inliers = _ransac_inliers(wall_xs, wall_ys, outlier_threshold)
    if inliers is None or inliers.sum() < min_points:
        return None
    wall_xs = wall_xs[inliers]
    wall_ys = wall_ys[inliers]
    center, direction, _ = _fit_line(wall_xs, wall_ys)

    # 垂直距离 = 中心点在法向上的投影长度
    normal = np.array([-direction[1], direction[0]])
    distance = abs(center.dot(normal))

    # 偏转角：前/后墙理想方向是 y 轴，左/右墙理想方向是 x 轴（推导见 README "贴墙精定位"一节）
    ux, uy = direction[0], direction[1]
    if side in ('front', 'back'):
        if uy < 0:
            ux, uy = -ux, -uy
        yaw_offset = math.atan2(ux, uy)
    else:
        if ux < 0:
            ux, uy = -ux, -uy
        yaw_offset = math.atan2(-uy, ux)
    if abs(yaw_offset) > max_yaw_offset:
        return None
    return WallFit(side, float(distance), float(yaw_offset), int(len(wall_xs)))


def points_in_base_frame(xs, ys, laser_offset_x):
    """激光坐标系的点 → 机器人坐标系（激光装在机器人中心前方 laser_offset_x 米处，没有转角）。"""
    return xs + laser_offset_x, ys
