# -*- coding: utf-8 -*-
"""环形靶靶心：直接在相机画面里找同心圆的圆心（代替 find_object）。

为什么换（2026-10-22 弹道测试，车完全不动时）：
  find_object 给的靶心横坐标和真靶心差 +1 ~ +12 像素（每组不一样），单帧还会跳 3 ~ 9 像素；
  2 米处 1 像素约 0.4 cm，所以就算弹着点标对了，左右也会散好几厘米。
  靶面是一圈圈同心环（10 环黄、9 环黑、8 环蓝、7 环白、6 环红），以真圆心为中心画圆，
  每个圆上的亮度几乎一样；圆心偏了，圆就会跨过不同颜色的环，同一个圆上亮度忽明忽暗。
  所以"让各个圆上的亮度变化最小"的点就是靶心。

做法（只用 numpy，Python 2/3 都能跑）：
  1. 粗找：没有上一帧结果时，在搜索框里每隔 4 像素试一个点（画面缩小一半算，快）；
     有上一帧结果时先只在它附近 ±track_px 里试。每个点的得分 = 灰度的比值 × 红减蓝的比值，
     比值 = 半径方向的变化 / 圆周方向的变化：靶心处环与环之间差别大（分子大）、同一圆上很均匀（分母小）。
     只用灰度时窗框、立柱的角落也能到靶的水平；红减蓝在灰白的墙和窗户上几乎为 0，乘上以后差距拉开到几十倍。
  2. 细找：对得分最高的几个点，各自在附近 ±2 像素按 1 像素、再 ±0.75 像素按 0.25 像素找"圆周方向灰度变化最小"的点，
     再按得分挑最好的一个。
  3. 得分低于 min_score 当作没看到靶（被挡住、不在画面里）。

坐标：原始相机像素（640x480），u 向右、v 向下。和 find_object 的像素不是一回事，
所以 ring.rings.aim_x_px 要单独标定（不能用 ring.aim_x_px 的 363）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import collections

import numpy as np


DEFAULTS = {
    'search_u': [200.0, 480.0],     # 搜索框（原始像素）：没有上一帧结果时在这里面找
    'search_v': [120.0, 320.0],
    'r_min_px': 4.0,                # 用多大半径范围内的环（像素）。2 米处 9 环外沿约 9.5、7 环外沿约 25、6 环外沿约 32.5
    'r_max_px': 32.0,               # 不要超过 6 环外沿太多：再往外是靶壳、底座，不对称
    'track_px': 12.0,               # 有上一帧结果时，只在它附近这么多像素里找
    'track_keep_s': 2.0,            # 上一帧结果多久以内才拿来缩小范围
    'candidates': 3,                # 粗找后最多细找几个候选点（粗得分不到最高者 1/4 的不找）
    'min_score': 5.0,               # 得分低于这个当作没看到靶。2026-10-22 实测：靶心 14~78，墙、窗框、立柱最高 0.9
    'max_rate_hz': 6.0,             # 最多每秒处理几帧（每帧几十到一两百毫秒，处理不过来就跳帧）
}


def to_channels(image_bgr):
    """灰度 和 红减蓝（R-B）两个通道。R-B 在灰白的墙、窗户上接近 0，靶上蓝环为负、红环为正，区分度很高。"""
    image = image_bgr.astype(np.float32)
    blue, green, red = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    return 0.114 * blue + 0.587 * green + 0.299 * red, red - blue


def _sample(gray, xs, ys, bilinear=True):
    """在 (xs, ys) 处取值（任意形状的坐标数组），出界的点夹到边上。"""
    height, width = gray.shape
    if not bilinear:
        xi = np.clip(np.rint(xs).astype(np.int32), 0, width - 1)
        yi = np.clip(np.rint(ys).astype(np.int32), 0, height - 1)
        return gray[yi, xi]
    xs = np.clip(xs, 0.0, width - 1.001)
    ys = np.clip(ys, 0.0, height - 1.001)
    x0 = np.floor(xs).astype(np.int32)
    y0 = np.floor(ys).astype(np.int32)
    fx = xs - x0
    fy = ys - y0
    top = gray[y0, x0] * (1.0 - fx) + gray[y0, x0 + 1] * fx
    bottom = gray[y0 + 1, x0] * (1.0 - fx) + gray[y0 + 1, x0 + 1] * fx
    return top * (1.0 - fy) + bottom * fy


def _profile(channel, us, vs, radii, angles, bilinear, mask=None):
    """每个候选点 (us[k], vs[k]) 周围各圆上的值：形状 (候选数, 半径数, 角度数)。mask 为 True 的像素当作缺失（NaN）。"""
    us = np.asarray(us, dtype=np.float32)[:, None, None]
    vs = np.asarray(vs, dtype=np.float32)[:, None, None]
    xs = us + radii[None, :, None] * np.cos(angles)[None, None, :]
    ys = vs + radii[None, :, None] * np.sin(angles)[None, None, :]
    values = _sample(channel, xs, ys, bilinear)
    if mask is not None:
        missing = _sample(mask.astype(np.float32), xs, ys, False) > 0.5
        values = np.where(missing, np.nan, values)
    return values


def _angular_var(values):
    """各圆上数值的方差，再对半径取平均（越小越像同心圆的圆心）。"""
    if np.isnan(values).any():
        return np.nanmean(np.nanvar(values, axis=2), axis=1)
    return values.var(axis=2).mean(axis=1)


def _ratio(values):
    """半径方向的变化 / 圆周方向的变化。+4：噪声底（2 个灰度的方差），防止平坦区域除出大数。"""
    ring_means = np.nanmean(values, axis=2) if np.isnan(values).any() else values.mean(axis=2)
    return ring_means.var(axis=1) / (_angular_var(values) + 4.0)


class RingFinder(object):
    def __init__(self, params=None):
        p = dict(DEFAULTS)
        p.update(params or {})
        self.p = p
        self.search_u = [float(x) for x in p['search_u']]
        self.search_v = [float(x) for x in p['search_v']]
        self.r_min = float(p['r_min_px'])
        self.r_max = float(p['r_max_px'])
        self.min_score = float(p['min_score'])
        self.track_px = float(p['track_px'])
        self.candidates = int(p['candidates'])
        self.fine_radii = np.arange(self.r_min, self.r_max + 1e-6, 0.5, dtype=np.float32)
        self.fine_angles = np.linspace(0.0, 2.0 * np.pi, 64, endpoint=False).astype(np.float32)
        self.coarse_angles = np.linspace(0.0, 2.0 * np.pi, 24, endpoint=False).astype(np.float32)

    def find(self, image_bgr, hint=None, mask=None):
        """返回 {'u', 'v', 'score'}（原始像素）；没找到靶返回 None。

        hint：上一帧的 (u, v)，有的话先只在附近找。mask：True 的像素不用（离线分析带标记线的存图时用）。
        """
        gray, red_blue = to_channels(image_bgr)
        return self.find_channels(gray, red_blue, hint, mask)

    def score(self, gray, red_blue, u, v, mask=None):
        """(u, v) 处的得分 = 灰度的"径向/周向变化比" × 红减蓝的"径向/周向变化比"。"""
        g = _profile(gray, [u], [v], self.fine_radii, self.fine_angles, True, mask)
        c = _profile(red_blue, [u], [v], self.fine_radii, self.fine_angles, True, mask)
        return float(_ratio(g)[0] * _ratio(c)[0])

    def find_channels(self, gray, red_blue, hint=None, mask=None):
        if hint is not None:
            starts = self._coarse(gray, red_blue, mask, hint[0] - self.track_px, hint[0] + self.track_px,
                                  hint[1] - self.track_px, hint[1] + self.track_px, half=False, step=2.0, keep=1)
            best = self._refine(gray, red_blue, mask, starts)
            if best is not None and best['score'] >= self.min_score:
                return best
        starts = self._coarse(gray, red_blue, mask, self.search_u[0], self.search_u[1],
                              self.search_v[0], self.search_v[1], half=True, step=4.0, keep=self.candidates)
        best = self._refine(gray, red_blue, mask, starts)
        if best is None or best['score'] < self.min_score:
            return None
        return best

    def _refine(self, gray, red_blue, mask, starts):
        best = None
        for (u0, v0, coarse_score) in starts:
            if coarse_score < 0.25 * starts[0][2]:
                break                                   # 粗得分差得多的候选不细找（省时间；靶心粗得分通常是最高的）
            u, v = self._fine(gray, mask, u0, v0)
            score = self.score(gray, red_blue, u, v, mask)
            if best is None or score > best['score']:
                best = {'u': u, 'v': v, 'score': score}
        return best

    def _coarse(self, gray, red_blue, mask, u0, u1, v0, v1, half, step, keep):
        """在 [u0,u1]×[v0,v1] 里每隔 step 像素试一个点，返回得分最高的 keep 个点（彼此相距 8 像素以上）。"""
        height, width = gray.shape
        u0, u1 = max(u0, self.r_max), min(u1, width - 1 - self.r_max)
        v0, v1 = max(v0, self.r_max), min(v1, height - 1 - self.r_max)
        if u1 < u0 or v1 < v0:
            return []
        grid_u, grid_v = np.meshgrid(np.arange(u0, u1 + 1e-6, step), np.arange(v0, v1 + 1e-6, step))
        grid_u, grid_v = grid_u.ravel(), grid_v.ravel()
        if half:
            scale = 2.0
            channels = (gray[::2, ::2], red_blue[::2, ::2])
            small_mask = mask[::2, ::2] if mask is not None else None
            radii = np.arange(max(2.0, self.r_min / 2.0), self.r_max / 2.0 + 1e-6, 1.0, dtype=np.float32)
        else:
            scale = 1.0
            channels = (gray, red_blue)
            small_mask = mask
            radii = np.arange(self.r_min, self.r_max + 1e-6, 1.0, dtype=np.float32)
        scores = []
        for k in range(0, len(grid_u), 1000):            # 分块算，省内存
            us, vs = grid_u[k:k + 1000] / scale, grid_v[k:k + 1000] / scale
            ratio_g = _ratio(_profile(channels[0], us, vs, radii, self.coarse_angles, False, small_mask))
            ratio_c = _ratio(_profile(channels[1], us, vs, radii, self.coarse_angles, False, small_mask))
            scores.append(ratio_g * ratio_c)
        scores = np.nan_to_num(np.concatenate(scores))
        picked = []
        for k in np.argsort(scores)[::-1]:
            if all((grid_u[k] - u) ** 2 + (grid_v[k] - v) ** 2 > 64.0 for (u, v, _) in picked):
                picked.append((float(grid_u[k]), float(grid_v[k]), float(scores[k])))
                if len(picked) >= keep:
                    break
        return picked

    def _fine(self, gray, mask, u, v):
        """在 (u, v) 附近找圆周方向灰度变化最小的点：先 ±2 像素按 1 像素，再 ±0.75 像素按 0.25 像素。"""
        for half_range, step in ((2.0, 1.0), (0.75, 0.25)):
            offsets = np.arange(-half_range, half_range + 1e-6, step)
            du, dv = np.meshgrid(offsets, offsets)
            cand_u, cand_v = u + du.ravel(), v + dv.ravel()
            costs = _angular_var(_profile(gray, cand_u, cand_v, self.fine_radii, self.fine_angles, True, mask))
            k = int(np.nanargmin(costs))
            u, v = float(cand_u[k]), float(cand_v[k])
        return u, v


def effective_ring_params(ring_cfg):
    """shooting.yaml ring 段实际生效的参数：detector 为 rings 时，ring.rings 里的同名项（aim_x_px、tolerance_px 等）覆盖上层。

    两种找靶心办法的像素不通用（find_object 的靶心和真靶心差 1~12 像素），所以各自的弹着点、噪声、延迟分开写。
    """
    params = dict(ring_cfg)
    if params.get('detector', 'find_object') == 'rings':
        for key, value in (ring_cfg.get('rings') or {}).items():
            if key != 'finder':
                params[key] = value
    return params


class RingSource(object):
    """环形靶靶心观测：[(拍摄时刻, 靶心横坐标 u), ...]，给 aiming.RingAimer 用。

    detector（shooting.yaml ring.detector）：
      rings        相机画面找同心圆（本模块 RingFinder，默认）
      find_object  旧做法：find_object 节点给的靶心（/object_position 话题）
    仿真（tests/sim_world.SimIO）提供 ring_finder_observations()，直接给"找同心圆"的结果，不渲染画面。
    """

    def __init__(self, ring_params):
        self.detector = ring_params.get('detector', 'find_object')
        self.accept_ids = ring_params.get('accept_ids', [])
        rings = ring_params.get('rings', {}) or {}
        self.finder = RingFinder(rings.get('finder', {}))
        self.min_period = 1.0 / float(self.finder.p['max_rate_hz'])
        self.track_keep_s = float(self.finder.p['track_keep_s'])
        self.results = collections.deque(maxlen=120)   # (拍摄时刻, u, v, 得分)
        self.last_stamp = None                         # 已经处理过的最新一帧的拍摄时刻
        self.last_found = None                         # (拍摄时刻, u, v)
        self.misses = 0                                # 处理了但没找到靶的帧数

    def observations(self, io, max_age, process=True):
        """最近 max_age 秒的观测 [(t, u)]。process=False 时不处理新画面（车在平移时省时间），只返回已有结果。"""
        if self.detector == 'find_object':
            return io.ring_observations(max_age, self.accept_ids)
        simulated = getattr(io, 'ring_finder_observations', None)
        if simulated is not None:
            return simulated(max_age)
        if process:
            self._process(io)
        oldest = io.now() - max_age
        return [(t, u) for (t, u, v, score) in self.results if t >= oldest]

    def latest(self):
        """最近一次找到的 (拍摄时刻, u, v, 得分)；还没有返回 None。"""
        return self.results[-1] if self.results else None

    def _process(self, io):
        since = (self.last_stamp + 1e-4) if self.last_stamp is not None else io.now() - 1.0
        frames = io.camera_frames(since)
        if not frames:
            return
        stamp, image = frames[-1]                      # 只处理最新的一帧
        if self.last_stamp is not None and stamp - self.last_stamp < self.min_period:
            return
        self.last_stamp = stamp
        hint = None
        if self.last_found is not None and stamp - self.last_found[0] <= self.track_keep_s:
            hint = self.last_found[1:]
        found = self.finder.find(image, hint)
        if found is None:
            self.misses += 1
            return
        self.last_found = (stamp, found['u'], found['v'])
        self.results.append((stamp, found['u'], found['v'], found['score']))
