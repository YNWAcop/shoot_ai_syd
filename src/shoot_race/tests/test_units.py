# -*- coding: utf-8 -*-
"""单元测试：各个纯计算模块 + 识别 + 配置检查 + 异常情况。

运行：python3 tests/test_units.py（电脑上）或 python2 tests/test_units.py（车上）
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))
sys.path.insert(0, HERE)

from shoot_race.compat import drop_python3_paths  # noqa: E402
drop_python3_paths()   # 车上 PYTHONPATH 里有 Python3 库目录，Python2 下要先去掉

import numpy as np  # noqa: E402

from shoot_race.aiming import MovingPredictor, RingAimer, RotatingPlanner, fit_wheel  # noqa: E402
from shoot_race.config import check_config, load_config, path_waypoints  # noqa: E402
from shoot_race.geometry import fit_wall, scan_to_points  # noqa: E402
from shoot_race.motion import ChassisModel, PathFollower, WallAligner, obstacle_in_path, VelocityCommand  # noqa: E402
from shoot_race.recognition import CardMatcher, CardRecognizer, parse_vlm_reply  # noqa: E402

RESULTS = []


def check(label, ok, detail=''):
    RESULTS.append(bool(ok))
    print('%s  %s  %s' % ('PASS' if ok else 'FAIL', label, detail))


CFG = load_config(os.path.join(PACKAGE_DIR, 'config'))


# ---------------- 墙线拟合 ----------------
def room_scan(yaw, back=1.2, left=0.6, right=0.8, front=2.0, noise=0.005, seed=0):
    rng = np.random.RandomState(seed)
    angles = -math.pi + np.arange(720) * 2 * math.pi / 720
    out = []
    for a in angles:
        dx, dy = math.cos(a + yaw), math.sin(a + yaw)
        d = np.inf
        if dx < -1e-9:
            d = min(d, -back / dx)
        if dx > 1e-9:
            d = min(d, front / dx)
        if dy > 1e-9:
            d = min(d, left / dy)
        if dy < -1e-9:
            d = min(d, -right / dy)
        out.append(d + rng.normal(0, noise))
    return scan_to_points(-math.pi, 2 * math.pi / 720, out)


for yaw_deg in (0.0, 2.0, -6.0):
    a, x, y = room_scan(math.radians(yaw_deg))
    fit = fit_wall(a, x, y, 'back', expected_distance=1.2)
    check('墙线拟合 back 墙，车偏 %+.0f 度' % yaw_deg,
          abs(fit.distance - 1.2) < 0.005 and abs(math.degrees(fit.yaw_offset) - yaw_deg) < 0.6, repr(fit))
check('看不到墙时返回 None', fit_wall(*room_scan(0.0), side='back', expected_distance=3.0) is None)

# ---------------- 贴墙精定位的方向 ----------------
params = dict(CFG['system']['wall_align'])
params['yaw_tolerance'] = math.radians(params['yaw_tolerance_deg'])
aligner = WallAligner([{'side': 'back', 'distance': 1.2}, {'side': 'left', 'distance': 0.6}], 'back', 0.0, params)
a, x, y = room_scan(math.radians(3.0), back=1.1, left=0.7)
fits = {'back': fit_wall(a, x, y, 'back', expected_distance=1.2), 'left': fit_wall(a, x, y, 'left', expected_distance=0.6)}
cmd, errors, aligned = aligner.update(fits)
check('离后墙太近 → 向前', cmd.vx > 0, repr(cmd))
check('离左墙太远 → 向左', cmd.vy > 0, repr(cmd))
check('车头偏左 → 向右转', cmd.wz < 0, repr(cmd))
check('误差大时不算完成', not aligned)

# ---------------- 路径跟随 ----------------
follower = PathFollower([(1.0, 0.0, 0.0), (1.0, -1.0, 0.0)], max_speed=0.35, corner_speed=0.2, decel=0.6,
                        max_yaw_rate=1.5, yaw_gain=2.5, goal_tolerance=0.04,
                        yaw_tolerance=math.radians(4))
cmd, done, _ = follower.update((0.0, 0.0, 0.0))
check('路径跟随：朝第一个点前进', cmd.vx > 0.3 and abs(cmd.vy) < 1e-6, repr(cmd))
follower = PathFollower([(1.0, 0.0, 0.0), (1.0, -1.0, 0.0)], max_speed=0.35, corner_speed=0.2, decel=0.6,
                        max_yaw_rate=1.5, yaw_gain=2.5, goal_tolerance=0.04,
                        yaw_tolerance=math.radians(4))   # 新建：跟随器输出有低通，不能拿上一个不连续的位姿接着算
cmd, done, _ = follower.update((0.95, 0.0, math.pi / 2))     # 车头朝左时，要往地图 -y 走 = 车体的右侧(-y)…
check('路径跟随：车头朝左时侧向分量正确', cmd.vx < -0.1 or cmd.vy < -0.1, repr(cmd))
cmd, done, _ = follower.update((1.0, -1.0, 0.0))
check('路径跟随：到终点完成', done)

# ---------------- 安全检查 ----------------
xs = np.array([0.30])
ys = np.array([0.0])
check('前方 0.35 m 有点、向前走 → 不挡（超过停车距离）',
      not obstacle_in_path(np.array([0.35]), ys, VelocityCommand(0.3, 0, 0), 0.23, 0.18, 0.16, 0.03, 0.08))
check('前方 0.27 m 有点、向前走 → 挡住',
      obstacle_in_path(np.array([0.27]), ys, VelocityCommand(0.3, 0, 0), 0.23, 0.18, 0.16, 0.03, 0.08))
check('前方有点、向后走 → 不挡',
      not obstacle_in_path(np.array([0.27]), ys, VelocityCommand(-0.3, 0, 0), 0.23, 0.18, 0.16, 0.03, 0.08))
check('后方 0.22 m 有点、向后走 → 挡住（车尾比车头短）',
      obstacle_in_path(np.array([-0.22]), ys, VelocityCommand(-0.3, 0, 0), 0.23, 0.18, 0.16, 0.03, 0.08))
# 2026-10-18 实测：通道里往右平移，挡板端头在车体 (0.214, -0.229)，下一刻车头就蹭上了。
# 按旧的对称车身（半长 0.18）它不在车道里，查不出来；车头按 0.23（含枪管）就会停车
check('挡板端头在车头右前方、向右平移 → 挡住（车头含枪管 0.23 m）',
      obstacle_in_path(np.array([0.214]), np.array([-0.229]), VelocityCommand(0, -0.2, 0), 0.23, 0.18, 0.16, 0.03, 0.12))
check('同一个点、按旧的对称车身 0.18 m → 查不出来（这次撞挡板的原因之一）',
      not obstacle_in_path(np.array([0.214]), np.array([-0.229]), VelocityCommand(0, -0.2, 0), 0.18, 0.18, 0.16, 0.02, 0.12))

# 急弯不切角：p1 → 通道点 → 通道点 → p2（两个直角弯），按实测底盘响应仿真，车头离挡板端头要留够余量
def sim_drive(start, waypoints, sharp_deg, max_speed=0.35, front=0.21, segments=None, dead=0.35, stats=None,
              gains=(0.75, 0.90, 0.80)):
    import collections
    from shoot_race.motion import VelocitySmoother
    p = CFG['system']['path_follower']
    f = PathFollower(waypoints, max_speed=max_speed, corner_speed=p['corner_speed'], decel=p['decel'],
                     max_yaw_rate=p['max_yaw_rate'], yaw_gain=p['yaw_gain'], goal_tolerance=p['goal_tolerance'],
                     yaw_tolerance=math.radians(p['yaw_tolerance_deg']), lookahead_m=p['lookahead_m'],
                     lookahead_s=p['lookahead_s'], wheel_limit=p['wheel_limit'], wheel_k=p['wheel_k'],
                     min_turn_rate=p['min_turn_rate'], close_finish_m=p['close_finish_m'],
                     sharp_corner_deg=sharp_deg, corner_reach_m=p['corner_reach_m'], segments=segments)
    model = ChassisModel(CFG['system']['chassis'], dt=0.05)
    smoother = VelocitySmoother(CFG['system']['accel_limits']['linear'], CFG['system']['accel_limits']['angular'])
    x, y, yaw = start
    v = [0.0, 0.0, 0.0]
    hist = collections.deque()
    t, worst_x = 0.0, -9.0
    min_x = 9.0
    while t < 30.0:
        cmd, done, _ = f.update((x, y, yaw), model)
        if done:
            break
        accel = f.current_segment().get('accel')
        smoother.linear_accel = accel if accel else CFG['system']['accel_limits']['linear']
        cmd = smoother.smooth(cmd, 0.05)
        model.record(t, cmd, tuple(v))
        hist.append((t, (cmd.vx, cmd.vy, cmd.wz)))
        d = (0.0, 0.0, 0.0)
        for tc, c in hist:
            if tc <= t - dead:
                d = c
        g = (gains[0] if d[0] >= 0 else gains[1], gains[2], 0.90)
        for _ in range(5):
            for i in range(3):
                v[i] += (g[i] * d[i] - v[i]) * (0.01 / 0.30)
            x += (v[0] * math.cos(yaw) - v[1] * math.sin(yaw)) * 0.01
            y += (v[0] * math.sin(yaw) + v[1] * math.cos(yaw)) * 0.01
            yaw += v[2] * 0.01
            if abs(y - (-0.96)) < 0.17:
                worst_x = max(worst_x, x + front)          # 经过挡板端头那一段时车头最靠前的位置
            min_x = min(min_x, x)
        t += 0.05
    if stats is not None:
        stats['min_x'] = min_x
    return t, worst_x, math.hypot(waypoints[-1][0] - x, waypoints[-1][1] - y)
# 到点后只剩转向：平移锁住（2026-10-18 实测：差 1 cm 时减速曲线仍给 0.1 m/s，车在终点前后左右点动 5 秒）
p = CFG['system']['path_follower']
follower = PathFollower([(1.0, -0.3, 0.0)], max_speed=0.35, corner_speed=0.15, decel=0.5, max_yaw_rate=1.0,
                        yaw_gain=2.0, goal_tolerance=0.04, yaw_tolerance=math.radians(4), min_turn_rate=0.45,
                        turn_hold_m=p['turn_hold_m'], turn_release_m=p['turn_release_m'])
model = ChassisModel(CFG['system']['chassis'], dt=0.05)
cmds = [follower.update((1.0 + dx, -0.3 + dy, math.radians(25)), model)[0]
        for dx, dy in ((0.01, 0.0), (-0.01, 0.01), (0.0, -0.012), (0.03, 0.02))]
check('到点还差 25 度：平移锁住、只原地转（位置抖 1~3 cm 也不点动）',
      all(abs(c.vx) + abs(c.vy) < 1e-9 and c.wz < -0.4 for c in cmds), repr(cmds[-1]))
cmd = follower.update((1.15, -0.3, math.radians(25)), model)[0]
check('锁住后被推开 15 cm（超过 turn_release_m）→ 重新平移', abs(cmd.vx) + abs(cmd.vy) > 0.01, repr(cmd))

# 交接朝向（2026-10-20）：车已停在终点 2 cm 处、朝向差 5 度（容差 4、交接 6）→ 行驶结束，交给贴墙精定位
follower = PathFollower([(1.0, -0.3, 0.0)], max_speed=0.35, corner_speed=0.15, decel=0.5, max_yaw_rate=1.0,
                        yaw_gain=2.0, goal_tolerance=0.04, yaw_tolerance=math.radians(4), min_turn_rate=0.45,
                        handoff_yaw=math.radians(p['handoff_yaw_deg']))
model = ChassisModel(CFG['system']['chassis'], dt=0.05)
for i in range(10):
    model.record(100.0 + 0.05 * i, VelocityCommand(0.0, 0.0, 0.0))
done = follower.update((1.0, -0.28, math.radians(5.0)), model)[1]
check('停在终点、朝向差 5 度（交接容差 %.0f 度）→ 交给精定位，不在原地补' % p['handoff_yaw_deg'], done)
done = follower.update((1.0, -0.28, math.radians(9.0)), model)[1]
check('朝向差 9 度 → 还要先转', not done)
# 原地转刚发指令、模型里车还在死区：不能因为"预测到位"就收（22:19 去 1 号点一发一停空转 2.3 秒）
follower = PathFollower([(1.0, -0.3, 0.0)], max_speed=0.35, corner_speed=0.15, decel=0.5, max_yaw_rate=1.0,
                        yaw_gain=2.0, goal_tolerance=0.04, yaw_tolerance=math.radians(4), min_turn_rate=0.45)
model = ChassisModel(CFG['system']['chassis'], dt=0.05)
t = 100.0
for i in range(10):
    model.record(t, VelocityCommand(0.0, 0.0, 0.0))
    t += 0.05
pose = (1.0, -0.3, math.radians(4.6))
rates = []
for i in range(12):                       # 0.6 秒：每个周期都应该继续发转向（死区 0.3 秒内车没转）
    wz, turned = follower.turn_in_place(pose, model, 0.0)
    model.record(t, VelocityCommand(0.0, 0.0, wz))
    rates.append(wz)
    t += 0.05
check('差 4.6 度（容差 4 度）原地转：死区里连续发转向，不一发一停', all(r < -0.4 for r in rates[:8]),
      ' '.join('%.2f' % r for r in rates))

lane = [(1.008, -0.357, 0.0), (0.10, -0.357, 0.0), (0.10, -1.541, 0.0), (0.972, -1.541, 0.0)]
t_new, head_new, err_new = sim_drive(lane[0], lane[1:], CFG['system']['path_follower']['sharp_corner_deg'])
check('急弯不切角：通道点 x=0.10 时车头离挡板端头（x=0.44）≥ 8 cm，到终点 < 5 cm',
      0.44 - head_new >= 0.08 and err_new < 0.05, '车头最远 x=%.3f，用时 %.1f s，终点差 %.1f cm' % (head_new, t_new, 100 * err_new))

# 分段档位（2026-10-20）：离开任务框后退、通道横移用爆发档（末端减到 0、停住再横移），进框用常规档
from shoot_race.config import path_segments  # noqa: E402
burst_cfg = dict(CFG)
burst_cfg['system'] = dict(CFG['system'], path_follower=dict(CFG['system']['path_follower'], profiles_enabled=True))
burst_cfg['field'] = dict(CFG['field'], paths={'t': [{'to': 'a', 'profile': 'burst'}, {'to': 'b', 'profile': 'burst'}, 'c']},
                          waypoints={'a': [0.10, -0.357, 0.0], 'b': [0.10, -1.541, 0.0], 'c': [0.972, -1.541, 0.0]})
segs = path_segments(burst_cfg, 't')
check('路径里 {to: 点, profile: burst} 用爆发档，直接写点名用常规档',
      [sg['profile'] for sg in segs] == ['burst', 'burst', 'normal'] and segs[0]['end_speed'] == 0.0, repr(segs))
off_cfg = dict(burst_cfg)
off_cfg['system'] = dict(CFG['system'], path_follower=dict(CFG['system']['path_follower'], profiles_enabled=False))
check('profiles_enabled: false → 全部按常规档', [sg['profile'] for sg in path_segments(off_cfg, 't')] == ['normal'] * 3)
MAT = (0.85, 0.93, 0.92)          # 2026-10-18 垫子上整局录包：前进、后退、横移实到指令的比例（0.35 m/s 时）
# 底盘死区：2026-10-20 16:19 录包按轮速计时间戳拟合 0.15 秒（10-18 拟合 0.10~0.20）；0.40 是以前偏保守的估计
t_norm, _, _ = sim_drive(lane[0], lane[1:], CFG['system']['path_follower']['sharp_corner_deg'], gains=MAT, dead=0.15)
st_burst = {}
t_burst, head_burst, err_burst = sim_drive(lane[0], lane[1:], CFG['system']['path_follower']['sharp_corner_deg'],
                                           segments=segs, stats=st_burst, gains=MAT, dead=0.15)
check('爆发档（死区 0.15）：比常规档快，车头离挡板端头 ≥ 8 cm，车尾冲过通道点 ≤ 4 cm，到终点 < 5 cm',
      t_burst < t_norm - 0.5 and 0.44 - head_burst >= 0.08 and 0.10 - st_burst['min_x'] <= 0.04 and err_burst < 0.05,
      '用时 %.1f s（常规 %.1f s），车头最远 x=%.3f，最远退到 x=%.3f，终点差 %.1f cm' % (
          t_burst, t_norm, head_burst, st_burst['min_x'], 100 * err_burst))
st_b40 = {}
t_b40, head_b40, err_b40 = sim_drive(lane[0], lane[1:], CFG['system']['path_follower']['sharp_corner_deg'],
                                     segments=segs, stats=st_b40, gains=MAT, dead=0.40)
check('爆发档（死区 0.40，偏保守）：车尾冲过通道点 ≤ 8 cm（通道车尾间隙留 12 cm，仍剩 ≥ 4 cm），车头 ≥ 8 cm，到终点 < 5 cm',
      0.10 - st_b40['min_x'] <= 0.08 and 0.44 - head_b40 >= 0.08 and err_b40 < 0.05,
      '用时 %.1f s，车头最远 x=%.3f，最远退到 x=%.3f，终点差 %.1f cm' % (t_b40, head_b40, st_b40['min_x'], 100 * err_b40))
st_slow = {}
t_slow, _, err_slow = sim_drive(lane[0], lane[1:], CFG['system']['path_follower']['sharp_corner_deg'],
                                segments=segs, dead=0.5, stats=st_slow, gains=MAT)
check('底盘死区 0.5 秒（冲过拐点）：照样开到终点，不会停在拐点不动', t_slow < t_norm + 1.0 and err_slow < 0.05,
      '用时 %.1f s，最远退到 x=%.3f，终点差 %.1f cm' % (t_slow, st_slow['min_x'], 100 * err_slow))

# ---------------- 环形靶对准（停稳看 → 横向平移 → 再看）----------------
ring = dict(CFG['shooting']['ring'])
ring['max_shift_m'] = 0.05                   # 这一组测平移逻辑（偏 8 像素要走 32 mm）；场地上的上限（10-25 起 2.5 cm）单独测
SHIFT = CFG['system']['lateral_shift']
FX = CFG['system']['camera']['fx']
aimer = RingAimer(ring, SHIFT, FX)
latency = ring['vision_latency_s']
origin = -0.30                               # 横向位置（米，左为正）= -左墙距离：车离左墙 0.30 m


def moved_left(distance):
    """车从 origin 往左平移 distance 米后的横向位置（左墙距离变小，横向位置变大）。"""
    return origin + distance


# 车停稳的时刻是 0；停稳前拍的帧（收到时刻 < 0 + latency）偏差很大，必须被忽略
AIM = ring['aim_x_px']
GAIN = ring['image_gain']
old_frames = [(0.05, AIM + 60.0), (0.10, AIM + 60.0)]
vy, ready, info = aimer.update(0.0, origin, [], True)            # t=0 车停稳，开始"看"
vy, ready, info = aimer.update(0.2, origin, old_frames, True)
check('停稳后只用 vision_latency_s 之后收到的帧', not ready and info['reason'] == 'waiting_frames', str(info))
fresh = old_frames + [(latency + 0.05, AIM + 8.0), (latency + 0.45, AIM + 8.5), (latency + 0.85, AIM + 7.5)]
vy, ready, info = aimer.update(latency + 0.9, origin, fresh, True)
expected_dy = -8.0 / (FX * GAIN) * ring['target_distance_m']
check('偏右 8px → 往右平移 8/(fx×image_gain)×D = %.1f mm' % (-expected_dy * 1000),
      aimer.state == 'shift' and abs(aimer.shift_target - expected_dy) < 1e-9 and not ready, str(info))
vy, ready, info = aimer.update(1.0, origin, fresh, False)
check('shift 状态：往右移 = 横向速度为负', vy < 0 and not ready, 'vy=%.3f' % vy)
vy, ready, info = aimer.update(1.05, moved_left(expected_dy / 2), fresh, False)
check('走了一半：继续往右移', vy < 0 and aimer.state == 'shift', 'vy=%.3f' % vy)
vy, ready, info = aimer.update(1.1, moved_left(expected_dy), fresh, False)
check('移到位 → 回到 look、速度为 0', aimer.state == 'look' and vy == 0.0, str(info))
# 平移到位后：新位置上看到 2 帧（recheck_frames）就够，和平移前的一轮合起来估计
here = moved_left(expected_dy)
t0 = 2.0
aimer.update(t0, here, fresh, True)                               # 停稳，开始看
after = fresh + [(t0 + latency + 0.1, AIM + 0.8), (t0 + latency + 0.5, AIM - 0.4)]
vy, ready, info = aimer.update(t0 + latency + 0.6, here, after, True)
check('平移后看 %d 帧、和平移前合并估计 → 开火' % ring['recheck_frames'],
      ready and info.get('looks') == 2 and abs(info['err_px']) <= ring['tolerance_px'], str(info))

stuck = RingAimer(ring, SHIFT, FX)
stuck.update(0.0, origin, [], True)
stuck.update(latency + 0.9, origin, fresh, True)                  # 进入 shift
for k in range(80):                                               # 横向位置一直不变（底盘卡住）
    vy, ready, info = stuck.update(1.0 + 0.05 * k, origin, fresh, False)
    if stuck.state == 'look':
        break
check('平移卡住 → shift_stall_s 后回到 look 重新看', stuck.state == 'look' and info.get('reason') == 'shift_stalled', str(info))

far = RingAimer(ring, SHIFT, FX)
far.update(0.0, origin, [], True)
way_off = [(latency + 0.1 * k, AIM + 200.0) for k in range(3)]     # 偏右 200 px（识别错了或车放偏了）
far.update(latency + 0.4, origin, way_off, True)
check('累计平移量不超过 max_shift_m', abs(far.shift_target) <= ring['max_shift_m'] + 1e-9,
      'shift=%.1f mm' % (far.shift_target * 1000))
far_pos = origin + far.shift_target
far.update(1.0, far_pos, way_off, False)                          # 移到位
far.update(2.0, far_pos, way_off, True)
still_off = way_off + [(2.0 + latency + 0.1 * k, AIM + 150.0) for k in range(3)]
vy, ready, info = far.update(2.0 + latency + 0.4, far_pos, still_off, True)
check('平移已到上限还是偏 → 直接开火（不干等到超时）', ready and info.get('reason') == 'max_shift_reached', str(info))

# 2026-10-25 整局：1 号框精定位到位后靶心在弹着点右 13.75 像素，平移 5 cm（当时的上限）打中 10 环但车出框。
# 场地配置的上限要留在框内（框左右余量约 4.5 cm，扣掉精定位误差 0.8 cm、车头转过后车角多出约 0.5 cm）
from shoot_race.ring_finder import effective_ring_params as _ring_params  # noqa: E402
field_ring = _ring_params(CFG['shooting']['ring'])
check('场地配置：环形靶累计平移上限 ≤ 3 cm（5 cm 实测出框）', field_ring['max_shift_m'] <= 0.03 + 1e-9,
      'max_shift_m=%.3f' % field_ring['max_shift_m'])
cap = RingAimer(field_ring, SHIFT, FX)
cap_aim = field_ring['aim_x_px']
cap_lat = 0.30
cap.update(0.0, cap_lat, [], True)
lat_ring = field_ring['vision_latency_s']
first = [(lat_ring + 0.1 * k, cap_aim + 13.75) for k in range(3)]
cap.update(lat_ring + 0.35, cap_lat, first, True)
check('10-25 场景：偏右 13.75 像素 → 只平移到上限（不再走 5 cm）', cap.state == 'shift' and
      abs(cap.shift_target + field_ring['max_shift_m']) < 1e-9, 'shift=%.1f mm' % (cap.shift_target * 1000))

aimer2 = RingAimer(ring, SHIFT, FX)
aimer2.update(0.0, origin, [], True)                              # t=0 车停稳
good = [(latency + 0.1 * k, ring['aim_x_px'] + d) for k, d in enumerate([0.5, -0.8, 0.3])]
vy, ready, info = aimer2.update(latency + 0.5, origin, good, True)
check('偏差在容差内 → 可以开火', ready and vy == 0.0, str(info))
vy, ready, info = RingAimer(ring, SHIFT, FX).update(0.0, origin, [], True)
check('看不到靶 → 原地不动、不开火', vy == 0.0 and not ready)

# 只差一点（容差 ~ look_more_px 之间）：原地多看几帧，不急着平移
near = RingAimer(ring, SHIFT, FX)
near.update(0.0, origin, [], True)
mid = 0.5 * (ring['tolerance_px'] + ring['look_more_px'])
frames3 = [(latency + 0.4 * k, AIM + mid + d) for k, d in enumerate([0.3, -0.2, 0.1])]
vy, ready, info = near.update(latency + 1.0, origin, frames3, True)
check('偏 %.1f px（容差 %.1f ~ %.1f 之间）→ 原地多看，不平移' % (mid, ring['tolerance_px'], ring['look_more_px']),
      not ready and near.state == 'look' and info.get('reason') == 'look_more', str(info))
frames6 = frames3 + [(latency + 1.2 + 0.4 * k, AIM + d) for k, d in enumerate([0.5, -1.0, 0.2])]
vy, ready, info = near.update(latency + 2.5, origin, frames6, True)
check('多看几帧后中位数回到容差内 → 开火（前 3 帧是噪声）', ready, str(info))

# 平移后新一轮和旧估计对不上（换算倍数错或靶被挪）→ 旧的作废
# 卡估计误差（max_sigma_px）：3 帧估计在容差内但误差大 → 多看几帧；帧数够了再开火（2026-10-19 加，默认不开）
gated = dict(ring, max_sigma_px=1.5, max_look_frames=12)
g = RingAimer(gated, SHIFT, FX)
noisy = [ring['aim_x_px'] + d for d in (1.0, -1.5, 2.0, 0.5, -1.0, 1.5, -0.5, 1.0, 0.0, -1.0, 0.5, 1.0, -0.5, 0.5)]
g.update(0.5, origin, [], True)                     # 车停稳，从 0.5 秒开始看
results = []
for n in range(1, len(noisy) + 1):
    obs = [(1.0 + 0.4 * k, noisy[k]) for k in range(n)]
    vy, ready, info = g.update(1.0 + 0.4 * (n - 1) + 0.01, origin, obs, True)
    if 'err_px' in info:
        results.append((n, ready, round(info['sigma_px'], 2)))
    if ready:
        break
check('卡估计误差：3 帧在容差内也先不开火，估计误差降到 1.5 像素以下才开火',
      results and not results[0][1] and results[-1][1] and results[-1][2] <= 1.5 and g.shifts == 0, str(results))
far_g = RingAimer(gated, SHIFT, FX)
far_g.update(0.5, origin, [], True)
obs = [(1.0 + 0.4 * k, ring['aim_x_px'] + 9.0 + d) for k, d in enumerate((0.5, -0.5, 0.3))]
vy, ready, info = far_g.update(1.85, origin, obs, True)
check('卡估计误差：明显在容差外（9 像素）照样直接平移，不多看', not ready and 'shift_mm' in info, str(info))

# 横移时稳车头（2026-10-19 22:29 菜单 6：不稳车头时横移 45 mm 转轮在画面里几乎不动）
from shoot_race.motion import hold_heading  # noqa: E402
check('稳车头：车头偏左 1 度 → 向右转；偏 0.1 度（在死区内）→ 不转；偏很多 → 限幅 0.2 rad/s',
      hold_heading(math.radians(1.0)) < 0 and hold_heading(math.radians(-1.0)) > 0
      and hold_heading(math.radians(0.1)) == 0.0 and abs(hold_heading(math.radians(20))) <= 0.2 + 1e-9)

jump = RingAimer(ring, SHIFT, FX)
jump.update(0.0, origin, [], True)
jump.update(latency + 0.9, origin, fresh, True)                   # 偏右 8 px → 平移
pos = origin + jump.shift_target
jump.update(1.5, pos, fresh, False)                               # 到位
jump.update(2.0, pos, fresh, True)
odd = fresh + [(2.0 + latency + 0.1, AIM - 15.0), (2.0 + latency + 0.5, AIM - 15.5)]
vy, ready, info = jump.update(2.0 + latency + 0.6, pos, odd, True)
check('平移后新帧和旧估计差 > reset_px → 丢掉旧的一轮', 'reset_px' in info and info.get('looks') == 1, str(info))

# ---------------- 移动靶预测（按 moving2.bag 实测的运动：18 px/s 往返，端点立即换向）----------------
DT = 1.0 / CFG['system']['loop_hz']
mp = dict(CFG['shooting']['moving'])


def rail_u(t, center=380.0, half_travel=60.0, period=13.6):  # noqa: E302
    """三角波往返：中心 center、单边行程 half_travel 像素、来回周期 period 秒。"""
    phase = (t / period) % 1.0
    return center + half_travel * (4 * phase - 1 if phase < 0.5 else 3 - 4 * phase)


def run_moving(aim_u, duration=30.0, misread=0.0, seed=0, static_u=None, t0=0.0, rail_period=13.6):
    """按 20 Hz 调用 MovingPredictor，AR 25 Hz、延迟 0.04 s。返回 [(开火时刻, 弹丸到达时目标真实位置)]。

    rail_period：来回周期（13.6 秒 = 2025 场地约 18 像素/秒；越小越快）。开火按 info['delay_s'] 精确延后（同 ShootMovingStep）。
    """
    rng = np.random.RandomState(seed)
    params = dict(mp)
    params['aim_u_px'] = aim_u
    predictor = MovingPredictor(params, DT)
    samples, fires = [], []
    next_frame, now = t0, t0
    resume = t0                                                    # 打完一枪后隔 2 秒再继续（真实步骤只打一次）
    while now < t0 + duration:
        while next_frame <= now - 0.04:
            u = static_u if static_u is not None else rail_u(next_frame, period=rail_period)
            if rng.rand() < misread:
                u += rng.choice([-55.0, -27.0, 27.0, 55.0])        # 认成旁边的码
            samples.append((next_frame, u + rng.normal(0, 0.4), 193.0))
            next_frame += 0.04
        recent = [x for x in samples if x[0] >= now - params['history_s']]
        if now >= resume:
            fire, info = predictor.check(now, recent)
            if fire:
                t_fire = now + info.get('delay_s', 0.0)
                truth = static_u if static_u is not None else rail_u(t_fire + params['latency_s'], period=rail_period)
                fires.append((t_fire, truth, info['reason']))
                predictor = MovingPredictor(params, DT)
                resume = now + 2.0
        now += DT
    return fires


fires = run_moving(380.0)
errors = [abs(truth - 380.0) for _, truth, _ in fires]
check('移动靶：行程中间的弹着点，每次开火弹丸到达时误差 < 1.5 px', fires and max(errors) < 1.5,
      '%d 次，最大误差 %.2f px' % (len(fires), max(errors) if errors else -1))
fires = run_moving(380.0, misread=0.05, seed=3)
errors = [abs(truth - 380.0) for _, truth, _ in fires]
check('移动靶：5% 误识别（码号认成旁边的码）不影响', fires and max(errors) < 1.5,
      '%d 次，最大误差 %.2f px' % (len(fires), max(errors) if errors else -1))
worst = 0.0
count = 0
for aim in (321.0, 323.0, 326.0, 434.0, 437.0, 439.0):            # 靠近两端（端点 320 / 440）
    for seed in range(3):
        for _, truth, _ in run_moving(aim, duration=40.0, seed=seed, t0=seed * 2.3):
            worst = max(worst, abs(truth - aim))
            count += 1
check('移动靶：弹着点靠近行程端点时，开火都准（不在换向时打）', worst <= mp['hit_tolerance_px'] + 1.0,
      '%d 次开火，最大误差 %.2f px' % (count, worst))
# 最新一帧恰好是误识别（码号对、位置是旁边的码）：不能把它当真、算出离谱的速度
predictor = MovingPredictor(mp, DT)
base = [(10.0 - 0.04 * k, rail_u(10.0 - 0.04 * k), 193.0) for k in range(40)][::-1]
bad_last = base[:-1] + [(base[-1][0], base[-1][1] + 27.0, 193.0)]
_, info_good = predictor.check(10.04, base)
_, info_bad = MovingPredictor(mp, DT).check(10.04, bad_last)
check('移动靶：最新一帧是误识别 → 被剔除，速度不受影响',
      abs(info_bad.get('speed', 99) - info_good.get('speed', 0)) < 2.0,
      'speed 正常 %.1f / 带坏点 %.1f' % (info_good.get('speed', 0), info_bad.get('speed', 0)))
# 2026-10-23：滑块快很多时（往返 4.5 秒 ≈ 53 像素/秒、2.7 秒 ≈ 89 像素/秒）照样开火，精确定时后误差仍 < 1.5 像素
fast = []
for period in (4.5, 2.7):
    fs = run_moving(380.0, duration=20.0, seed=4, rail_period=period)
    fast.append((period, len(fs), max([abs(truth - 380.0) for _, truth, _ in fs] or [99.0])))
check('移动靶：滑块快到 50~90 像素/秒也开火（以前 > 40 判数据有问题），误差 < 2.5 px（码宽约 14 像素；不在控制周期上取整）',
      all(n >= 2 and e < 2.5 for _, n, e in fast), '；'.join('往返 %.1f 秒 开火 %d 次 最大误差 %.2f px' % f for f in fast))
# ---------------- 2026-10-23 移动靶截击（扭车头把弹着点挪到码更早经过的地方）----------------
from shoot_race.steps import ShootMovingStep, DONE as STEP_DONE  # noqa: E402


class _FakeLog(object):
    def __init__(self):
        self.events, self.shots = [], []

    def event(self, step_id, event, detail=''):
        self.events.append((event, detail))

    def shot(self, step_id, target, trigger, detail=''):
        self.shots.append(detail)

    def info(self, text):
        pass


class _MovingWorld(object):
    """移动靶 + 车头（原地转按实车：指令 0.3 秒后起作用、实际 0.2 倍、起步要连续发 0.5 秒、时间常数 0.3 秒）。

    码在"车头正对侧墙"坐标里的行程 = 10-23 实测（车头 -2 度时 6 号码 254~363）换算；速度 14.3 像素/秒，两端停 0.4 秒。
    画面 u = 世界 u + 8.8 × 车头（度，左转为正）。AR 约 25 帧/秒、延迟 0.04 秒、噪声 0.4 像素。
    """

    def __init__(self, phase_s, heading0_deg=-2.0, stroke=(254.0, 363.0), speed=14.3, seed=0, k_scale=1.0,
                 layout=None, blind_px=None, aim=350.5):
        self.rng = np.random.RandomState(seed)
        self.layout = layout          # {码号: 画面里相对 6 号码的像素}；None = 三个码号都返回同一组帧（老用例）
        self.blind_px = blind_px      # 6 号码离弹着点不到这么多像素时认不出来（10-25 傍晚：弹着点附近断档）
        self.aim = aim
        self.t = 0.0
        self.k = 8.8 * k_scale
        self.heading = math.radians(heading0_deg)           # 车头相对侧墙
        self.cal = math.radians(-2.0)
        self.left, self.right = stroke[0] - 8.8 * -2.0, stroke[1] - 8.8 * -2.0   # 换到正对侧墙的坐标
        self.speed, self.dwell = speed, 0.4
        self.leg = (self.right - self.left) / speed + self.dwell
        self.phase = phase_s
        self.rate, self.hist, self.cmd = 0.0, [], 0.0
        self.frames, self.next_frame = [], 0.0
        self.still_since = 0.0
        self.shots = []

    def code_world(self, t):
        x = (t + self.phase) % (2 * self.leg)
        if x < self.leg:
            return self.left + min(x, self.leg - self.dwell) * self.speed
        x -= self.leg
        return self.right - min(x, self.leg - self.dwell) * self.speed

    def image_u(self, t, heading):
        return self.code_world(t) + self.k * math.degrees(heading)

    def advance(self, dt):
        end = self.t + dt
        while self.t < end - 1e-9:
            step = min(DT, end - self.t)
            self.hist.append((self.t, self.cmd))
            delayed = ([c for (tc, c) in self.hist if tc <= self.t - 0.3] or [0.0])[-1]
            held = 0.0
            for tc, c in reversed(self.hist):
                if tc > self.t - 0.3:
                    continue
                if abs(c) < 0.15:
                    break
                held += DT
            target = 0.2 * delayed if (abs(delayed) >= 0.15 and (held >= 0.5 or abs(self.rate) > 0.005)) else 0.0
            self.rate += (target - self.rate) * min(1.0, step / 0.3)
            self.heading += self.rate * step
            if abs(self.rate) > math.radians(0.3) or abs(self.cmd) > 1e-6:
                self.still_since = self.t
            self.t += step
            while self.next_frame <= self.t - 0.04:
                u = self.image_u(self.next_frame, self.heading)
                if self.layout is None:
                    self.frames.append((self.next_frame, u + self.rng.normal(0, 0.4), 196.0))
                else:                                       # 另两个码各自的识别噪声
                    self.frames.append((self.next_frame, u + self.rng.normal(0, 0.4), 196.0,
                                        self.rng.normal(0, 0.4), self.rng.normal(0, 0.4)))
                self.next_frame += 0.04 * (1 + 0.2 * self.rng.rand())

    # ---- ctx.io ----
    def now(self):
        return self.t

    def ar_pixels(self, ids, window):
        recent = [f for f in self.frames if f[0] >= self.t - window]
        if self.layout is None:
            return dict((k, recent) for k in ids)
        out = {}
        for k in ids:
            if k == 6:
                out[k] = [f[:3] for f in recent if self.blind_px is None or abs(f[1] - self.aim) >= self.blind_px]
            elif k in self.layout:
                d = self.layout[k] - self.layout[6]
                out[k] = [(f[0], f[1] + d + f[3 if k == 7 else 4], f[2] + 0.5) for f in recent]
            else:
                out[k] = []
        return out

    def odom_yaw(self):
        return self.heading + 0.3 + self.rng.normal(0, 0.0005)      # 里程计朝向（原点任意）

    def scan_points(self):
        return None

    def sleep(self, seconds):
        self.advance(seconds)

    def send_velocity(self, command):
        self.cmd = 0.0

    def camera_frames(self, since):
        return []


class _Ctx(object):
    def __init__(self, world, cfg):
        self.io, self.cfg, self.log = world, cfg, _FakeLog()
        self.targets = {'moving': 6}
        self.command = None
        self.dry_fire = True
        self.shooter = None

    def robot_still_for(self, seconds):
        return self.io.t - self.io.still_since >= seconds

    def still_duration(self):
        return max(0.0, self.io.t - self.io.still_since)


def run_moving_step(phase_s, intercept=True, seed=0, heading0_deg=-2.0, max_time=16.0, stroke=(254.0, 363.0), k_scale=1.0,
                    fuse=None, layout=None, blind_px=None):
    cfg = copy.deepcopy(CFG)
    cfg['shooting']['moving']['intercept']['enabled'] = intercept
    if fuse is not None:
        cfg['shooting']['moving']['fuse_codes'] = [6, 7, 8] if fuse else []
    world = _MovingWorld(phase_s, heading0_deg, stroke, seed=seed, k_scale=k_scale, layout=layout, blind_px=blind_px,
                         aim=cfg['shooting']['moving']['aim_u_px'])
    ctx = _Ctx(world, cfg)
    step = ShootMovingStep({'id': 'shoot_move', 'type': 'shoot_ar', 'target': 'moving', 'timeout_s': 16})
    step.start(ctx)
    aim = cfg['shooting']['moving']['aim_u_px']
    latency = cfg['shooting']['moving']['latency_s']
    while world.t < max_time:
        status = step.update(ctx)
        if status == STEP_DONE:
            t_fire = world.t - cfg['shooting']['moving']['fire']['post_fire_wait_s']
            err = world.image_u(t_fire + latency, world.heading) - aim
            return t_fire, err, step.twists, math.degrees(world.heading), ctx.log
        world.cmd = ctx.command.wz if ctx.command is not None else 0.0
        world.advance(DT)
    return None, None, step.twists, math.degrees(world.heading), ctx.log


import copy  # noqa: E402
leg = (363.0 - 254.0) / 14.3 + 0.4
phases = np.linspace(0, 2 * leg, 16, endpoint=False)
base = [run_moving_step(ph, intercept=False, seed=700 + i) for i, ph in enumerate(phases)]
icpt = [run_moving_step(ph, intercept=True, seed=700 + i) for i, ph in enumerate(phases)]
tb = [r[0] if r[0] is not None else 16.0 for r in base]
ti = [r[0] if r[0] is not None else 16.0 for r in icpt]
check('移动靶截击（6 号码，弹着点离行程右端约 12 像素）：平均等待明显缩短、最长 ≤ 11 秒，开火误差 ≤ 3 像素、扭不超上限',
      np.mean(ti) <= np.mean(tb) - 1.0 and max(ti) <= 11.0 and all(r[0] is not None and abs(r[1]) <= 3.0 for r in icpt)
      and max(abs(r[3]) for r in icpt) <= CFG['shooting']['moving']['intercept']['max_twist_deg'] + 0.5,
      '不扭 平均 %.1f 最长 %.1f 秒 / 截击 平均 %.1f 最长 %.1f 秒；截击误差最大 %.1f px，扭 %s 次，车头最多 %.1f 度' % (
          np.mean(tb), max(tb), np.mean(ti), max(ti), max(abs(r[1]) for r in icpt if r[1] is not None),
          sum(r[2] for r in icpt), max(abs(r[3]) for r in icpt)))
off = [run_moving_step(ph, intercept=True, seed=800 + i, stroke=(244.0, 353.0)) for i, ph in enumerate(phases[::2])]
check('移动靶截击：车停的位置不同（行程整体偏 10 像素、标定值不准；不扭的话弹着点离端点只剩 2.5 像素、根本打不了）照样打准、不超时',
      all(r[0] is not None and abs(r[1]) <= CFG['shooting']['moving']['hit_tolerance_px'] for r in off),
      '用时 %s；误差 %s' % (' '.join('%.1f' % (r[0] or -1) for r in off), ' '.join('%.1f' % (r[1] or 99) for r in off)))
mis_k = [run_moving_step(ph, intercept=True, seed=900 + i, k_scale=1.3) for i, ph in enumerate(phases[::2])]
check('移动靶截击：扭的换算差 30%（扭完弹着点没落在算好的位置）照样打准（开火看画面，不靠扭得准）',
      all(r[0] is not None and abs(r[1]) <= 3.0 for r in mis_k),
      '用时 %s；误差 %s' % (' '.join('%.1f' % (r[0] or -1) for r in mis_k), ' '.join('%.1f' % (r[1] or 99) for r in mis_k)))

fires = run_moving(380.0, duration=3.0, static_u=381.5)
check('移动靶：静止目标（手扶着）在容差内 → 开火', fires and fires[0][2] == 'static', str(fires[:1]))
fires = run_moving(380.0, duration=3.0, static_u=395.0)
check('移动靶：静止目标不在容差内 → 不开火', not fires)
predictor = MovingPredictor(mp, DT)
fire, info = predictor.check(10.0, [])
check('看不到目标 → 不开火', not fire and info['reason'] == 'no_target')
fire, info = predictor.check(10.0, [(9.0 + 0.04 * k, 380.0, 193.0) for k in range(10)])
check('样本太旧 → 不开火', not fire and info['reason'] == 'stale')
fire, info = predictor.check(10.0, [(9.0, 380.0, 193.0), (9.95, 380.0, 193.0)])
check('窗口内样本不足（丢帧）→ 不用旧样本凑数、不开火', not fire and info['reason'] == 'few_samples', info['reason'])

# ---------------- 2026-10-25 移动靶三码融合（目标码没认出来的帧用同一滑块上的另外两个码补）----------------
from shoot_race.aiming import fuse_slider_codes  # noqa: E402
own6 = [(0.04 * k, 300.0 + 0.6 * k, 196.0) for k in range(10)]                 # 6 号码只认出前 10 帧（15 像素/秒）
code7 = [(0.04 * k, 326.8 + 0.6 * k, 196.5) for k in range(25)]                 # 7 号码 25 帧都认出来，右边 26.8 像素
fused = fuse_slider_codes({6: own6, 7: code7}, 6)
check('三码融合：目标码没认出来的帧用旁码按实测偏移换算，目标码自己认出来的帧不动',
      len(fused) == 25 and abs(fused[20][1] - (300.0 + 0.6 * 20)) < 0.01 and abs(fused[20][2] - 196.0) < 0.01
      and fused[5] == own6[5], 'n=%d u20=%.2f' % (len(fused), fused[20][1]))
mis7 = list(code7)
mis7[2] = (mis7[2][0], own6[2][1], 196.5)                                        # 两帧 7 号码认串成 6 号码的位置
mis7[4] = (mis7[4][0], own6[4][1], 196.5)
fused = fuse_slider_codes({6: own6, 7: mis7}, 6)
check('三码融合：偏移取中位数，认串的几帧不影响', abs(fused[20][1] - (300.0 + 0.6 * 20)) < 0.01, 'u20=%.2f' % fused[20][1])
fused = fuse_slider_codes({6: own6[:2], 7: code7}, 6)
check('三码融合：同一帧都认出来的不到 3 帧（偏移没量准）→ 先不用旁码', len(fused) == 2, 'n=%d' % len(fused))
fused = fuse_slider_codes({6: [], 7: code7}, 6, prior_px={6: 0.0, 7: 26.8})
check('三码融合：给了先验间距时，量不到偏移也能用旁码', len(fused) == 25 and abs(fused[3][1] - 301.8) < 0.01,
      'n=%d' % len(fused))

glare = dict(layout={6: 0.0, 7: 26.8, 8: 53.3}, blind_px=25.0)
solo = [run_moving_step(ph, intercept=False, seed=950 + i, fuse=False, **glare) for i, ph in enumerate(phases[::2])]
both = [run_moving_step(ph, intercept=False, seed=950 + i, fuse=True, **glare) for i, ph in enumerate(phases[::2])]
check('10-25 傍晚的情况：目标码在弹着点 ±25 像素内认不出来 → 只看目标码一次也打不了；三码融合每次都按时开火、误差 ≤ 2.5 像素',
      all(r[0] is None for r in solo) and all(r[0] is not None and abs(r[1]) <= 2.5 for r in both),
      '只看目标码 开火 %d/%d；融合 用时 %s 误差 %s' % (
          sum(r[0] is not None for r in solo), len(solo), ' '.join('%.1f' % (r[0] or -1) for r in both),
          ' '.join('%.1f' % (r[1] if r[1] is not None else 99) for r in both)))
clear = dict(layout={6: 0.0, 7: 26.8, 8: 53.3}, blind_px=None)
same = [(run_moving_step(ph, intercept=False, seed=990 + i, fuse=False, **clear),
         run_moving_step(ph, intercept=False, seed=990 + i, fuse=True, **clear)) for i, ph in enumerate(phases[::4])]
check('三码融合：目标码一直认得出时和以前一样（同一时刻开火、误差相同）',
      all(a[0] is not None and b[0] is not None and abs(a[0] - b[0]) <= 0.06 and abs(a[1] - b[1]) <= 0.3
          and abs(b[1]) <= 2.5 for a, b in same),
      ' '.join('%.2f/%.2f 秒 %.1f/%.1f 像素' % (a[0] or -1, b[0] or -1, a[1] or 99, b[1] or 99) for a, b in same))


def replay_moving_csv(name, fuse):
    """用 10-25 实车录的 AR 记录（tests/data/moving_*.csv）回放移动靶这一步：按收到时刻喂给预测，20 Hz。
    返回 (开火时刻或 None, 弹丸到达时码的真实位置 - 弹着点)；真实位置用整段记录（三码合起来）在到达时刻前后 0.4 秒拟合直线。"""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', name)
    with io.open(path, encoding='utf-8') as handle:
        target = int(re.search(r'目标码 (\d+)', handle.readline()).group(1))
    rows = np.loadtxt(path, delimiter=',', skiprows=2)
    ids = (6, 7, 8)

    def by_id(mask):
        sel = rows[mask]
        return dict((i, [(r[0], r[3], r[4]) for r in sel if int(r[2]) == i]) for i in ids)

    whole = np.array([(t, u) for (t, u, v) in fuse_slider_codes(by_id(rows[:, 0] > -1e9), target)])
    predictor = MovingPredictor(mp, DT)
    now = 0.0
    while now <= 16.4:
        avail = by_id(rows[:, 1] <= now)                           # 记录从这一步开始前 0.5 秒起
        if fuse:
            samples = fuse_slider_codes(avail, target, prior_px=mp['fuse_layout_px'])
        else:
            samples = avail[target]
        samples = [x for x in samples if x[0] >= -0.4]             # 车停稳以后的（这一步开始前约 0.4 秒停稳）
        fire, info = predictor.check(now, samples)
        if fire:
            t_hit = now + info.get('delay_s', 0.0) + mp['latency_s']
            near = whole[np.abs(whole[:, 0] - t_hit) <= 0.4]
            slope, u0 = np.polyfit(near[:, 0] - t_hit, near[:, 1], 1)
            return now, u0 - mp['aim_u_px']
        now += DT
    return None, None


import io  # noqa: E402
import re  # noqa: E402
real = dict((name, (replay_moving_csv(name, False), replay_moving_csv(name, True))) for name in (
    'moving_20261025_183142.csv', 'moving_20261025_182547.csv', 'moving_20261025_155609.csv'))
dark = [real['moving_20261025_183142.csv'], real['moving_20261025_182547.csv']]
good = real['moving_20261025_155609.csv']
check('10-25 实车回放：傍晚两局（目标码断档最长 7 秒 / 2 秒，当时 16 秒超时乱打）融合后按时开火、误差 ≤ 1.5 像素；'
      '下午那局（不断档）开火时刻和误差不变',
      all(a[0] is None and b[0] is not None and abs(b[1]) <= 1.5 for a, b in dark)
      and good[0][0] is not None and good[1][0] is not None and abs(good[0][0] - good[1][0]) <= 0.06
      and abs(good[1][1]) <= 1.0,
      '；'.join('%s 只看目标码 %s / 融合 %.2f 秒 误差 %.2f 像素' % (
          n[7:22], ('%.2f 秒' % a[0]) if a[0] is not None else '不开火', b[0] or -1, b[1] if b[1] is not None else 99)
          for n, (a, b) in sorted(real.items())))


# ---------------- 旋转靶（按 rotating.bag 实测：43 s 一圈、5 个码、半径 28.6 px）----------------
rp = dict(CFG['shooting']['rotating'])
rp['aim_u_px'], rp['aim_v_px'] = 336.4, 201.2                  # 测试用固定几何（弹着点在圆心左下），不随实弹标定变化
rp['fire_side'] = 'any'                                        # 下面的老用例不限左右；只打右侧的用例在后面单独测
rp['correction'] = 'shift'                                     # 老用例测平移；扭车头（2026-10-23）在后面单独测
WHEEL = dict(uc=353.3, vc=194.7, radius=28.6, omega=math.radians(8.4))


def wheel_uv(marker_id, t, phase0, shift_px=0.0):
    angle = phase0 + WHEEL['omega'] * t + 2 * math.pi * (marker_id - 1) / 5.0
    return (WHEEL['uc'] + shift_px + WHEEL['radius'] * math.cos(angle),
            WHEEL['vc'] + WHEEL['radius'] * math.sin(angle))


def run_rotating(phase0, target=3, deadline=60.0, misread=0.02, seed=0, distance_error=1.0, max_time=70.0,
                 no_wall=False):
    """闭环仿真：规划器发横向速度 → 车横向位置变化（起步延迟 0.4 s）→ 画面里转轮整体平移。

    distance_error：真实距离 / 程序以为的距离（换算误差，检验"平移完再看一次"的修正）。
    返回 (开火时刻或 None, 弹丸到达时目标离弹着点的像素距离, 日志 [(t, info)])。
    """
    rng = np.random.RandomState(seed)
    planner = RotatingPlanner(rp, SHIFT, FX, target, DT)
    lateral, origin = -0.30, -0.30
    samples = dict((k, []) for k in range(1, 6))
    log, now, next_frame = [], 0.0, 0.0
    moving_since, last_move = None, -10.0
    velocity = 0.0                                          # 车实际横向速度
    shift_px = lambda: FX * (lateral - origin) / (rp['target_distance_m'] * distance_error)   # noqa: E731
    while now < max_time:
        while next_frame <= now - 0.04:
            for k in range(1, 6):
                u, v = wheel_uv(k, next_frame, phase0, shift_px())
                reported = k if rng.rand() >= misread else int(rng.choice([j for j in range(1, 6) if j != k]))
                samples[reported].append((next_frame, u + rng.normal(0, 0.4), v + rng.normal(0, 0.4)))
            next_frame += 0.04
        recent = dict((k, [x for x in v if x[0] >= now - rp['history_s']]) for k, v in samples.items())
        still_for = now - last_move
        vx, vy, fire, info = planner.update(now, None if no_wall else lateral, recent, still_for >= rp['settle_s'],
                                        still_for, deadline)
        if not log or (log[-1][1].get('state'), log[-1][1].get('reason')) != (info.get('state'), info.get('reason')):
            log.append((now, info))
        if fire:
            u, v = wheel_uv(target, now + rp['latency_s'], phase0, shift_px())
            return now, math.hypot(u - rp['aim_u_px'], v - rp['aim_v_px']), log, lateral - origin
        if info.get('give_up'):
            return None, None, log, lateral - origin
        if vx != 0.0:                                       # keepalive 前后挪：只改距离，不影响横向位置和画面
            last_move = now
        if vy != 0.0:                                       # 底盘：指令持续 0.4 s 后才开始动（实测起步延迟）
            moving_since = now if moving_since is None else moving_since
            if now - moving_since >= 0.4:
                velocity = vy
            last_move = now
        else:
            moving_since = None
            velocity *= math.exp(-DT / 0.25)                # 松油门后还会滑（实测 0.05 m/s 时约 11 mm）
            if abs(velocity) > 1e-3:
                last_move = now
        lateral += velocity * DT
        now += DT
    return None, None, log, lateral - origin


# 单独测拟合：1 秒数据
rng = np.random.RandomState(1)
obs = dict((k, []) for k in range(1, 6))
for i in range(25):
    for k in range(1, 6):
        u, v = wheel_uv(k, 0.04 * i, 0.3)
        obs[k].append((0.04 * i, u + rng.normal(0, 0.4), v + rng.normal(0, 0.4)))
obs[2][10] = (obs[2][10][0],) + wheel_uv(4, obs[2][10][0], 0.3)        # 一帧把 4 号码认成 2 号
model, why = fit_wheel(obs)
check('转轮拟合：1 秒数据，圆心误差 < 1 px、转速误差 < 1 度/秒（含 1 帧误识别）',
      model is not None and math.hypot(model.uc - WHEEL['uc'], model.vc - WHEEL['vc']) < 1.0
      and abs(math.degrees(model.omega) - 8.4) < 1.0 and len(model.phases) == 5,
      '%s 圆心 (%.1f, %.1f) 转速 %.2f 度/秒' % (why, model.uc, model.vc, math.degrees(model.omega)) if model else why)

worst, waits, shifts = 0.0, [], []
for case, phase0 in enumerate(np.linspace(0, 2 * math.pi, 8, endpoint=False)):
    fired, error, log, shifted = run_rotating(phase0, seed=case)
    if fired is None:
        worst = float('inf')
        break
    worst = max(worst, error)
    waits.append(fired)
    shifts.append(shifted)
check('旋转靶：8 种初始相位都能打，弹丸到达时离弹着点 < 2 px', worst < 2.0,
      '最大误差 %.2f px，用时 %s 秒，平移 %s mm' % (worst, ' '.join('%.0f' % w for w in waits),
                                              ' '.join('%.0f' % (1000 * s) for s in shifts)))
check('旋转靶：最长用时不超过一圈多一点（43 秒 + 平移）', max(waits) < 48.0, '%.1f 秒' % max(waits))
check('旋转靶：只平移到够得着的左交点（约 +25 mm），不超过平移上限', all(0.015 < s < 0.035 for s in shifts),
      ' '.join('%.0f' % (1000 * s) for s in shifts))
fired, error, log, _ = run_rotating(1.0, distance_error=1.3, seed=7)
check('旋转靶：距离换算差 30% 时，平移后偏差在容差内就开火（不再来回平移）', fired is not None and error <= rp['hit_tolerance_px'],
      'error=%s 平移次数 %d' % (error, sum(1 for _, i in log if i.get('reason') == 'shift')))
fired, error, log, _ = run_rotating(1.0, distance_error=1.8, seed=7)
check('旋转靶：距离换算差 80% 时，平移后还偏就再修正一次', fired is not None and error <= rp['hit_tolerance_px']
      and sum(1 for _, i in log if i.get('reason') == 'shift') == 2,
      'error=%s 平移次数 %d' % (error, sum(1 for _, i in log if i.get('reason') == 'shift')))
# 弹着点离圆周不到容差：车一步不动，等目标码转到圆周上离弹着点最近的点开火（2026-10-17 实测横移开环误差大）
saved_aim = rp['aim_u_px'], rp['aim_v_px']
rp['aim_u_px'] = WHEEL['uc'] + (WHEEL['radius'] + 2.0) * math.cos(math.radians(150))
rp['aim_v_px'] = WHEEL['vc'] + (WHEEL['radius'] + 2.0) * math.sin(math.radians(150))
fired, error, log, shifted = run_rotating(1.0, seed=3)
rp['aim_u_px'], rp['aim_v_px'] = saved_aim
check('旋转靶：弹着点离圆周 2 像素 → 不平移，到时刻开火，误差 ≈ 2 像素', fired is not None and shifted == 0.0
      and error < 3.0 and not any(i.get('reason') == 'shift' for _, i in log),
      '开火 %s 秒，误差 %s px，平移 %.0f mm' % (fired, error, 1000 * shifted))
long_case = None
for phase0 in np.linspace(0, 2 * math.pi, 16, endpoint=False):
    fired, error, log, _ = run_rotating(phase0, seed=11)
    if fired is not None and fired > 25.0:
        long_case = (fired, error, sum(1 for _, i in log if i.get('reason') == 'keepalive'))
        break
check('旋转靶：等待超过 12 秒会 keepalive 前后挪，回来后照样打准',
      long_case is not None and long_case[2] >= 1 and long_case[1] < 2.0, str(long_case))
fired, error, log, shifted = run_rotating(2.0, no_wall=True)
best = min(abs(c['u'] - rp['aim_u_px']) for c in [
    {'u': WHEEL['uc'] - math.sqrt(WHEEL['radius'] ** 2 - (rp['aim_v_px'] - WHEEL['vc']) ** 2)}])
check('旋转靶：看不到侧墙（不能平移）→ 不平移，在最近点的时刻尽力开火',
      fired is not None and shifted == 0.0 and abs(error - best) < 2.0,
      '开火 %s 秒，误差 %.1f px（交点本身偏 %.1f px）' % (fired, error if error else -1, best))
saved_omega = WHEEL['omega']
WHEEL['omega'] = -saved_omega                                     # 今年转轮如果反着转
fired, error, log, shifted = run_rotating(1.0, seed=5)
WHEEL['omega'] = saved_omega
check('旋转靶：转轮反方向转也能打准', fired is not None and error < 2.0,
      '开火 %s 秒，误差 %s px，平移 %.0f mm' % (fired, error, 1000 * shifted))
# 只打右侧（fire_side: right，2026-10-18）：弹着点在轮毂附近，左右交点都要平移约一个半径。
# 平移上限放宽到 15 cm（两侧都够得着），不限左右时会按"等得最短"挑，有时打左侧；限右侧后每一枪都在圆心右边
saved = rp['aim_u_px'], rp['aim_v_px'], rp['max_shift_m'], rp['fire_side']
rp['aim_u_px'], rp['aim_v_px'], rp['max_shift_m'] = WHEEL['uc'] - 3.0, WHEEL['vc'] - 4.0, 0.15
sides, errors_px = {'any': [], 'right': []}, []
for mode in ('any', 'right'):
    rp['fire_side'] = mode
    for case, phase0 in enumerate(np.linspace(0, 2 * math.pi, 6, endpoint=False)):
        fired, error, log, shifted = run_rotating(phase0, seed=20 + case)
        if fired is None:
            sides[mode].append('none')
            continue
        center_u = WHEEL['uc'] + FX * shifted / rp['target_distance_m']
        u, _ = wheel_uv(3, fired + rp['latency_s'], phase0, FX * shifted / rp['target_distance_m'])
        sides[mode].append('right' if u > center_u else 'left')
        if mode == 'right':
            errors_px.append(error)
rp['aim_u_px'], rp['aim_v_px'], rp['max_shift_m'], rp['fire_side'] = saved
check('旋转靶 fire_side=right：每一枪都在转轮右半边开火，误差 < 2.5 px',
      sides['right'] == ['right'] * 6 and max(errors_px) < 2.5,
      'right: %s；误差 %s；对照 any: %s' % (' '.join(sides['right']), ' '.join('%.1f' % e for e in errors_px),
                                          ' '.join(sides['any'])))
# ---------------- 2026-10-23 旋转靶扭车头：左右交点都瞄，哪边先到打哪边 ----------------
from shoot_race.aiming import YawTwister  # noqa: E402
TWIST = CFG['system']['yaw_twist']


def run_twist(phase0, aim, target=3, k_true_scale=1.0, seed=0, max_time=70.0, deadline=60.0, omega_sign=1.0):
    """闭环仿真：规划器发角速度（扭车头）/ 横向速度 → 车头朝向、横向位置变化 → 画面里转轮整体平移。

    底盘：转向指令 0.25 秒后才起作用，实际角速度 = 0.3 × 指令（垫子），时间常数 0.3 秒；侧墙偏角带 0.15 度噪声。
    画面：车左转 ψ → 转轮右移 k_true × ψ 像素（k_true = 配置的 twist_px_per_deg × k_true_scale）；左移 Δy → 右移 FX×0.7×Δy/1.5。
    返回 (开火时刻, 弹丸到达时离弹着点像素, 开火那一侧, 扭的总角度（度）, 日志)。
    """
    rng = np.random.RandomState(seed)
    params = dict(CFG['shooting']['rotating'])
    params['aim_u_px'], params['aim_v_px'] = aim
    params['correction'] = 'twist'                     # v29 做法（现在默认是 servo，下面单独测）
    planner = RotatingPlanner(params, SHIFT, FX, target, DT, TWIST)
    k_true = math.degrees(1.0) * params['twist_px_per_deg'] * k_true_scale
    yaw, rate, lateral, velocity = 0.0, 0.0, -0.30, 0.0
    wz_hist = []
    samples = dict((k, []) for k in range(1, 6))
    log, now, next_frame, last_move = [], 0.0, 0.0, -10.0
    omega = WHEEL['omega'] * omega_sign

    def uv(k, t):
        angle = phase0 + omega * t + 2 * math.pi * (k - 1) / 5.0
        du = k_true * yaw + FX * 0.7 * (lateral + 0.30) / 1.5
        return (WHEEL['uc'] + du + WHEEL['radius'] * math.cos(angle), WHEEL['vc'] + WHEEL['radius'] * math.sin(angle))
    while now < max_time:
        while next_frame <= now - 0.04:
            for k in range(1, 6):
                u, v = uv(k, next_frame)
                samples[k].append((next_frame, u + rng.normal(0, 0.4), v + rng.normal(0, 0.4)))
            next_frame += 0.04
        recent = dict((k, [x for x in v if x[0] >= now - params['history_s']]) for k, v in samples.items())
        still_for = now - last_move
        vx, vy, fire, info = planner.update(now, lateral, recent, still_for >= params['settle_s'], still_for, deadline,
                                            yaw + rng.normal(0, math.radians(0.15)))
        if not log or (log[-1][1].get('state'), log[-1][1].get('reason')) != (info.get('state'), info.get('reason')):
            log.append((now, info))
        if fire:
            u, v = uv(target, now + params['latency_s'])
            angle = phase0 + omega * (now + params['latency_s']) + 2 * math.pi * (target - 1) / 5.0
            side = 'right' if math.cos(angle) > 0 else 'left'
            return now, math.hypot(u - params['aim_u_px'], v - params['aim_v_px']), side, math.degrees(yaw), log
        if info.get('give_up'):
            return None, None, None, math.degrees(yaw), log
        wz_hist.append((now, planner.wz))
        delayed = [w for (t, w) in wz_hist if t <= now - 0.25]
        cmd = delayed[-1] if delayed else 0.0
        rate += (0.3 * cmd - rate) * min(1.0, DT / 0.3)
        yaw += rate * DT
        velocity = vy * 0.9 if vy != 0.0 else velocity * math.exp(-DT / 0.25)
        lateral += velocity * DT
        if abs(rate) > math.radians(0.3) or abs(velocity) > 1e-3 or planner.wz != 0.0 or vy != 0.0 or vx != 0.0:
            last_move = now
        now += DT
    return None, None, None, math.degrees(yaw), log


def first_pass(phase0, side_angle, target=3, after=0.0, omega_sign=1.0):
    """目标码在 after 秒之后第一次转到 side_angle 的时刻。"""
    omega = WHEEL['omega'] * omega_sign
    angle0 = phase0 + 2 * math.pi * (target - 1) / 5.0 + omega * after
    turn = ((side_angle - angle0) * math.copysign(1.0, omega)) % (2 * math.pi)
    return after + turn / abs(omega)


# 弹着点在轮毂附近（圆心左 3、下 10 像素）：左右交点都要扭约 3 度，平移 5 cm 够不着
hub_aim = (WHEEL['uc'] - 3.0, WHEEL['vc'] + 10.0)
dv = hub_aim[1] - WHEEL['vc']
side_angles = {'left': math.atan2(dv, -math.sqrt(WHEEL['radius'] ** 2 - dv ** 2)),
               'right': math.atan2(dv, math.sqrt(WHEEL['radius'] ** 2 - dv ** 2))}
results, worst_err, max_twist_deg, sides_used, on_time = [], 0.0, 0.0, set(), 0
for case, phase0 in enumerate(np.linspace(0, 2 * math.pi, 8, endpoint=False)):
    fired, err, side, twisted, log = run_twist(phase0, hub_aim, seed=40 + case)
    if fired is None:
        results.append('none')
        worst_err = float('inf')
        continue
    worst_err = max(worst_err, err)
    max_twist_deg = max(max_twist_deg, abs(twisted))
    sides_used.add(side)
    # "哪边先到打哪边"：观察 1 秒 + 扭车头准备约 4 秒以后，左右交点哪个先到
    earliest = min(first_pass(phase0, side_angles[s], after=5.0) for s in ('left', 'right'))
    if fired <= earliest + 1.5:
        on_time += 1
    results.append('%s %.0fs %.1fpx %+.1f°' % (side, fired, err, twisted))
check('旋转靶扭车头：弹着点在轮毂附近（左右交点都要扭约 3 度），8 种相位都打得到、误差 ≤ 容差、左右两边都用上、扭不超过上限',
      worst_err <= CFG['shooting']['rotating']['hit_tolerance_px'] + 0.3 and sides_used == {'left', 'right'} and max_twist_deg <= CFG['shooting']['rotating']['max_twist_deg'] + 0.5,
      '; '.join(results))
check('旋转靶扭车头：多数情况打的是"算上准备时间先到的那一侧"（≥ 6/8）', on_time >= 6, '%d/8' % on_time)
fired, err, side, twisted, log = run_twist(1.0, hub_aim, k_true_scale=1.35, seed=60)
twist_count = sum(1 for _, i in log if i.get('reason') == 'twist')
check('旋转靶扭车头：换算系数差 35% 时，扭完重新看、再修一次，照样打准', fired is not None and err < 3.0,
      '误差 %s px，扭 %d 次，总共 %+.1f 度' % (err, twist_count, twisted))
fired, err, side, twisted, log = run_twist(2.5, hub_aim, seed=61, omega_sign=-1.0)
check('旋转靶扭车头：转轮反着转也能打', fired is not None and err < 3.0, '误差 %s px，%s，%+.1f 度' % (err, side, twisted))
tw = YawTwister(TWIST)
tw.start(math.radians(3.0), 0.0)
wz, status, _ = tw.update(0.0, 0.0)
wz2, status2, _ = tw.update(0.05, None)
check('YawTwister：要左转 3 度 → 发正的角速度（≥ 最小指令）；看不到侧墙 → 停', wz >= TWIST['min_rate'] and status == 'moving'
      and wz2 == 0.0 and status2 == 'no_wall', '%.2f %s / %s' % (wz, status, status2))

# ---------------- 2026-10-23 v30 旋转靶"边看边对"（RotatingServo）----------------
from shoot_race.aiming import RotatingServo, WheelTracker  # noqa: E402
from shoot_race.motion import hold_heading  # noqa: E402


def run_servo(ph, aim=(350.5, 199.5), target=3, seed=0, k_scale=1.0, yaw_gain=0.3, yaw_dead=0.25, lat_dead=0.35,
              omega_sign=1.0, pre_s=3.0, max_time=70.0, deadline=60.0, no_pose=False, omega=None, controller='servo',
              servo_over=None, breakaway=0.0, wheel=None, heading0_deg=None):
    """闭环仿真（底盘量级按实车）：
      原地转：指令 yaw_dead 秒后起作用，实际 = yaw_gain × 指令（垫子上 0.25~0.38），指令 < 0.15 转不动，时间常数 0.3 秒；
      横移：驱动截成整数 cm/s，lat_dead 秒后起作用，0.9 倍，时间常数 0.25 秒；横移时车头被带偏（右移 45 mm 左转 1 度）；
      画面：左转 ψ → 转轮右移 8.8 × k_scale 像素/度；左移 Δy → 右移 FX × 0.7 × Δy / 1.5；AR 约 25 帧/秒、延迟 0.04 秒、噪声 0.4 像素、2% 认错码；
      这一步开始前 pre_s 秒的画面也在缓存里（精定位最后那段）。
    controller='twist' 用 v29 的 RotatingPlanner（对比用）。
    返回 (开火时刻, 弹丸到达时离弹着点像素, 开火那一侧, 扭的角度（度）, 横移（米）, 日志, 控制器)。
    """
    rng = np.random.RandomState(seed)
    params = dict(CFG['shooting']['rotating'])
    params['aim_u_px'], params['aim_v_px'] = aim
    if controller == 'twist':
        params['correction'] = 'twist'
        ctl = RotatingPlanner(params, SHIFT, FX, target, DT, TWIST)
    else:
        servo = dict(params['servo'])
        servo.update(servo_over or {})
        ctl = RotatingServo(params, servo, target, DT)
        if heading0_deg is not None:
            ctl.set_heading_reference(-math.radians(heading0_deg))   # 车头正对侧墙时的里程计朝向（仿真里程计从 0 开始）
    k_true = math.degrees(1.0) * 8.8 * k_scale
    wl = dict(WHEEL, **(wheel or {}))
    w = (wl['omega'] if omega is None else omega) * omega_sign
    yaw, yaw_rate, lat, lat_vel = 0.0, 0.0, 0.0, 0.0
    hist, log = [], []
    samples = dict((k, []) for k in range(1, 6))
    now, next_frame, last_move, hold_ref = -pre_s, -pre_s, -10.0, None

    def uv(k, t):
        a = ph + w * t + 2 * math.pi * (k - 1) / 5.0
        du = k_true * yaw + FX * 0.7 * lat / 1.5
        return wl['uc'] + du + wl['radius'] * math.cos(a), wl['vc'] + wl['radius'] * math.sin(a)
    while now < max_time:
        while next_frame <= now - 0.04:
            for k in range(1, 6):
                u, v = uv(k, next_frame)
                rep = k if rng.rand() >= 0.02 else int(rng.choice([j for j in range(1, 6) if j != k]))
                samples[rep].append((next_frame, u + rng.normal(0, 0.4), v + rng.normal(0, 0.4)))
            next_frame += 0.04 * (1 + 0.2 * rng.rand())
        if now < 0.0:
            now += DT
            continue
        recent = dict((k, [x for x in v if x[0] >= now - 6.0]) for k, v in samples.items())
        still_for = now - last_move
        y_meas = None if no_pose else yaw + rng.normal(0, math.radians(0.05))
        l_meas = None if no_pose else lat + rng.normal(0, 0.0003)
        if controller == 'twist':
            recent = dict((k, [x for x in v if x[0] >= now - params['history_s']]) for k, v in recent.items())
            vx, vy, fire, info = ctl.update(now, l_meas, recent, still_for >= params['settle_s'], still_for, deadline,
                                            y_meas)
            if ctl.state == 'twist':
                wz, hold_ref = ctl.wz, None
            elif abs(vy) > 1e-6:
                hold_ref = y_meas if hold_ref is None else hold_ref
                wz = hold_heading(y_meas - hold_ref, deadband_rad=math.radians(0.3))
            else:
                wz, hold_ref = 0.0, None
        else:
            vx, vy, wz, fire, info = ctl.update(now, l_meas, y_meas, recent, still_for, deadline)
        if not log or (log[-1][1].get('state'), log[-1][1].get('reason')) != (info.get('state'), info.get('reason')):
            log.append((now, info))
        if fire:
            u, v = uv(target, now + params['latency_s'])
            a = ph + w * (now + params['latency_s']) + 2 * math.pi * (target - 1) / 5.0
            side = 'right' if math.cos(a) > 0 else 'left'
            return now, math.hypot(u - aim[0], v - aim[1]), side, math.degrees(yaw), lat, log, ctl
        if info.get('give_up'):
            return None, None, None, math.degrees(yaw), lat, log, ctl
        hist.append((now, vy, wz))
        cmd_w = ([h[2] for h in hist if h[0] <= now - yaw_dead] or [0.0])[-1]
        cmd_l = ([h[1] for h in hist if h[0] <= now - lat_dead] or [0.0])[-1]
        cmd_l = math.copysign(math.floor(abs(cmd_l) * 100 + 1e-9) / 100.0, cmd_l)
        in_place = abs(cmd_l) < 1e-3
        target_w = (0.0 if abs(cmd_w) < 0.15 else yaw_gain * cmd_w) if in_place else 0.9 * cmd_w
        if in_place and breakaway > 0 and abs(yaw_rate) < math.radians(0.5):
            # 原地静摩擦：从停着开始，指令要连续发够 breakaway 秒才转得起来（2026-10-23 实车：只发 0.4 秒车不动）
            held = 0.0
            for h in reversed(hist):
                if h[0] > now - yaw_dead:
                    continue
                if abs(h[2]) < 0.15:
                    break
                held += DT
            if held < breakaway:
                target_w = 0.0
        yaw_rate += (target_w - yaw_rate) * min(1.0, DT / 0.3)
        lat_vel += (0.9 * cmd_l - lat_vel) * min(1.0, DT / 0.25)
        yaw += yaw_rate * DT - math.radians(1.0) / 0.045 * lat_vel * DT
        lat += lat_vel * DT
        if abs(yaw_rate) > math.radians(0.3) or abs(lat_vel) > 1e-3 or vx or vy or wz:
            last_move = now
        now += DT
    return None, None, None, math.degrees(yaw), lat, log, ctl


SERVO = CFG['shooting']['rotating']['servo']
phases12 = np.linspace(0, 2 * math.pi, 12, endpoint=False)


def servo_batch(**kw):
    out = []
    for case, ph in enumerate(phases12):
        out.append(run_servo(ph, seed=200 + case, **kw))
    return out


def corner_mm(r):
    return 1000 * (abs(r[4]) + SERVO.get('corner_lever_m', 0.23) * abs(math.sin(math.radians(r[3]))))


base = servo_batch()
times = [r[0] for r in base if r[0] is not None]
errs = [r[1] for r in base if r[0] is not None]
check('旋转靶边看边对：弹着点 (350.5, 199.5)，12 种相位都打、误差 ≤ 容差、左右都用、扭/横移不超上限',
      len(times) == 12 and max(errs) <= CFG['shooting']['rotating']['hit_tolerance_px']
      and set(r[2] for r in base) == {'left', 'right'}
      and max(abs(r[3]) for r in base) <= CFG['shooting']['rotating']['max_twist_deg'] + 0.5
      and max(corner_mm(r) for r in base) <= 1000 * SERVO['max_corner_m'] + 8,
      '用时 %s；误差 %s；车角最多 %.0f mm' % (' '.join('%.0f' % t for t in times), ' '.join('%.1f' % e for e in errs),
                                       max(corner_mm(r) for r in base)))
# 交点约半圈一个（两侧各 43 秒一次），从随机相位开始平均要等约 10.75 秒 + 准备时间：平均 ≤ 15 秒、最长 ≤ 半圈 + 准备
check('旋转靶边看边对（2025 转轮 8.4 度/秒）：平均用时 ≤ 16 秒、最长 ≤ 27 秒（v29 实测 20~35 秒）', np.mean(times) <= 16.0 and max(times) <= 27.0,
      '平均 %.1f 最长 %.1f 秒' % (np.mean(times), max(times)))
twist_slow = [run_servo(ph, seed=200 + case, controller='twist', k_scale=1.35, yaw_gain=0.38)
              for case, ph in enumerate(phases12[:6])]
servo_slow = [run_servo(ph, seed=200 + case, k_scale=1.35, yaw_gain=0.38) for case, ph in enumerate(phases12[:6])]
t_twist = np.mean([r[0] if r[0] is not None else 70.0 for r in twist_slow])
t_servo = np.mean([r[0] if r[0] is not None else 70.0 for r in servo_slow])
check('对比 v29：底盘/换算和设定不一样时（扭得快 35%、原地转 0.38 倍）v30 比 v29 快、照样打准',
      t_servo < t_twist and all(r[0] is not None and r[1] <= 4.5 for r in servo_slow),
      'v29 平均 %.1f 秒 / v30 平均 %.1f 秒，v30 误差 %s' % (t_twist, t_servo, ' '.join('%.1f' % r[1] for r in servo_slow)))
robust = []
for name, kw in (('扭得慢 30%', dict(k_scale=0.7)), ('扭得快 40%', dict(k_scale=1.4)), ('原地转 0.25 倍', dict(yaw_gain=0.25)),
                 ('死区 0.4/0.5 秒', dict(yaw_dead=0.4, lat_dead=0.5)), ('反着转', dict(omega_sign=-1.0)),
                 ('进来前没画面', dict(pre_s=0.0))):
    rs = [run_servo(ph, seed=300 + case, **kw) for case, ph in enumerate(phases12[::2])]
    ok = all(r[0] is not None and r[1] <= 4.5 for r in rs)
    robust.append((name, ok, np.mean([r[0] or 70.0 for r in rs]), max(r[1] or 99.0 for r in rs)))
check('旋转靶边看边对：底盘、换算系数、转向和设定不一样也都打准（≤ 4.5 像素）、平均 ≤ 17 秒',
      all(ok and t <= 17.0 for _, ok, t, _ in robust),
      '；'.join('%s %s %.1fs %.1fpx' % (n, 'ok' if ok else 'FAIL', t, e) for n, ok, t, e in robust))
# 2026-10-23 实车（v30 第一版）：转轮 15.3 度/秒、圆心 u≈378（右交点要修约 55 像素 ≈ 6.3 度）、原地转只出 0.2 倍、
# 原地起步要连续发约 0.5 秒指令。那一版按"最大指令 × 滑行时间"给上限留了 5.4 度余量，扭过 0.6 度后每次一扭就判到上限、
# 车没动就松开，卡在 -3.8 度，码到了右交点也不开火，改打另一侧（17 / 22 / 43 秒）。
real_wheel = dict(uc=378.0, vc=191.0, radius=28.5, omega=math.radians(15.3))
real = [run_servo(ph, seed=500 + case, pre_s=0.0, yaw_gain=0.2, yaw_dead=0.3, breakaway=0.5, wheel=real_wheel)
        for case, ph in enumerate(phases12)]
check('实车情况（转轮 15 度/秒、右交点要扭 6 度多、原地起步慢）：不再卡在上限，12 种相位都不放过交点、≤ 4 像素、最长 ≤ 17 秒、扭不超上限 0.5 度',
      all(r[0] is not None and r[1] <= 4.0 and r[6].misses == 0 for r in real) and max(r[0] for r in real) <= 17.0
      and max(abs(r[3]) for r in real) <= CFG['shooting']['rotating']['max_twist_deg'] + 0.5,
      '用时 %s；误差 %s；放过 %s；扭 %s 度' % (' '.join('%.0f' % (r[0] or -1) for r in real),
                                     ' '.join('%.1f' % (r[1] or -1) for r in real), ' '.join(str(r[6].misses) for r in real),
                                     ' '.join('%.1f' % r[3] for r in real)))
k_learned = [r[6].k_px_rad / math.degrees(1.0) for r in real if abs(r[3]) > 2.0]
check('扭过以后学到的换算系数接近真实值 8.8 像素/度（±20%）', k_learned and all(7.0 <= k <= 10.6 for k in k_learned),
      ' '.join('%.1f' % k for k in k_learned))
r = run_servo(phases12[1], seed=77, pre_s=0.0, yaw_gain=0.2, yaw_dead=0.3, breakaway=0.5,
              wheel=dict(real_wheel, uc=393.0))
plans = [i for _, i in r[5] if i.get('reason') == 'plan']
check('交点远到扭 + 横移都够不着时，规划直接选够得着的那一侧（不去白扭）', r[0] is not None and r[1] <= 4.0
      and plans and plans[0].get('side') == 'left' and r[6].misses == 0,
      '第一次选 %s（右交点要 %.0f 像素）用时 %.1f 秒' % (plans[0].get('side') if plans else '-', 393.0 + 28.5 - 350.5, r[0] or -1))

# 2026-10-23 18:58 实车：车摆得很偏（转轮圆心 u≈216，离弹着点 135 像素，扭 + 横移都够不着）。
# 那一版扭冲到 8.3 度、之后原地反复"对准 / 到上限"，27 秒放弃没开枪。现在：扭不超上限，到点尽力开一枪
far = [run_servo(ph, seed=600 + case, pre_s=0.0, yaw_gain=0.2, yaw_dead=0.3, breakaway=0.5,
                 wheel=dict(real_wheel, uc=216.0)) for case, ph in enumerate(phases12[::3])]
check('车摆得很偏、又测不到侧墙（只能按开始时的车头限扭）：扭不超上限 0.7 度、不来回折腾，30 秒内尽力开一枪',
      all(r[0] is not None and r[0] <= 30.0 for r in far)
      and max(abs(r[3]) for r in far) <= CFG['shooting']['rotating']['max_twist_deg'] + 0.7
      and all(len(r[5]) < 60 for r in far),
      '开火 %s 秒；扭 %s 度；日志 %s 行' % (' '.join('%.0f' % (r[0] or -1) for r in far),
                                     ' '.join('%.1f' % r[3] for r in far), ' '.join(str(len(r[5])) for r in far)))

# 同样摆歪约 15 度，但看得到侧墙（实车 18:58 那次激光算出车头偏约 15.6 度）：扭的上限按"正对侧墙"算，往回扭不受限 → 扭回来正常打
back = [run_servo(ph, seed=650 + case, pre_s=0.0, yaw_gain=0.2, yaw_dead=0.3, breakaway=0.5,
                  wheel=dict(real_wheel, uc=216.0), heading0_deg=-15.0) for case, ph in enumerate(phases12[::2])]
check('车摆歪 15 度（转轮圆心偏到 u≈216）、看得到侧墙：扭回来，误差 ≤ 4 像素、最长 ≤ 30 秒，车头相对侧墙不超过 6.5 度',
      all(r[0] is not None and r[1] <= 4.0 and r[0] <= 30.0 for r in back)
      and all(abs(r[3] - 15.0) <= CFG['shooting']['rotating']['max_twist_deg'] + 0.5 for r in back),
      '用时 %s；误差 %s；扭回 %s 度（车头相对侧墙 %s 度）' % (
          ' '.join('%.0f' % (r[0] or -1) for r in back), ' '.join('%.1f' % (r[1] or -1) for r in back),
          ' '.join('%.1f' % r[3] for r in back), ' '.join('%.1f' % (r[3] - 15.0) for r in back)))

# 码快到了、来不及对准：不开这一枪（偏差大），改打另一侧。故意把准备时间估得很乐观，让它去追 1.5 秒后就到的右交点（要修约 31 像素）
dv = 199.5 - WHEEL['vc']
right_angle = math.atan2(dv, math.sqrt(WHEEL['radius'] ** 2 - dv ** 2))
ph_rush = right_angle - WHEEL['omega'] * 1.5 - 2 * math.pi * 2 / 5.0
r = run_servo(ph_rush, seed=7, servo_over={'prep_base_s': 0.0, 'servo_px_s': 1000.0, 'hold_s': 0.0})
first = [i for _, i in r[5] if i.get('reason') in ('missed', 'fire')]
check('来不及对准时不硬打（偏差大就不开这一枪，改打另一侧），开火时照样准',
      r[0] is not None and r[0] > 10.0 and r[1] <= 4.0 and first and first[0].get('reason') == 'missed' and r[2] == 'left',
      '开火 %.1f 秒 %s 误差 %.1f px；第一次到点：%s 偏 %.1f px' % (
          r[0] or -1, r[2], r[1] or -1, first[0].get('reason') if first else '-', first[0].get('miss_px', -1) if first else -1))
# 等得久：keepalive 前后挪，回来接着对准
long_r = None
for case, ph in enumerate(phases12):
    rr = run_servo(ph, seed=400 + case)
    if rr[0] is not None and rr[0] > 18.0:
        long_r = rr
        break
check('旋转靶边看边对：等待超过 12 秒会 keepalive 前后挪，回来后照样打准',
      long_r is not None and any(i.get('reason') == 'keepalive' for _, i in long_r[5]) and long_r[1] <= 4.0,
      '开火 %.1f 秒 误差 %.1f' % (long_r[0], long_r[1]) if long_r else 'none')
# 拿不到车头/横向位置（里程计、侧墙都没有）：不动车，弹着点正好在左交点上照样打
dv = 199.5 - WHEEL['vc']
left_u = WHEEL['uc'] - math.sqrt(WHEEL['radius'] ** 2 - dv ** 2)
r = run_servo(phases12[3], seed=9, aim=(left_u + 1.0, 199.5), no_pose=True)
check('拿不到车头/横向位置 → 不扭不移，只等对得上的交点（左）打', r[0] is not None and r[1] <= 3.0 and abs(r[3]) < 0.3
      and r[2] == 'left' and not any(i.get('reason') == 'missed' for _, i in r[5])
      and abs(r[4]) < 0.003, '开火 %s 误差 %s 扭 %.1f 度 移 %.0f mm' % (r[0], r[1], r[3], 1000 * r[4]))
# 转轮停着：对准码现在的位置就打
r = run_servo(phases12[2], seed=11, omega=0.0, aim=(350.5, 199.5))
check('转轮不转：对准码现在的位置开火（高低差只能尽力）', r[0] is not None and r[0] < 10.0,
      '开火 %s 秒 误差 %s' % (r[0], r[1]))
# 转轮跟踪：车一边扭（画面 40 像素/秒）一边看，转速、圆心照样准
rng = np.random.RandomState(5)
tracker = WheelTracker(3, 3.0, math.radians(0.5), 6.0)
obs = dict((k, []) for k in range(1, 6))
for i in range(75):
    t = 0.04 * i
    shift = 40.0 * t
    for k in range(1, 6):
        u, v = wheel_uv(k, t, 0.3, shift)
        obs[k].append((t, u + rng.normal(0, 0.4), v + rng.normal(0, 0.4)))
model, why = tracker.update(obs, 3.0)
check('转轮跟踪：车扭着（画面 40 像素/秒）看 3 秒，转速误差 < 0.5 度/秒、圆心是"现在"的位置（最近 3 帧中位数，< 1 像素）、画面速度 ≈ 40',
      model is not None and abs(math.degrees(model.omega) - 8.4) < 0.5
      and abs(model.uc - (WHEEL['uc'] + 40.0 * 2.92)) < 1.0 and abs(tracker.center_rate() - 40.0) < 4.0,
      '%s 转速 %.2f 圆心 %.1f（应 %.1f）速度 %.1f' % (why, math.degrees(model.omega) if model else -1, model.uc if model else -1,
                                             WHEEL['uc'] + 40.0 * 2.92, tracker.center_rate()))
model_old, _ = fit_wheel(obs)
check('对照：老的 fit_wheel 车动着拟合就不行（所以 v29 每动一次都要停下来重新看）',
      model_old is None or abs(math.degrees(model_old.omega) - 8.4) > 0.5 or abs(model_old.uc - (WHEEL['uc'] + 118.4)) > 3.0,
      '老拟合 %s' % ('失败' if model_old is None else '转速 %.1f 圆心 %.1f' % (math.degrees(model_old.omega), model_old.uc)))

fired, error, log, _ = run_rotating(phase0, deadline=3.0)
check('旋转靶：等待时间超过最晚开火时刻 → 放弃', fired is None and log[-1][1].get('give_up'), str(log[-1][1]))
first_confirm = min([t for t, i in log if i.get('reason') == 'confirm_give_up'] or [None])
give_up_t = log[-1][0]
check('旋转靶：放弃前先多观察到 give_up_observe_s（%.1f 秒）再确认' % rp['give_up_observe_s'],
      first_confirm is not None and give_up_t - first_confirm >= rp['give_up_observe_s'] - rp['observe_s'] - 0.1,
      '第一次想放弃 %.2f 秒，真正放弃 %.2f 秒' % (first_confirm or -1, give_up_t))


# ---------------- 豆包回答解析 ----------------
aliases = CFG['recognition']['fruit_aliases']
check('解析 JSON 回答', parse_vlm_reply('{"answer": 4, "fruit": "山竹"}', aliases) == (4, 'mangosteen'))
check('解析带多余文字的回答', parse_vlm_reply('答案如下：{"answer":2,"fruit":"杨桃"}。', aliases) == (2, 'starfruit'))
check('解析非 JSON 回答', parse_vlm_reply('题目答案是5，图中是榴莲', aliases) == (5, 'durian'))
check('答案超出 1~5 → 无效', parse_vlm_reply('{"answer": 7, "fruit": "山竹"}', aliases) is None)
check('认不出水果 → 无效', parse_vlm_reply('{"answer": 3, "fruit": "苹果"}', aliases) is None)


# ---------------- 识别策略 ----------------
class FakeMatcher(object):
    def __init__(self, index, score, margin):
        self.result = {'index': index, 'score': score, 'margin': margin}

    def match(self, image):
        return dict(self.result)


class FakeVlm(object):
    def __init__(self, reply=None, fail=False):
        self.reply, self.fail = reply, fail

    def is_configured(self):
        return True

    def ask(self, image):
        if self.fail:
            raise IOError('timeout')
        return self.reply


CardRecognizer._run_in_thread = staticmethod(lambda job, image: job(image))   # 测试里同步执行
rec_cfg = CFG['recognition']


def recognize(policy, matcher, vlm, wait=0.0):
    cfg = dict(rec_cfg)
    cfg['policy'] = policy
    r = CardRecognizer(cfg, '', matcher=matcher, vlm=vlm)
    r.start(None, 0.0)
    return r.poll(wait)


res = recognize('vlm_first', FakeMatcher(0, 0.9, 0.5), FakeVlm('{"answer": 3, "fruit": "杨桃"}'))
check('vlm_first：豆包成功 → 用豆包', res['source'] == 'vlm' and res['rotating'] == 3 and res['moving'] == 7, str(res))
res = recognize('vlm_first', FakeMatcher(7, 0.6, 0.2), FakeVlm(fail=True))
check('vlm_first：豆包失败 → 用可信的离线结果', res['source'] == 'offline' and res['rotating'] == 3, str(res))
res = recognize('vlm_first', FakeMatcher(7, 0.2, 0.01), FakeVlm(fail=True))
check('vlm_first：都不可信 → 离线最佳猜测', res['source'] == 'offline_guess', str(res))
res = recognize('vlm_first', None, FakeVlm(fail=True))
check('vlm_first：没有离线、豆包失败 → 默认靶号', res['source'] == 'default', str(res))
res = recognize('offline_first', FakeMatcher(14, 0.7, 0.3), FakeVlm('{"answer": 1, "fruit": "榴莲"}'))
check('offline_first：离线可信 → 立即用离线', res['source'] == 'offline' and res['rotating'] == 5, str(res))

# ---------------- 离线模板匹配（合成照片，15 张卡各 2 次）----------------
from test_mission_sim import synth_card_photo  # noqa: E402

cards_dir = os.path.join(PACKAGE_DIR, 'config', 'cards')
matcher = CardMatcher(rec_cfg['cards'], cards_dir, rec_cfg['offline'])
correct = confident_wrong = 0
for index, card in enumerate(rec_cfg['cards']):
    for seed in (1, 2):
        result = matcher.match(synth_card_photo(os.path.join(cards_dir, card['file']), seed=seed))
        confident = result['score'] >= rec_cfg['offline']['min_score'] and result['margin'] >= rec_cfg['offline']['min_margin']
        correct += result['index'] == index
        confident_wrong += (result['index'] != index) and confident
check('离线匹配：30 张合成照片的准确率', correct >= 28, '%d/30' % correct)
check('离线匹配：没有"自信地认错"', confident_wrong == 0, '%d 张' % confident_wrong)

# ---------------- 配置检查 ----------------
check('当前配置无错误', not check_config(CFG), str(check_config(CFG)))
bad = load_config(os.path.join(PACKAGE_DIR, 'config'))
bad['mission']['steps'][0]['timeout_s'] = 25
check('timeout_s ≥ 20 会被拦下', any('20' in e for e in check_config(bad)))
bad = load_config(os.path.join(PACKAGE_DIR, 'config'))
bad['shooting']['rotating']['keepalive_nudge_s'] = 0.0
check('旋转靶关掉 keepalive 后 timeout_s ≥ 20 会被拦下', any('shoot_rot' in e for e in check_config(bad)))
bad = load_config(os.path.join(PACKAGE_DIR, 'config'))
bad['mission']['steps'][1]['path'] = 'no_such_path'
check('引用不存在的路径会被拦下', any('no_such_path' in e for e in check_config(bad)))

# 临时改参数（race.launch set:=...）
import copy  # noqa: E402
from shoot_race.config import apply_overrides  # noqa: E402
over = copy.deepcopy(CFG)
changes = apply_overrides(over, 'system.path_follower.max_speed=0.2；shooting.ring.mode=taught')
check('set:= 临时改参数：数字、字符串都按 yaml 解析，返回改动列表',
      over['system']['path_follower']['max_speed'] == 0.2 and over['shooting']['ring']['mode'] == 'taught'
      and len(changes) == 2 and CFG['shooting']['ring']['mode'] == 'vision', str(changes))
try:
    apply_overrides(copy.deepcopy(CFG), 'system.path_follower.maxspeed=0.2')
    typo_caught = False
except ValueError:
    typo_caught = True
check('set:= 参数名写错直接报错（不会以为改了其实没改）', typo_caught)
bad_mode = copy.deepcopy(CFG)
bad_mode['shooting']['ring']['mode'] = 'teach'
check('ring.mode 写错会被配置检查拦下', any('ring.mode' in e for e in check_config(bad_mode)))
bad_side = copy.deepcopy(CFG)
bad_side['shooting']['rotating']['fire_side'] = 'rigth'
check('rotating.fire_side 写错会被配置检查拦下', any('fire_side' in e for e in check_config(bad_side)))
check('现在的配置：旋转靶左右都瞄、边看边对（2026-10-23 v30 servo）', CFG['shooting']['rotating'].get('fire_side') == 'any'
      and CFG['shooting']['rotating'].get('correction') == 'servo' and not check_config(CFG))
bad_servo = copy.deepcopy(CFG)
bad_servo['shooting']['rotating']['servo']['yaw_min_rate'] = 0.6
bad_servo['shooting']['rotating']['servo']['tolerance_px'] = 6.0
errs = check_config(bad_servo)
check('rotating.servo 参数不合理（最小转速 > 最大、对准容差 > 开火容差）会被配置检查拦下',
      any('yaw_min_rate' in e for e in errs) and any('tolerance_px' in e for e in errs), '; '.join(errs))
bad_corr = copy.deepcopy(CFG)
bad_corr['shooting']['rotating']['correction'] = 'servoo'
check('rotating.correction 写错会被配置检查拦下', any('correction' in e for e in check_config(bad_corr)))
# 弹道测试的像素对准（菜单 22）：横移在画面里只起理论值 40% 的作用时也要收敛
from shoot_race.aiming import PixelAligner  # noqa: E402


def run_pixel_align(efficiency, start_err_px, seed=0, max_total=0.15):
    rng = np.random.RandomState(seed)
    theory = FX / 1.5
    aligner = PixelAligner(360.0, theory, 2.0, max_total, 6)
    lateral, u0 = 0.0, 360.0 - start_err_px
    for _ in range(10):
        u = u0 + efficiency * theory * lateral + rng.normal(0, 0.4)
        state, target, info = aligner.plan(u, lateral + rng.normal(0, 0.001))
        if state != 'shift':
            return state, info['err_px'], info['iteration']
        lateral = target + rng.normal(0, 0.003)          # 横移本身有 ±3 mm 误差
    return 'loop', None, None


results = [run_pixel_align(eff, 12.0, seed) for eff, seed in ((1.0, 1), (0.4, 2), (0.6, 3))]
check('弹道测试细调：粗调后差 12 像素，横移效率 100% / 40% / 60% 都在 6 次内对到 ±2 像素',
      all(r[0] == 'done' and abs(r[1]) <= 2.0 for r in results), str(results))
state, err, _ = run_pixel_align(0.4, 200.0, 4)
check('弹道测试对准：差太多、顶到平移上限 → 放弃（不无限平移）', state == 'give_up', '%s %s' % (state, err))
from shoot_race.config import turn_update  # noqa: E402
turn_cfg = copy.deepcopy(CFG)
turn_cfg['field']['boxes']['p2']['yaw_ref_deg'] = -2.58
turn_cfg['field']['waypoints']['p2'] = [0.972, -1.471, 0.5]
update, notes = turn_update(turn_cfg, 'p2', -3.4)
check('teach.py turn p2 -3.4：精定位朝向基准和同名路径点朝向都向右转 3.4 度，墙距不动',
      update == {'boxes': {'p2': {'yaw_ref_deg': -5.98}}, 'waypoints': {'p2': [0.972, -1.471, -2.9]}},
      '; '.join(notes))

# ---------------- 开火测试记录：电压、每枪出弹数（2026-10-18） ----------------
import io  # noqa: E402
import shutil  # noqa: E402
from shoot_race.shotfile import (add_beads_column, beads_summary, parse_beads, parse_rings,  # noqa: E402
                                 parse_voltage, rings_summary)

check('电量表读数：8.1 / 8.1V / 回车 都能解析', parse_voltage('8.1') == 8.1 and parse_voltage(' 8.1v ') == 8.1
      and parse_voltage('') is None)
bad_inputs = []
for text in ('81', 'abc', '3.0'):
    try:
        parse_voltage(text)
    except ValueError:
        bad_inputs.append(text)
check('电量表读数输错（81、abc、3.0）会被拦下', bad_inputs == ['81', 'abc', '3.0'], str(bad_inputs))
check('出弹数：空格、逗号、中文逗号都能隔开', parse_beads('1 0,2，1') == [1, 0, 2, 1] and parse_beads('') == [])
try:
    parse_beads('1 x 1')
    beads_typo = False
except ValueError:
    beads_typo = True
check('出弹数里有非数字会报错', beads_typo)
check('出弹统计', beads_summary([1, 0, 2, 1]) == '出弹 3/4 枪（共 4 颗），空发 1 枪', beads_summary([1, 0, 2, 1]))
try:
    parse_rings('9 11')
    ring_range = False
except ValueError:
    ring_range = True
check('报靶环数：0~10，超出范围报错', parse_rings('9，10 0') == [9, 10, 0] and parse_rings('') == [] and ring_range)
check('报靶统计（脱靶不算进平均）', rings_summary([8, 10, 0, 9]) == '上靶 3/4 枪，上靶平均 9.0 环（共 27 环）'
      and rings_summary([0, 0]) == '上靶 0/2 枪', rings_summary([8, 10, 0, 9]))

tmp_dir = tempfile.mkdtemp()
csv_path = os.path.join(tmp_dir, 'shots.csv')
with io.open(csv_path, 'w', encoding='utf-8') as handle:
    handle.write('shot,clock,ros_time,aim_u,aim_v,id6_u,id6_v,id7_u,id7_v,id8_u,id8_v\n'
                 '1,20:10:13.502,1.0,366.0,214.0,,,,,379.9,196.2\n'
                 '2,20:10:17.505,5.0,366.0,214.0,,,352.7,198.3,379.9,196.2\n'
                 '3,20:10:21.508,9.0,366.0,214.0,,,,,379.9,196.2\n')
try:
    add_beads_column(csv_path, [1, 0])
    count_mismatch = False
except ValueError:
    count_mismatch = True
unchanged = 'beads' not in io.open(csv_path, encoding='utf-8').readline()
check('出弹数个数和枪数对不上时不改文件（防止错位）', count_mismatch and unchanged)
add_beads_column(csv_path, [1, 0, 2])
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'scripts'))
import aim_calib  # noqa: E402
rows = aim_calib.read_shots(csv_path)
check('加了 beads 列后菜单 20 照样读得出码的位置，并知道第 2 枪没出弹',
      len(rows) == 3 and rows[1]['beads'] == 0 and rows[2]['beads'] == 2 and rows[1]['ids'][7] == (352.7, 198.3),
      str([(r['shot'], r['beads'], sorted(r['ids'])) for r in rows]))
try:
    import shot_log  # noqa: E402
except ImportError:                     # 电脑上没有 ROS：用空模块代替（这里只测不碰 ROS 的函数）
    import types
    for name in ('rospy', 'shoot_race.ros_io'):
        sys.modules.setdefault(name, types.ModuleType(str(name)))
    sys.modules['shoot_race.ros_io'].RosIO = object
    import shot_log  # noqa: E402
os.environ['HOME'] = tmp_dir
os.makedirs(os.path.join(tmp_dir, 'race_logs'))
ring_dir, ring_csv = shot_log.new_log_dir('aimtest', 'ring', 5, 2.0, 0.1, (365.0, None), None, 'target_id: ring\n')
ring_header = io.open(ring_csv, encoding='utf-8').readline().strip()
check('环形靶记录：shots.csv 有 ring_u 列（不再以逗号结尾），菜单 20 自动跳过它',
      ring_header == 'shot,clock,ros_time,aim_u,aim_v,ring_u' and aim_calib.is_ring_log(ring_dir)
      and not aim_calib.is_ring_log(tmp_dir), ring_header)


class FakeRing(object):
    def ring_observations(self, max_age, ids):
        return [(9.0, 300.0), (9.6, 362.0), (10.0, 364.0), (10.3, 366.0), (10.5, 999.0)]


from shoot_race.ring_finder import RingSource as _RingSource  # noqa: E402
check('开火时靶心取开火前 1.2 秒到开火后 0.45 秒的中位数（find_object）',
      shot_log.ring_at(FakeRing(), _RingSource({'detector': 'find_object', 'accept_ids': [31]}), 10.0) == 364.0)
shutil.rmtree(tmp_dir)

# ---------------- 2026-10-20 16:19 菜单 10 录包复盘后的改动 ----------------
fence_y = np.linspace(-1.5, 0.5, 400)
stop_fast = 0.08 + 0.5 ** 2 / 2.0 + 0.5 * 0.15
check('安全检查：车尾后 12 cm 的后围栏，横移 0.45 m/s 带 0.05 m/s 后退纠偏 → 不挡（原来按"最靠前的角"算会误挡）',
      not obstacle_in_path(np.full_like(fence_y, -0.30), fence_y, VelocityCommand(-0.05, -0.45, 0), 0.23, 0.18, 0.16, 0.03,
                           stop_fast))
check('安全检查：起步时后围栏贴着车尾（0.5 cm），往右前方走 → 不挡（激光点落在车角上不算撞）',
      not obstacle_in_path(np.full_like(fence_y, -0.185), fence_y, VelocityCommand(0.06, -0.065, 0), 0.23, 0.18, 0.16,
                           0.03, 0.08))
check('安全检查：同样贴着后围栏，往后退 → 挡',
      obstacle_in_path(np.full_like(fence_y, -0.185), fence_y, VelocityCommand(-0.05, 0, 0), 0.23, 0.18, 0.16, 0.03, 0.08))
check('安全检查：横移时挡板端头离车头 2 cm、在前方 14 cm 处擦过 → 挡；离 5 cm → 不挡',
      obstacle_in_path(np.array([0.25]), np.array([-0.30]), VelocityCommand(0, -0.4, 0), 0.23, 0.18, 0.16, 0.03, 0.22)
      and not obstacle_in_path(np.array([0.28]), np.array([-0.30]), VelocityCommand(0, -0.4, 0), 0.23, 0.18, 0.16, 0.03,
                               0.22))

# 2026-10-20 17:57 整局：激光每帧都扫到车尾右侧的零件（机器人坐标约 (-0.115, -0.047)，陷进车身轮廓 6.6 cm）
self_x, self_y = np.array([-0.115, -0.115, -0.113, -0.113]), np.array([-0.045, -0.047, -0.048, -0.050])
check('安全检查：只有车自己的零件（车尾轮廓里 6.6 cm 深），去 1 号点那条右移带后退的指令 → 不挡（17:57 被它挡到超时）',
      not obstacle_in_path(self_x, self_y, VelocityCommand(-0.024, -0.165, -1.0), 0.23, 0.18, 0.16, 0.03, 0.08))
check('安全检查：车尾边上的零件（16:19 录包，中心后方 14.3 cm）在 rear_length 0.18 下算车自己的，往后退不挡'
      '（填实测的 0.15 就会被当成障碍）',
      not obstacle_in_path(np.array([-0.143]), np.array([0.082]), VelocityCommand(-0.05, -0.3, 0), 0.23, 0.18, 0.155, 0.03, 0.08)
      and obstacle_in_path(np.array([-0.143]), np.array([0.082]), VelocityCommand(-0.05, -0.3, 0), 0.23, 0.15, 0.155, 0.03, 0.08))
check('安全检查：车自己的零件 + 贴着车尾的后围栏，往后退 → 照样挡；往前走 → 不挡',
      obstacle_in_path(np.r_[self_x, np.full_like(fence_y, -0.185)], np.r_[self_y, fence_y], VelocityCommand(-0.05, 0, 0),
                       0.23, 0.18, 0.16, 0.03, 0.08)
      and not obstacle_in_path(np.r_[self_x, np.full_like(fence_y, -0.185)], np.r_[self_y, fence_y],
                               VelocityCommand(0.05, 0, 0), 0.23, 0.18, 0.16, 0.03, 0.08))
check('安全检查：车自己的零件 + 横移时挡板端头离车头 2 cm 擦过 → 照样挡（车身里的点不能把"擦过"判断关掉）',
      obstacle_in_path(np.r_[self_x, 0.25], np.r_[self_y, -0.30], VelocityCommand(0, -0.4, 0), 0.23, 0.18, 0.16, 0.03, 0.22))

# ---------------- 2026-10-21 每局用时表（RunLog.summary）----------------
from shoot_race.runlog import RunLog  # noqa: E402
sum_clock = [0.0]
sum_dir = tempfile.mkdtemp()
sum_log = RunLog(sum_dir, printer=lambda text: None, clock=lambda: sum_clock[0])
sum_log.start_match()
for t_, step_, event_, detail_ in [(0.0, 'go_recog', 'start', ''), (1.0, 'go_recog', 'blocked', 'x'),
                                   (3.9, 'go_recog', 'done', '用时 3.90 秒'), (3.9, 'align_recog', 'start', ''),
                                   (5.6, 'align_recog', 'done', '用时 1.70 秒'),
                                   (5.6, 'align_recog', 'align_error', 'front=-1.2cm'),
                                   (5.6, 'go_p1', 'start', ''), (13.6, 'go_p1', 'timeout', '超过 8.0 秒')]:
    sum_clock[0] = t_
    sum_log.event(step_, event_, detail_)
sum_lines = sum_log.summary()
sum_text = '\n'.join(sum_lines)
check('用时表：每步用时、被挡次数、精定位误差、超时，行驶 / 精定位分开合计，存成 summary.txt',
      '3.90' in sum_text and '被安全检查挡住 1 次' in sum_text and '误差 front=-1.2cm' in sum_text
      and '行驶 11.9' in sum_text and '精定位 1.7' in sum_text and '超时：go_p1' in sum_text
      and os.path.exists(os.path.join(sum_log.folder, 'summary.txt')), sum_text)
shutil.rmtree(sum_dir)

# ---------------- 2026-10-21 场地地面分组（system.surface / surfaces）----------------
surf_cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
check('场地地面：默认 mats，底盘转向增益 0.36 / 原地 0.25、精定位最小转向 0.10，配置检查通过',
      surf_cfg['system']['surface'] == 'mats' and surf_cfg['system']['chassis']['gain_yaw'] == 0.36
      and surf_cfg['system']['chassis']['gain_yaw_inplace'] == 0.25 and surf_cfg['system']['wall_align']['min_yaw_rate'] == 0.10
      and not check_config(surf_cfg), str(check_config(surf_cfg)))
apply_overrides(surf_cfg, 'system.chassis.gain_yaw=0.5; system.surface=other')
check('场地地面：set 里换成 other 整组生效，同一个 set 里单写的 gain_yaw=0.5 不被盖掉',
      surf_cfg['system']['chassis']['gain_yaw'] == 0.5 and surf_cfg['system']['chassis']['gain_yaw_inplace'] == 0.4
      and surf_cfg['system']['wall_align']['min_yaw_rate'] == 0.04)
try:
    apply_overrides(surf_cfg, 'system.surface=ice')
    bad_surface = False
except ValueError:
    bad_surface = True
check('场地地面：换成不存在的地面名直接报错', bad_surface)
del surf_cfg['system']['surfaces']['other']['chassis']['gain_lateral']
check('场地地面：某种地面少写一项参数 → 配置检查报错',
      any('surfaces.other' in e and 'gain_lateral' in e for e in check_config(surf_cfg)), str(check_config(surf_cfg)))

off_cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
off_cfg['field']['waypoints']['end'] = [0.03, -3.25, -1.0]
off_cfg['field']['paths']['p3_to_end'] = [{'to': 'end', 'offset': [0.08, 0.0], 'profile': 'burst'}]
target = path_waypoints(off_cfg, 'p3_to_end')[0]
check('路径项 offset：终点往 +x 挪 8 cm，朝向不变', abs(target[0] - 0.11) < 1e-9 and abs(target[1] + 3.25) < 1e-9
      and abs(math.degrees(target[2]) + 1.0) < 1e-9, str(target))
off_cfg['field']['paths']['p3_to_end'] = [{'to': 'end', 'offset': [0.8, 0.0]}]
check('路径项 offset 超过 0.5 m 会被配置检查拦下', any('offset' in e for e in check_config(off_cfg)))

bad_wp = load_config(os.path.join(PACKAGE_DIR, 'config'))
bad_wp['field']['waypoints'].update({'start': [-0.056, 0.089, 0.1], 'p1': [0.007, -0.081, 0.4], 'p2': [-0.03, -0.138, 0.8],
                                     'p3': [-0.007, -0.148, -1.3], 'recog': [-0.081, -0.117, 88.6]})
errs = check_config(bad_wp)
check('15:46 那种"示教点全在起点旁边"的坐标会被拦下（不再原地转圈跑完一整局）',
      sum('离起点只有' in e for e in errs) == 4 and any('只隔' in e for e in errs), '; '.join(errs[:2]))

# 贴线段：车在通道线前方 5 cm 横移，纠偏优先、沿线速度让出轮速
seg_cfg = [{'max_speed': 0.5, 'decel': 0.8, 'end_speed': 0.0, 'wheel_limit': 0.5, 'track': True, 'profile': 'burst'},
           {'max_speed': 0.5, 'decel': 0.8, 'end_speed': 0.0, 'wheel_limit': 0.5, 'track': True, 'profile': 'burst'},
           {'max_speed': 0.35, 'decel': 0.5, 'end_speed': None, 'wheel_limit': 0.45, 'profile': 'normal'}]
track = PathFollower([(0.10, -0.37, 0.0), (0.10, -1.60, 0.0), (0.95, -1.60, 0.0)], max_speed=0.35, corner_speed=0.15,
                     decel=0.5, max_yaw_rate=1.0, yaw_gain=5.0, goal_tolerance=0.04, yaw_tolerance=math.radians(4),
                     wheel_limit=0.45, segments=seg_cfg, cross_gain=4.0, cross_max=0.15)
track.update((0.10, -0.37, 0.0))            # 起点就在第一个通道点上
track.segment = 1
cmd, _, _ = track.update((0.15, -0.70, 0.0))
check('贴线段：横移时车在通道线前方 5 cm → 往后纠偏（输出低通后约 -0.075），横移让出轮速，单轮不超 0.5',
      cmd.vx < -0.05 and cmd.vy < -0.3 and abs(cmd.vx) + abs(cmd.vy) <= 0.5 + 1e-6,
      'vx=%.3f vy=%.3f' % (cmd.vx, cmd.vy))

# ---------------- 2026-10-22 环形靶找同心圆（ring_finder.py）----------------
from shoot_race.ring_finder import RingFinder, RingSource, effective_ring_params  # noqa: E402
try:
    import cv2  # noqa: E402
    RING_IMAGE = cv2.imread(os.path.join(HERE, 'data', 'ring_1022.jpg'))
except ImportError:
    RING_IMAGE = None
check('测试图片 tests/data/ring_1022.jpg 能读', RING_IMAGE is not None and RING_IMAGE.shape == (480, 640, 3))
if RING_IMAGE is not None:
    ring_cfg = CFG['shooting']['ring']
    finder = RingFinder((ring_cfg.get('rings') or {}).get('finder', {}))
    found = finder.find(RING_IMAGE)
    check('找同心圆：10-22 16:27 车上画面靶心 (354.25, 209.25)（离线分析值），误差 < 0.3 像素',
          found is not None and abs(found['u'] - 354.25) < 0.3 and abs(found['v'] - 209.25) < 0.3, str(found))
    shifted = np.roll(np.roll(RING_IMAGE, 25, axis=1), -12, axis=0)
    found_s = finder.find(shifted)
    check('画面平移 (+25, -12) 像素，找到的靶心跟着平移（误差 < 0.3）',
          found_s is not None and abs(found_s['u'] - 379.25) < 0.3 and abs(found_s['v'] - 197.25) < 0.3, str(found_s))
    found_h = finder.find(RING_IMAGE, hint=(250.0, 150.0))
    check('上一帧位置不对（hint 离靶 100 像素）时退回全框搜索，照样找到',
          found_h is not None and abs(found_h['u'] - 354.25) < 0.3, str(found_h))
    covered = RING_IMAGE.copy()
    covered[150:270, 300:420] = np.median(RING_IMAGE[100:150, 250:300].reshape(-1, 3), axis=0)
    check('靶被挡住时返回 None（墙、窗框、立柱的得分都低于 min_score）', finder.find(covered) is None)

    class FrameIO(object):
        """只有相机画面的假 RosIO：每次 camera_frames 返回 since 之后的帧。"""
        def __init__(self):
            self.t = 10.0
            self.frames = []

        def now(self):
            return self.t

        def camera_frames(self, since):
            return [(t, img) for (t, img) in self.frames if t >= since]

    fio = FrameIO()
    src_cfg = dict(ring_cfg, detector='rings')
    source = RingSource(src_cfg)
    fio.frames = [(9.9, RING_IMAGE), (10.0, RING_IMAGE)]
    obs1 = source.observations(fio, 1.5)
    fio.t = 10.05
    fio.frames.append((10.05, shifted))           # 离上一帧只有 0.05 秒（< 1/max_rate_hz）：跳过不处理
    obs2 = source.observations(fio, 1.5)
    fio.t = 10.4
    fio.frames.append((10.4, shifted))
    obs3 = source.observations(fio, 1.5, process=False)    # 车在平移：不处理新画面
    obs4 = source.observations(fio, 1.5)
    check('RingSource：只处理最新一帧、按 max_rate_hz 跳帧、process=False 时不处理，结果带拍摄时间戳',
          len(obs1) == 1 and obs1[0][0] == 10.0 and abs(obs1[0][1] - 354.25) < 0.3 and len(obs2) == 1
          and len(obs3) == 1 and len(obs4) == 2 and obs4[1][0] == 10.4 and abs(obs4[1][1] - 379.25) < 0.3,
          '%s %s %s %s' % (obs1, obs2, obs3, obs4))

eff = effective_ring_params(dict(CFG['shooting']['ring'], detector='rings'))
old = effective_ring_params(dict(CFG['shooting']['ring'], detector='find_object'))
check('detector=rings 时 ring.rings 覆盖同名项（弹着点、容差），find_object 时用上层的值',
      eff['aim_x_px'] == CFG['shooting']['ring']['rings']['aim_x_px'] and eff['tolerance_px'] == 2.0
      and old['aim_x_px'] == CFG['shooting']['ring']['aim_x_px'] and 'finder' not in eff,
      'rings %.1f / find_object %.1f' % (eff['aim_x_px'], old['aim_x_px']))
bad_det = load_config(os.path.join(PACKAGE_DIR, 'config'))
bad_det['shooting']['ring']['detector'] = 'hough'
check('ring.detector 写错会被配置检查拦下', any('ring.detector' in e for e in check_config(bad_det)))
no_aim = load_config(os.path.join(PACKAGE_DIR, 'config'))
no_aim['shooting']['ring']['detector'] = 'rings'
del no_aim['shooting']['ring']['rings']['aim_x_px']
check('detector=rings 但没写 ring.rings.aim_x_px 会被拦下（两种像素不通用）',
      any('ring.rings.aim_x_px' in e for e in check_config(no_aim)))

print('\n%d/%d 通过' % (sum(RESULTS), len(RESULTS)))
sys.exit(0 if all(RESULTS) else 1)
