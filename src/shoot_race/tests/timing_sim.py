# -*- coding: utf-8 -*-
"""行驶 + 精定位用时仿真（只看"路上"花的时间，不看打靶）。

和 test_mission_sim.py 的区别：场地和底盘尽量按 2026-10-18 22:19 整局录包（full_221916.bag）实测来设：
  场地：后围栏 x=-0.21，左围栏 y=0.29，右围栏 y=-3.60；挡板 y=-0.95、y=-2.19，端头 x=0.42（激光叠图量的，误差约 ±5 cm）。
        各任务框位置按示教的墙距反推。
  底盘：死区 0.40 秒、惯性 0.3 秒；0.35 m/s 时后退实到 0.93、横移 0.92（录包实测）；
        边走边转 0.36、原地转 0.25（垫子上）；
        低速不动：横移指令 ≤0.025 m/s、前后 ≤0.015 m/s 车不动（录包：横移 0.022 m/s 发了 2.3 秒没动）；
        原地小角速度只能转一点点：<0.10 rad/s 时只转出指令的 0.12 倍（录包：0.04 rad/s 实际约 0.005）。
  定位：路径点和"精定位完的真实位置"对不上（AMCL 偏差），偏差按 22:19 那局各框"示教路径点 - 精定位完成位姿"设置；
        adopt=1 时模拟 teach.py adopt 之后（路径点 = 真实位置 + 2 cm / 1 度的随机误差）。
打靶步骤换成 0.5 秒的 pause。

用法：python3 tests/timing_sim.py [包目录] [seed=0] [adopt=0|1] [real=1|0] [覆盖参数 ...]
      覆盖参数同 race.launch overrides，例如 system.path_follower.max_speed=0.42
输出：每步用时、路上合计、和挡板/围栏的最小间隙（车身按 前 0.23 / 后 0.18 / 半宽 0.16）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import codecs
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
args = [a for a in sys.argv[1:]]
PACKAGE_DIR = os.path.abspath(args.pop(0)) if args and os.path.isdir(args[0]) else os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

import sim_world as sim  # noqa: E402
from shoot_race.config import apply_overrides, check_config, load_config  # noqa: E402
from shoot_race.geometry import fit_wall  # noqa: E402
from shoot_race.mission import Context, MissionRunner  # noqa: E402
from shoot_race.runlog import RunLog  # noqa: E402

opts = dict(a.split('=', 1) for a in args if '=' in a and not a.startswith(('system.', 'field.', 'mission.', 'shooting.')))
overrides = '; '.join(a for a in args if a.startswith(('system.', 'field.', 'mission.', 'shooting.')))
SEED = int(opts.get('seed', 0))
ADOPT = opts.get('adopt', '0') == '1'
REAL = opts.get('real', '1') == '1'

# ---------------- 场地 ----------------
# field=cad（默认，2026-10-20 用户给的场地 CAD 图）：360 × 360 cm；挡板离左围栏 120 / 240 cm，从后围栏 60 cm 处到执行线（180 cm）；
#   任务框 32 宽 × 38 深，中心离后围栏 121 cm、离左围栏 60 / 180 / 300 cm。地图坐标里后围栏取 x=-0.20、左围栏 y=0.29（和录包吻合）。
# field=bag：之前按 10-18 录包激光叠图估的（挡板 y=-0.95 / -2.19、端头 x=0.42、右围栏 y=-3.60）
FIELD = opts.get('field', 'today')
PART2_END_X = None
if FIELD == 'today':
    # field=today（默认，2026-10-20 16:19 菜单 10 录包 full_161955.bag，AMCL 坐标）：
    #   后围栏 x=-0.20（通道里横移时激光量车尾间隙反推，-0.17~-0.22）；挡板端头 1 号 x=0.445、2 号 x=0.40（车头间隙反推）；
    #   挡板 y 按 1/2/3 号框示教墙距反推：-0.957、-2.22；右围栏 -3.52；左围栏 0.30（识别区前墙距反推）
    BACK_X, LEFT_Y, RIGHT_Y, FRONT_X = -0.20, 0.30, -3.52, 3.40
    PART1_Y, PART2_Y, PART_END_X, PART2_END_X = -0.957, -2.22, 0.445, 0.40
elif FIELD == 'cad':
    BACK_X, LEFT_Y = -0.20, 0.29
    RIGHT_Y, FRONT_X = LEFT_Y - 3.60, BACK_X + 3.60
    PART1_Y, PART2_Y, PART_END_X = LEFT_Y - 1.20, LEFT_Y - 2.40, BACK_X + 0.60
else:
    BACK_X, LEFT_Y, RIGHT_Y, FRONT_X = -0.21, 0.29, -3.60, 3.30
    PART1_Y, PART2_Y, PART_END_X = -0.95, -2.19, 0.42
PART2_END_X = PART_END_X if PART2_END_X is None else PART2_END_X
sim.FENCE = (BACK_X, FRONT_X, RIGHT_Y, LEFT_Y)
sim.PARTITIONS = [((PART_END_X, PART1_Y), (1.75, PART1_Y)), ((PART2_END_X, PART2_Y), (1.75, PART2_Y))]
LX = sim.LASER_OFFSET_X
# 任务框真实位置：按 config_snapshot 里示教的墙距反推（车朝 +x；识别区朝 +y）
TRUE_BOX = {
    'p1': (BACK_X - LX + 1.221, PART1_Y + 0.657, 0.0),
    'p2': (BACK_X - LX + 1.238, PART2_Y + 0.690, 0.0),
    'p3': (BACK_X - LX + 1.236, PART2_Y - 0.601, 0.0),
    'end': (BACK_X - LX + 0.269 + 0.03, RIGHT_Y + 0.266, 0.0),
    'recog': (BACK_X + 0.533, LEFT_Y - LX - 0.329, math.pi / 2),
}
if FIELD == 'today':
    # 16:19 那局各框精定位完成的位姿（AMCL 坐标，events.csv 的 pose 事件）= 真实位置
    TRUE_BOX = {
        'p1': (0.994, -0.371, math.radians(4.1)),
        'p2': (0.947, -1.603, math.radians(2.1)),
        'p3': (0.986, -2.854, math.radians(0.2)),
        'end': (0.028, -3.255, math.radians(-1.2)),
        'recog': (0.333, -0.132, math.radians(91.9)),
    }
if opts.get('cadbox') == '1':
    # 车停在 CAD 任务框正中（左右离两侧墙各 0.60 m、中心离后围栏 1.21 m）；终点框中心离后、右围栏各 0.20 m
    for name, n in (('p1', 0), ('p2', 1), ('p3', 2)):
        TRUE_BOX[name] = (BACK_X + 1.21, LEFT_Y - 0.60 - 1.20 * n, 0.0)
    TRUE_BOX['end'] = (BACK_X + 0.20 + 0.03, RIGHT_Y + 0.20, 0.0)
sim.START_POSE = (BACK_X + 0.20, LEFT_Y - 0.20, 0.0)          # 起点框正中
if FIELD == 'today':
    # 16:19 那局开局位姿 (-0.054, 0.086)；起点附近 AMCL 的 x 比通道里偏约 4 cm（这里后围栏只有一条线），x 取 -0.015 免得一开始就算碰墙
    sim.START_POSE = (-0.015, 0.086, math.radians(0.2))
sim.BOX_CENTERS = dict((k, v[:2]) for k, v in TRUE_BOX.items() if k != 'recog')
sim.RECOG_POSE = TRUE_BOX['recog']
# 22:19 那局"示教路径点 - 精定位完成位姿"（AMCL 坐标，米/度）
WAYPOINT_BIAS = {'recog': (0.126, -0.029, -3.6), 'p1': (0.069, 0.037, -6.2), 'p2': (-0.027, 0.0, -0.8),
                 'p3': (-0.038, 0.092, -2.7), 'end': (-0.066, 0.221, -6.3)}

if REAL:
    sim.SIM_DEAD_TIME = 0.40
    sim.SIM_GAIN_FORWARD = 0.85
    sim.SIM_GAIN_BACKWARD = 0.93
    sim.SIM_GAIN_LATERAL = 0.92
    sim.SIM_GAIN_YAW = 0.38            # 16:19 录包：边走边转 1.0 rad/s 时 IMU 实测 0.36~0.40
    sim.SIM_GAIN_YAW_INPLACE = 0.25
if 'dead' in opts:
    sim.SIM_DEAD_TIME = float(opts['dead'])          # 死区敏感性检查（录包里 0.35~0.5 秒）

FRONT, REAR, HALF = 0.23, 0.18, 0.16
WHEEL_SAT = float(opts.get('wheelsat', '0.5'))      # 单轮线速度上限（m/s），0 = 不模拟
AMCL_X_BIAS = float(opts.get('amclx', '0'))
if opts.get('selfpts') == '0':
    sim.SELF_POINTS = []                             # 不加激光扫到的车身零件（对比用）


class RealIO(sim.SimIO):
    """底盘加上低速不动 / 原地小转速只转一点（静摩擦），记录离障碍的最小间隙。"""

    def __init__(self, *a, **k):
        sim.SimIO.__init__(self, *a, **k)
        self.min_clear = {}
        self.step = ''

    def send_velocity(self, command):
        vx, vy, wz = command.vx, command.vy, command.wz
        if REAL:
            vx = 0.0 if abs(vx) <= 0.015 else vx
            vy = 0.0 if abs(vy) <= 0.025 else vy
            if abs(vx) + abs(vy) < 1e-3 and abs(wz) < 0.10:
                wz *= 0.12 / sim.SIM_GAIN_YAW_INPLACE        # 原地小角速度：实际只有指令的约 0.12 倍
        # 底盘固件上限（abot_bringup base_driver：max_v_liner_x/y = 50 cm/s，max_v_angular_z = 250 = 2.5 rad/s）
        vx = max(-0.5, min(0.5, vx))
        vy = max(-0.5, min(0.5, vy))
        wz = max(-2.5, min(2.5, wz))
        if WHEEL_SAT:
            # 单个麦轮线速度上限：|vx ± vy ± k·wz| 超过的轮子被削平，合成速度方向会歪
            # （16:19 录包：横移 0.48 m/s 时同时发的 -0.07 m/s 后退几乎没起作用）
            k = 0.225                                    # 固件转向力臂 = abot_bringup wheel_track 225 mm（URDF 轮心 0.12 + 0.108）
            wheels = [vx - vy - k * wz, vx + vy + k * wz, vx + vy - k * wz, vx - vy + k * wz]
            wheels = [max(-WHEEL_SAT, min(WHEEL_SAT, w)) for w in wheels]
            vx = sum(wheels) / 4.0
            vy = (-wheels[0] + wheels[1] + wheels[2] - wheels[3]) / 4.0
            wz = (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) / (4.0 * k)
        self.command = [vx, vy, wz]

    def _advance(self, dt):
        sim.SimIO._advance(self, dt)
        if self.step.startswith('go_'):
            c = self.clearance()
            if c < self.min_clear.get(self.step, (9.0,))[0]:
                self.min_clear[self.step] = (c, tuple(round(v, 3) for v in self.pose))

    def clearance(self):
        """车身矩形到挡板/围栏线段的最小距离（米，负数 = 碰到）。"""
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        best = 9.0
        for (ax, ay), (bx, by) in self.segments:
            n = int(max(2, math.hypot(bx - ax, by - ay) / 0.01))
            px = np.linspace(ax, bx, n) - x
            py = np.linspace(ay, by, n) - y
            fx = px * c + py * s
            fy = -px * s + py * c
            dx = np.maximum(np.maximum(fx - FRONT, -REAR - fx), 0.0)
            dy = np.maximum(np.abs(fy) - HALF, 0.0)
            inside = (fx < FRONT) & (fx > -REAR) & (np.abs(fy) < HALF)
            d = np.where(inside, -0.01, np.hypot(dx, dy))
            best = min(best, float(d.min()))
        return best

    def robot_pose(self):
        x, y, yaw = self.pose
        # amclx：通道附近 AMCL 的 x 整体偏差（米）。16:19 那局 2 号挡板端头在地图里比 10-18 那局近了约 3 cm
        bias = AMCL_X_BIAS if x < 0.6 else 0.0
        return (x + bias + self.rng.normal(0, 0.01), y + self.rng.normal(0, 0.01), yaw + self.rng.normal(0, 0.002))


def build_field(cfg, io, rng):
    deg = math.degrees
    wp = cfg['field']['waypoints']
    for name, (x, y, yaw) in TRUE_BOX.items():
        if ADOPT:
            bx, by, byaw = rng.normal(0, 0.02), rng.normal(0, 0.02), rng.normal(0, 1.0)
        else:
            bx, by, byaw = WAYPOINT_BIAS[name]
        wp[name] = [x + bx, y + by, deg(yaw) + byaw]
    wp['start'] = [sim.START_POSE[0], sim.START_POSE[1], deg(sim.START_POSE[2])]
    if opts.get('wp') == 'today':
        # 16:12 写进车里的路径点（按 10-18 精定位位姿 + 新旧墙距差推算），16:19 那局就是用它跑的
        wp.update({'recog': [0.297, -0.145, 91.8], 'p1': [0.963, -0.365, 6.9], 'p2': [0.96, -1.565, 2.8],
                   'p3': [1.028, -2.836, 3.8], 'end': [0.016, -3.112, -3.2]})
    lane_x = float(opts.get('lane_x', 0.10))
    for lane, box in (('lane_p1', 'p1'), ('lane_p2', 'p2'), ('lane_p2b', 'p2'), ('lane_p3', 'p3')):
        if lane in wp:
            wp[lane] = [lane_x, wp[box][1], wp[lane][2] if opts.get('lane_yaw') == 'cfg' else 0.0]
    if 'end_near' in wp:
        near = float(opts.get('end_near', 0.12))
        wp['end_near'] = [wp['end'][0] + near, wp['end'][1], wp['end'][2]]
    sides = {'p1': ['right', 'back'], 'p2': ['right', 'back'], 'p3': ['left', 'back'], 'end': ['right', 'back'],
             'recog': ['front', 'left']}
    for box, box_sides in sides.items():
        io.pose = list(TRUE_BOX[box])
        angles, xs, ys = io.scan_points()
        fits = dict((side, fit_wall(angles, xs, ys, side)) for side in box_sides)
        yaw_side = 'front' if box == 'recog' else box_sides[0] if box != 'end' else 'back'
        old = cfg['field']['boxes'].get(box, {})
        cfg['field']['boxes'][box] = dict(old, walls=[{'side': s, 'distance': fits[s].distance} for s in box_sides],
                                          yaw_side=yaw_side, yaw_ref_deg=deg(fits[yaw_side].yaw_offset))
    if opts.get('end_route') == 'L':
        # 用户 10-20 画的路线：3 号点先往右横移到终点那一列，再直线后退进终点框
        wp['pre_end'] = [wp['p3'][0], wp['end'][1], 0.0]
        cfg['field']['paths']['p3_to_end'] = ['pre_end', 'end']
    elif opts.get('end_route') == 'back':
        # 先直退到终点前方，再横移进角落
        wp['pre_end'] = [wp['end'][0], wp['p3'][1], 0.0]
        cfg['field']['paths']['p3_to_end'] = ['pre_end', 'end']
    io.pose = list(sim.START_POSE)


def main():
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    io = RealIO(np.zeros((480, 640, 3), np.uint8), seed=SEED)
    rng = np.random.RandomState(SEED + 100)
    build_field(cfg, io, rng)
    steps = []
    for spec in cfg['mission']['steps']:
        if spec['type'] in ('shoot_ring', 'shoot_ar') and opts.get('rehearsal') != '1':
            spec = {'id': spec['id'], 'name': spec['name'], 'type': 'pause', 'seconds': 0.5, 'timeout_s': 2}
        if spec['type'] == 'recognize':
            spec = {'id': spec['id'], 'name': spec['name'], 'type': 'pause', 'seconds': 0.5, 'timeout_s': 2}
        steps.append(spec)
    cfg['mission']['steps'] = steps
    apply_overrides(cfg, overrides)
    errors = check_config(cfg)
    assert not errors, errors
    log = RunLog(tempfile.mkdtemp(), printer=lambda text: None, clock=io.now)

    class Quiet(object):
        def __getattr__(self, name):
            return lambda *a, **k: True
    shooter = sim.SimShooter(io)
    voice = sim.SimVoice(io)
    ctx = Context(cfg, io, shooter, voice, Quiet(), log)
    # 记录当前步骤（算每个行驶步骤的最小间隙）
    orig_event = log.event

    def event(step, kind, detail=''):
        if kind == 'start':
            io.step = step
        return orig_event(step, kind, detail)
    log.event = event
    total = MissionRunner(ctx).run()
    events = codecs.open(os.path.join(log.folder, 'events.csv'), encoding='utf-8').read().splitlines()[1:]
    dur = {}
    kinds = {}
    for e in events:
        parts = e.split(',', 3)
        if parts[2] in ('done', 'timeout'):
            dur[parts[1]] = float(parts[3].split()[1])
            kinds[parts[1]] = parts[2]
    road = sum(v for k, v in dur.items() if k.startswith(('go_', 'align_')))
    print('seed=%d adopt=%d total=%.1f road=%.1f collisions=%d' % (SEED, ADOPT, total, road, io.collisions))
    for k in [s['id'] for s in steps if s['id'].startswith(('go_', 'align_'))]:
        clear = io.min_clear.get(k)
        print('  %-12s %5.2f %s %s' % (k, dur.get(k, float('nan')), 'TIMEOUT' if kinds.get(k) == 'timeout' else '',
                                       '' if clear is None else 'min_clear=%.3f at %s' % clear))
    for e in events:
        if ',blocked,' in e or ',near,' in e or ',partial,' in e or ',near_goal,' in e or 'boost' in e:
            print('   ', e)
    return road


if __name__ == '__main__':
    main()
