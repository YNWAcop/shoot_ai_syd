# -*- coding: utf-8 -*-
"""整局仿真测试：用 sim_world 的假设场地跑完整个 mission.yaml。

检查的是"逻辑"：每一步是否完成、有没有超时、有没有撞墙、播报时车是否静止、
识别结果是否正确、开火次数、总用时、预测开火时机按假设延迟算是否命中目标点附近。
（假设值见 sim_world.py 顶部说明，结果不代表真实命中率。）

运行：python3 tests/test_mission_sim.py
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import codecs
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PACKAGE_DIR, 'src'))
sys.path.insert(0, HERE)

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import sim_world as sim  # noqa: E402
from shoot_race.config import apply_overrides, check_config, load_config  # noqa: E402
from shoot_race.geometry import fit_wall  # noqa: E402
from shoot_race.mission import Context, MissionRunner  # noqa: E402
from shoot_race.recognition import CardMatcher, CardRecognizer  # noqa: E402
from shoot_race.runlog import RunLog  # noqa: E402


class FakeVlm(object):
    """假的豆包：直接返回预设答案（或模拟失败）。"""

    def __init__(self, reply=None, fail=False):
        self.reply = reply
        self.fail = fail

    def is_configured(self):
        return True

    def ask(self, image):
        if self.fail:
            raise IOError('network unreachable')
        return self.reply


def synth_card_photo(card_path, seed=3):
    """把卡片模板透视变换后贴进 640x480 的画面，模拟相机拍照。"""
    rng = np.random.RandomState(seed)
    card = cv2.imread(card_path)
    h, w = card.shape[:2]
    frame = np.full((480, 640, 3), 150, np.uint8)
    out_h = 480 * 0.7
    out_w = out_h * w / h
    cx, cy = 320, 240
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    jitter = lambda: rng.uniform(-0.06, 0.06) * out_w  # noqa: E731
    dst = np.float32([[cx - out_w / 2 + jitter(), cy - out_h / 2 + jitter()],
                      [cx + out_w / 2 + jitter(), cy - out_h / 2 + jitter()],
                      [cx + out_w / 2 + jitter(), cy + out_h / 2 + jitter()],
                      [cx - out_w / 2 + jitter(), cy + out_h / 2 + jitter()]])
    matrix = cv2.getPerspectiveTransform(src, dst)
    photo = cv2.warpPerspective(card, matrix, (640, 480), dst=frame, borderMode=cv2.BORDER_TRANSPARENT)
    return cv2.GaussianBlur(photo, (3, 3), 0.8)


def rotating_box_yaw(params, io):
    """fire_side 为 left/right 时，车在 2 号框中心转多少度（弧度，左转为正），那一侧交点正好落在弹着点上。

    车转 ψ，画面里转轮圆心横移约 fx·tanψ；在 ±8 度里二分。fire_side = any 时不转（返回 0）。
    """
    side = params.get('fire_side', 'any')
    if side not in ('left', 'right'):
        return 0.0
    center_world = (sim.WHEEL_CENTER_DEPTH, sim.BOX_CENTERS['p2'][1] - sim.WHEEL_CENTER_RIGHT, sim.WHEEL_CENTER_Y_CAM)
    saved = io.pose

    def crossing_error(yaw):
        io.pose = [sim.BOX_CENTERS['p2'][0], sim.BOX_CENTERS['p2'][1], yaw]
        cam = io.marker_camera_xy(center_world)
        uc, vc = io.camera_to_pixel(cam)
        radius = sim.AR_FX * sim.WHEEL_RADIUS / cam[2]            # cam[2] = 相机到转轮的前向距离
        du = math.sqrt(max(radius ** 2 - (params['aim_v_px'] - vc) ** 2, 0.0))
        return uc + (du if side == 'right' else -du) - params['aim_u_px']

    low, high = math.radians(-8.0), math.radians(8.0)
    for _ in range(40):
        mid = 0.5 * (low + high)
        # 车左转（yaw 增大）→ 转轮在画面里右移 → 交点 u 增大
        if crossing_error(mid) > 0:
            high = mid
        else:
            low = mid
    io.pose = saved
    return 0.5 * (low + high)


def build_sim_field(cfg, io):
    """把 field 配置换成仿真场地的数值；任务框的墙距用仿真激光"示教"得到（模拟 teach.py box）。"""
    deg = math.degrees
    wp = {
        'start': list(sim.START_POSE[:2]) + [0.0],
        'recog': [sim.RECOG_POSE[0], sim.RECOG_POSE[1], deg(sim.RECOG_POSE[2])],
        'p1': list(sim.BOX_CENTERS['p1']) + [0.0],
        'lane_p1': [0.25, sim.BOX_CENTERS['p1'][1], 0.0],
        'lane_p2': [0.25, sim.BOX_CENTERS['p2'][1], 0.0],
        'p2': list(sim.BOX_CENTERS['p2']) + [0.0],
        'lane_p2b': [0.25, sim.BOX_CENTERS['p2'][1], 0.0],
        'lane_p3': [0.25, sim.BOX_CENTERS['p3'][1], 0.0],
        'p3': list(sim.BOX_CENTERS['p3']) + [0.0],
        'end': list(sim.BOX_CENTERS['end']) + [0.0],
        'pre_end': [sim.BOX_CENTERS['end'][0], sim.BOX_CENTERS['p3'][1], 0.0],   # 先直退，再侧移进角落的终点框
    }
    cfg['field']['waypoints'] = wp
    cfg['field']['paths']['p3_to_end'] = ['pre_end', 'end']
    sides = {'p1': ['left', 'back'], 'p2': ['right', 'back'], 'p3': ['right', 'back'], 'end': ['right', 'back']}
    # 旋转靶只打一侧时（fire_side），2 号框按 teach.py turn 的做法示教一个朝向偏转，让那一侧的圆周经过弹着点
    yaw_at = {'p2': rotating_box_yaw(cfg['shooting']['rotating'], io)}
    wp['p2'][2] = deg(yaw_at['p2'])
    for box, box_sides in sides.items():
        io.pose = [sim.BOX_CENTERS[box][0], sim.BOX_CENTERS[box][1], yaw_at.get(box, 0.0)]
        angles, xs, ys = io.scan_points()
        walls = []
        fits = {}
        for side in box_sides:
            fits[side] = fit_wall(angles, xs, ys, side)
            walls.append({'side': side, 'distance': fits[side].distance})
        # 偏转过的框按实测偏角示教（同 teach.py box），其余框朝向基准 0（和以前一样）
        yaw_ref = deg(fits['back'].yaw_offset) if yaw_at.get(box, 0.0) else 0.0
        cfg['field']['boxes'][box] = {'walls': walls, 'yaw_side': 'back', 'yaw_ref_deg': yaw_ref}
    # 识别区：车头朝围栏上的卡片，贴前方围栏定距离和朝向、左侧围栏定横向（同 teach.py box recog front left yaw=front）
    io.pose = list(sim.RECOG_POSE)
    angles, xs, ys = io.scan_points()
    fits = dict((side, fit_wall(angles, xs, ys, side)) for side in ('front', 'left'))
    cfg['field']['boxes']['recog'] = {'walls': [{'side': side, 'distance': fits[side].distance} for side in ('front', 'left')],
                                      'yaw_side': 'front', 'yaw_ref_deg': deg(fits['front'].yaw_offset)}
    io.pose = list(sim.START_POSE)


def run_case(name, policy, vlm, card_index=7, expect_source=None, hidden=False, overrides=''):
    print('\n==================== %s ====================' % name)
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    cards_dir = os.path.join(PACKAGE_DIR, 'config', 'cards')
    card = cfg['recognition']['cards'][card_index]
    photo = synth_card_photo(os.path.join(cards_dir, card['file']))
    if hidden:
        # 异常情况：环形靶和 2 号旋转靶的码一直识别不到
        io = sim.SimIO(photo, hide_ring=True, hide_markers=[1, 2, 3, 4, 5])
    else:
        io = sim.SimIO(photo)
    build_sim_field(cfg, io)
    cfg['recognition']['policy'] = policy
    apply_overrides(cfg, overrides)
    errors = check_config(cfg)
    assert not errors, errors

    log = RunLog(tempfile.mkdtemp(), printer=lambda text: None, clock=io.now)
    # 仿真时间比真实时间跑得快得多，识别的后台线程（真实耗时约 0.3 秒）会"来不及"；
    # 测试里改成同步执行，相当于假设识别在一个控制周期内完成。
    CardRecognizer._run_in_thread = staticmethod(lambda job, image: job(image))
    matcher = CardMatcher(cfg['recognition']['cards'], cards_dir, cfg['recognition']['offline'])
    recognizer = CardRecognizer(cfg['recognition'], cards_dir, matcher=matcher, vlm=vlm)
    shooter = sim.SimShooter(io)
    voice = sim.SimVoice(io)
    ctx = Context(cfg, io, shooter, voice, recognizer, log)
    total = MissionRunner(ctx).run()

    # ---------- 读日志，逐项检查 ----------
    # 用 codecs.open（Python2/3 都支持 encoding 参数）；这里变量 io 是仿真对象，不能用 io.open
    events = codecs.open(os.path.join(log.folder, 'events.csv'), encoding='utf-8').read().splitlines()[1:]
    timeouts = [e for e in events if ',timeout,' in e]
    skipped = [e for e in events if ',skip,' in e]
    shots = codecs.open(os.path.join(log.folder, 'shots.csv'), encoding='utf-8').read().splitlines()[1:]
    expect_rot = card['answer']
    expect_move = cfg['recognition']['fruit_to_moving_id'][card['fruit']]
    final_x, final_y, final_yaw = io.pose
    end_x, end_y = sim.BOX_CENTERS['end']

    results = []

    def check(label, ok, detail=''):
        results.append(ok)
        print('%s  %s  %s' % ('PASS' if ok else 'FAIL', label, detail))

    shooting = cfg['shooting']
    total_shots = sum(int(shooting[t]['fire']['shots']) for t in ('ring', 'rotating', 'moving'))
    check('总用时 < 120 秒', total < 120, '%.1f 秒' % total)
    if hidden:
        # 环形靶：超时兜底开火；旋转靶：看不清转轮 blind_timeout_s 秒后不干等，照样开一枪
        ok = (any(',shoot_ring,timeout,' in e for e in events) and any(',shoot_rot,blind,' in e for e in events)
              and len(shooter.fire_times) == total_shots and any(',say_end,done,' in e for e in events))
        check('看不到靶的两步照样开火、流程继续', ok,
              '; '.join(e for e in events if ',timeout,' in e or ',blind,' in e) + '；开火 %d 次' % len(shooter.fire_times))
        check('没有撞墙', io.collisions == 0)
        return all(results)
    check('没有步骤超时', not timeouts, '; '.join(timeouts))
    check('没有步骤被跳过', not skipped, '; '.join(skipped))
    check('没有撞墙', io.collisions == 0, '碰撞帧数 %d' % io.collisions)
    check('识别靶号正确', ctx.targets == {'rotating': expect_rot, 'moving': expect_move},
          '%s（应为 rotating=%d moving=%d）' % (ctx.targets, expect_rot, expect_move))
    if expect_source:
        source = [e for e in events if ',targets,' in e][0].split('source=')[1]
        check('识别结果来源 = %s' % expect_source, source == expect_source, source)
    check('播报 3 次且播报时车静止', len(voice.played) == 3 and all(s < 0.01 for _, _, s in voice.played),
          str([(round(t - 100, 1), f, round(s, 3)) for t, f, s in voice.played]))
    check('开火次数 = 三个靶 shots 之和（%d）' % total_shots, len(shooter.fire_times) == total_shots,
          '%d 次' % len(shooter.fire_times))
    check('触发方式都不是超时兜底', all(',timeout,' not in s for s in shots), '; '.join(s.split(',')[3] for s in shots))
    check('停在终点框内（误差 < 1.5 cm）', math.hypot(final_x - end_x, final_y - end_y) < 0.015,
          '误差 %.1f cm' % (100 * math.hypot(final_x - end_x, final_y - end_y)))
    check('最高速度', True, '%.2f m/s' % io.max_speed_seen)

    # ---------- 环形靶：开火时刻弹着点离靶心多远（按仿真的真实位姿算）----------
    check('仿真的环形靶弹着点与 shooting.yaml 一致（find_object：ring.aim_x_px；找同心圆：ring.rings.aim_x_px）',
          abs(sim.RING_AIM_PX - shooting['ring']['aim_x_px']) < 1e-6
          and abs(sim.RINGS_AIM_PX - (shooting['ring'].get('rings') or {}).get('aim_x_px', sim.RINGS_AIM_PX)) < 1e-6,
          'sim %.1f / %.1f，yaml %.1f / %s' % (sim.RING_AIM_PX, sim.RINGS_AIM_PX, shooting['ring']['aim_x_px'],
                                              (shooting['ring'].get('rings') or {}).get('aim_x_px')))
    ring_rows = [s for s in shots if ',ring,' in s]
    if cfg['shooting']['ring'].get('mode', 'vision') != 'vision':
        # 按示教位置直接开火：仿真里环形靶故意放在框中心偏 4 cm 的地方，所以不查弹着点，只查开火快不快
        ring_done = [e for e in events if ',shoot_ring,done,' in e]
        check('环形靶 %s：精定位后 1 秒内开火' % cfg['shooting']['ring']['mode'],
              ring_rows and ring_done and float(ring_done[0].split('用时 ')[1].split(' ')[0]) < 1.0,
              '; '.join(ring_done + [r.split(',', 4)[-1][:60] for r in ring_rows]))
    elif ring_rows:
        fire_time = float(ring_rows[0].split(',')[0]) + log.match_start
        pose_at_fire = min(io.log_pose, key=lambda p: abs(p[0] - fire_time))
        miss = io.ring_miss_cm(pose_at_fire[1:])
        if cfg['shooting']['ring'].get('detector', 'find_object') == 'rings':
            # 找同心圆：识别噪声约 0.3 像素，误差主要是对准容差 ring.rings.tolerance_px（2 像素 ≈ 0.8 cm）
            check('环形靶（找同心圆）：开火时弹着点离靶心 < 1.0 cm', abs(miss) < 1.0,
                  '%.2f cm；%s' % (miss, ring_rows[0].split(',', 4)[-1][:80]))
        else:
            check('环形靶：开火时弹着点离靶心 < 2 cm（识别噪声 ±3 像素 ≈ ±1.5 cm）', abs(miss) < 2.0,
                  '%.2f cm；%s' % (miss, ring_rows[0].split(',', 4)[-1][:80]))

    # ---------- 预测开火的自洽性：按"假设的真实延迟"算，弹丸到达时目标离弹着点多远 ----------
    for target in ('rotating', 'moving'):
        params = cfg['shooting'][target]
        marker = ctx.targets[target]
        fire_rows = [s for s in shots if ',%s,' % target in s]
        if not fire_rows:
            continue
        fire_time = float(fire_rows[0].split(',')[0]) + log.match_start
        impact = fire_time + sim.TRUE_LATENCY_S
        world = (io.rotating_marker_world(marker, impact) if target == 'rotating'
                 else io.moving_marker_world(marker, impact))
        # 开火时车静止在任务框里，用那一刻的位姿算相机坐标
        pose_at_fire = min(io.log_pose, key=lambda p: abs(p[0] - fire_time))
        saved = io.pose
        io.pose = list(pose_at_fire[1:])
        cam = io.marker_camera_xy(world)
        io.pose = saved
        u, v = io.camera_to_pixel(cam)
        err_u = u - params['aim_u_px']
        err_v = (v - params['aim_v_px']) if (target == 'rotating' or params.get('use_v')) else 0.0
        error_cm = 100.0 * math.hypot(err_u, err_v) / sim.AR_FX * cam[2]       # 像素 → 靶面上的厘米
        check('%s 预测开火：到达时刻目标偏离弹着点 < 1.5 cm' % target, error_cm < 1.5,
              'du=%.1f px dv=%.1f px（%.2f cm）' % (err_u, err_v, error_cm))
    print('用时明细：')
    for e in events:
        if ',done,' in e or ',align_error,' in e:
            print('   ', e)
    return all(results)


def run_near_wall():
    """行驶到贴墙的终点、还要转 90 度：被激光判定"前方有墙"后应原地转完再结束（2026-10-17 实车只转到 56 度）。"""
    print('\n==================== 贴墙终点：被挡住后原地转完 ====================')
    from shoot_race.steps import DONE, build_step
    cfg = load_config(os.path.join(PACKAGE_DIR, 'config'))
    cfg['field']['waypoints']['near_wall'] = [0.5, 0.10, 90]         # 离左围挡很近，车头要转向左
    cfg['field']['paths']['to_wall'] = ['near_wall']
    io = sim.SimIO(np.zeros((480, 640, 3), np.uint8))
    io.pose = [0.5, -0.05, 0.0]
    log = RunLog(tempfile.mkdtemp(), printer=lambda text: None, clock=io.now)
    ctx = Context(cfg, io, sim.SimShooter(io), sim.SimVoice(io), None, log)
    step = build_step({'id': 'go_wall', 'type': 'drive', 'path': 'to_wall', 'timeout_s': 10})
    step.start(ctx)
    started, status = io.now(), None
    while io.now() - started < 10.0:
        status = step.update(ctx)
        ctx.publish_motion()
        if status == DONE:
            break
        io.wait_next_tick()
    used = io.now() - started
    step.finish(ctx)
    for _ in range(30):                     # 再发 1.5 秒零速度，等车真正停下再量（底盘有死区 + 惯性）
        ctx.publish_motion()
        io.wait_next_tick()
    yaw_error = abs(math.degrees(io.pose[2]) - 90.0)
    ok = status == DONE and used < 8.0 and yaw_error < 4.0 and io.collisions == 0   # 8 秒 = go_recog 的时间上限；原地转很慢（实测增益约 0.4）
    print('%s  8 秒内结束、停稳后朝向差 < 4 度、不撞墙  用时 %.2f 秒，朝向差 %.1f 度，碰撞 %d' % (
        'PASS' if ok else 'FAIL', used, yaw_error, io.collisions))
    return ok


if __name__ == '__main__':
    reply = '{"answer": 3, "fruit": "杨桃"}'    # 第 8 张卡：2只小鸡又来1只 → 3；杨桃
    ok = run_case('豆包正常（vlm_first）', 'vlm_first', FakeVlm(reply=reply), expect_source='vlm')
    ok &= run_case('断网：豆包失败 → 离线结果', 'vlm_first', FakeVlm(fail=True), expect_source='offline')
    ok &= run_case('离线优先（offline_first）', 'offline_first', FakeVlm(fail=True), card_index=11,
                   expect_source='offline')
    ok &= run_case('异常：环形靶、旋转靶都识别不到', 'vlm_first', FakeVlm(reply=reply), hidden=True)
    ok &= run_case('环形靶按示教位置直接开火（ring.mode=taught）', 'vlm_first', FakeVlm(reply=reply),
                   expect_source='vlm', overrides='shooting.ring.mode=taught')
    ok &= run_near_wall()
    print('\n全部通过' if ok else '\n有失败项')
    sys.exit(0 if ok else 1)
