# -*- coding: utf-8 -*-
"""任务执行器：按 mission.yaml 的顺序执行步骤，统一管理时间和底盘指令。

主循环（每个控制周期，默认 20 Hz）做三件事：
    1. 调用当前步骤的 update()，步骤把想要的速度写进 ctx.command；
    2. publish_motion()：对 ctx.command 做加速度限制后，通过 RosIO 发出（全程序唯一出口）；
    3. 检查这一步是否完成、是否超时。

时间管理（对应规则"全程 2 分钟""20 秒状态不变判结束"）：
    - 每一步有 timeout_s（config.check_config 保证都 < 20 秒；旋转靶例外，它等待时会定期小幅平移，
      见 aiming.RotatingPlanner 的 keepalive），超时就调用 on_timeout 并进入下一步；
    - 步骤可以写 skip_if_less_than_s：开始这一步前，如果比赛剩余时间少于它，就跳过，
      把时间留给"到点""终点播报"这些确定能拿的分。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

from .motion import STOP, ChassisModel, VelocityCommand, VelocitySmoother
from .steps import DONE, build_step


class Context(object):
    """所有步骤共享的对象和数据。"""

    def __init__(self, cfg, io, shooter, voice, recognizer, log, dry_fire=False):
        self.cfg = cfg
        self.io = io                    # RosIO（或测试用的 SimIO）
        self.shooter = shooter
        self.voice = voice
        self.recognizer = recognizer
        self.log = log
        self.dry_fire = dry_fire        # 演练模式：只记录不真正发射
        default = cfg['recognition']['default_targets']
        self.targets = {'rotating': int(default['rotating']), 'moving': int(default['moving'])}

        self.command = STOP             # 当前步骤想要的速度（还没平滑、没发布）
        self.external_control = False   # True：move_base 在控制底盘，本程序不发速度
        accel = cfg['system']['accel_limits']
        self.smoother = VelocitySmoother(accel['linear'], accel['angular'])
        # 底盘响应模型（死区 + 惯性），行驶和精定位用它预测停车点，见 motion.ChassisModel
        self.chassis = ChassisModel(cfg['system'].get('chassis', {}), dt=1.0 / float(cfg['system'].get('loop_hz', 20)))
        self.still_since = None         # 从什么时刻开始车一直是停着的
        self.last_publish_time = None
        self.match_start = None         # 比赛（或单步测试）开始时刻，MissionRunner.run 设置

    def publish_motion(self):
        """把 ctx.command 平滑后发出去，并更新"车是否停稳"的判断。"""
        now = self.io.now()
        dt = 0.05 if self.last_publish_time is None else max(0.0, min(0.2, now - self.last_publish_time))
        self.last_publish_time = now
        if self.external_control:
            self.smoother.reset()
            self.chassis.reset()
            self.still_since = None
            return
        sent = self.smoother.smooth(self.command, dt)
        self.io.send_velocity(sent)
        self.chassis.record(now, sent, self.io.wheel_speed())
        self._update_still(now, sent)

    def _update_still(self, now, sent):
        """车停稳 = 我们发的是零速度，并且轮速计测到的速度也很小。"""
        limits = self.cfg['system']['still_speed']
        wheel = self.io.wheel_speed()
        wheel_still = wheel is None or (abs(wheel[0]) < limits['linear'] and abs(wheel[1]) < limits['linear']
                                        and abs(wheel[2]) < limits['angular'])
        if sent.is_zero() and wheel_still:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None

    def robot_still_for(self, seconds):
        """车是否已经连续停稳了 seconds 秒。"""
        return self.still_since is not None and self.io.now() - self.still_since >= seconds

    def still_duration(self):
        """车已经连续静止多久（秒）；正在动返回 0。"""
        return 0.0 if self.still_since is None else self.io.now() - self.still_since

    def match_end(self):
        """比赛结束时刻（开始 + match_time_s）。"""
        start = self.match_start if self.match_start is not None else self.io.now()
        return start + float(self.cfg['mission']['match_time_s'])

    def stop_translation(self):
        """立即停止平移（不经过平滑），转向照常平滑。下一次 publish_motion 才发出。"""
        current = self.smoother.current
        self.smoother.current = VelocityCommand(0.0, 0.0, current.wz)

    def stop_robot(self):
        """立即停车（不经过平滑）。"""
        self.command = STOP
        self.smoother.reset()
        self.io.send_velocity(STOP)
        self.chassis.record(self.io.now(), STOP, self.io.wheel_speed())


class MissionRunner(object):
    def __init__(self, ctx):
        self.ctx = ctx
        self.specs = ctx.cfg['mission']['steps']
        self.match_time_s = float(ctx.cfg['mission']['match_time_s'])

    def select_steps(self, from_step=None, to_step=None):
        """按 id 截取一段步骤（单步测试用）；不指定时执行全部。id 写错时抛出 ValueError。"""
        ids = [spec['id'] for spec in self.specs]
        for name in (from_step, to_step):
            if name and name not in ids:
                raise ValueError('mission.yaml 里没有步骤 %s，可选：%s' % (name, ', '.join(ids)))
        first = ids.index(from_step) if from_step else 0
        last = ids.index(to_step) if to_step else len(ids) - 1
        if last < first:
            raise ValueError('to_step 在 from_step 前面')
        return self.specs[first:last + 1]

    def run(self, from_step=None, to_step=None):
        ctx = self.ctx
        io = ctx.io
        match_start = io.now()
        ctx.match_start = match_start
        ctx.log.start_match()
        pause_s = float(ctx.cfg['mission'].get('rehearsal_pause_s', 0) or 0)
        for spec in self.select_steps(from_step, to_step):
            if io.is_shutdown():
                break
            if pause_s > 0 and spec.get('type') in ('shoot_ring', 'shoot_ar'):
                # 整局演练（菜单 10）：打靶步骤换成原地停 pause_s 秒，不射击、不看靶，只练路上
                spec = {'id': spec['id'], 'name': '%s（演练：停 %g 秒，不射击）' % (spec.get('name', spec['id']), pause_s),
                        'type': 'pause', 'seconds': pause_s, 'timeout_s': pause_s + 2.0}
            step = build_step(spec)
            remaining = self.match_time_s - (io.now() - match_start)
            if step.skip_if_less_than_s is not None and remaining < step.skip_if_less_than_s:
                ctx.log.event(step.id, 'skip', '%s：剩余 %.1f 秒 < %.1f 秒' % (step.name, remaining, step.skip_if_less_than_s))
                continue
            self._run_one(step)
        ctx.stop_robot()
        total = io.now() - match_start
        ctx.log.event('mission', 'end', '总用时 %.1f 秒' % total)
        try:
            ctx.log.summary()                  # 用时表（summary.txt）；出错也不影响比赛结果
        except Exception as error:             # noqa: BLE001
            ctx.log.info('用时表生成失败：%s' % error)
        return total

    def _run_one(self, step):
        ctx = self.ctx
        io = ctx.io
        ctx.log.event(step.id, 'start', step.name)
        limit = step.timeout_s
        if step.spec.get('type') == 'drive':
            # 场地测试限速跑时，行驶步骤的时间上限按比例放宽（race.launch set:="mission.drive_timeout_scale=2"）
            limit *= float(ctx.cfg['mission'].get('drive_timeout_scale', 1.0))
        started = io.now()
        step.start(ctx)
        while not io.is_shutdown():
            status = step.update(ctx)
            ctx.publish_motion()
            if status == DONE:
                ctx.log.event(step.id, 'done', '用时 %.2f 秒' % (io.now() - started))
                break
            if io.now() - started > limit:
                ctx.log.event(step.id, 'timeout', '超过 %.1f 秒' % limit)
                step.on_timeout(ctx)
                break
            io.wait_next_tick()
        step.finish(ctx)
        ctx.publish_motion()
