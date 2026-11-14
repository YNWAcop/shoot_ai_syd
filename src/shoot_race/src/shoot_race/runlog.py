# -*- coding: utf-8 -*-
"""每局运行记录。

每次启动 race_node 都会在日志目录（system.yaml 的 log_dir，默认 ~/race_logs）下新建一个
以时间命名的文件夹，里面有：
    events.csv            时间线：每一步的开始、结束、超时、跳过以及关键数据
    shots.csv             每一枪：哪个靶、怎么触发的（瞄准/预测/超时）、当时的误差
    config_snapshot.yaml  这一局使用的完整配置（事后分析时知道当时的参数）
    recognition.jpg       识别区拍到的卡片照片
    recognition.json      识别结果（离线、豆包各自的结果和最终采用的结果）

打偏了、超时了，打开这个文件夹就能还原当时发生了什么，不用靠回忆。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import io
import json
import os
import time

import yaml

from .compat import to_native, to_text


class RunLog(object):
    def __init__(self, log_dir, printer=None, clock=None):
        """log_dir：日志根目录。
        printer：打印函数（主程序里传 rospy.loginfo，测试时用 print）。
        clock：取当前时间的函数（主程序里用 ROS 时间，仿真测试里用仿真时间）。
        """
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.folder = os.path.join(os.path.expanduser(log_dir), stamp)
        if not os.path.isdir(self.folder):
            os.makedirs(self.folder)
        self.printer = printer or print
        self.clock = clock or time.time
        self.match_start = None      # 比赛开始时刻（输入 1 之后），用于计算"比赛第几秒"
        self.records = []            # [(比赛时间, 步骤, 事件, 说明)]，结束时汇总成用时表（summary）
        self._write_header('events.csv', 'match_time,step,event,detail\n')
        self._write_header('shots.csv', 'match_time,step,target,trigger,detail\n')

    # ---------- 内部工具 ----------
    def _path(self, name):
        return os.path.join(self.folder, name)

    def _write_header(self, name, header):
        with io.open(self._path(name), 'w', encoding='utf-8') as handle:
            handle.write(to_text(header))
            handle.flush()
            os.fsync(handle.fileno())

    def _append(self, name, line):
        # 每写一行都 fsync：2026-10-16 车跑到一半没电，events.csv、shots.csv 全是 0 字节
        # （数据还在内存缓存里没写到存储上）。fsync 每次约几毫秒，日志一秒最多几行，不影响主循环。
        with io.open(self._path(name), 'a', encoding='utf-8') as handle:
            handle.write(to_text(line))
            handle.flush()
            os.fsync(handle.fileno())

    def match_time(self):
        """比赛开始后经过的秒数；比赛还没开始时为 -1。"""
        if self.match_start is None:
            return -1.0
        return self.clock() - self.match_start

    def start_match(self):
        """输入 1 开赛时调用，此后日志里的时间都是"比赛第几秒"。"""
        self.match_start = self.clock()

    # ---------- 对外接口 ----------
    def info(self, text):
        """打印一行信息到终端（不写文件）。"""
        self.printer(to_native('[%6.1fs] %s' % (self.match_time(), to_text(text))))

    def event(self, step_id, event, detail=''):
        """记录一个事件：写入 events.csv 并打印。"""
        detail = to_text(detail).replace(',', ';')
        self._append('events.csv', '%.2f,%s,%s,%s\n' % (self.match_time(), step_id, event, detail))
        self.records.append((self.match_time(), to_text(step_id), to_text(event), detail))
        self.info('%s | %s %s' % (step_id, event, detail))

    def summary(self):
        """整局结束时的用时表：每一步用了几秒、是否超时，加上行驶 / 精定位里值得注意的事
        （被安全检查挡住几次、离终点多远交给精定位、通道按墙挪了多少、精定位最后误差）。
        打印到终端并存成 summary.txt。2026-10-21 加：跑完一眼看出慢在哪，不用先拉日志。"""
        steps = []                # 按出现顺序
        info = {}
        for t, step, event, detail in self.records:
            if step in ('config', 'mission', 'preflight'):
                continue
            item = info.get(step)
            if item is None:
                item = info[step] = {'time': None, 'result': '', 'notes': [], 'blocked': 0, 'boost': 0}
                steps.append(step)
            if event in ('done', 'timeout'):
                numbers = [w for w in detail.replace('秒', ' ').split() if _is_number(w)]
                item['time'] = float(numbers[0]) if numbers else None
                item['result'] = '超时' if event == 'timeout' else ''
            elif event == 'skip':
                item['result'] = '跳过'
            elif event == 'blocked':
                item['blocked'] += 1
            elif event == 'boost':
                item['boost'] += 1
            elif event in ('near_goal', 'lane', 'near', 'partial', 'align_error', 'wall_rejected', 'error', 'warning'):
                item['notes'].append(detail if event != 'align_error' else '误差 ' + detail)
        lines = ['=' * 20 + ' 本局用时 ' + '=' * 20, '%-14s %7s  %-4s %s' % ('步骤', '用时(秒)', '结果', '备注')]
        groups = [('行驶', 'go_'), ('精定位', 'align_'), ('播报/识别', ('say_', 'recognize')), ('打靶/停靠', 'shoot_')]
        totals = dict((name, 0.0) for name, _ in groups)
        timeouts = []
        for step in steps:
            item = info[step]
            notes = list(item['notes'])
            if item['blocked']:
                notes.insert(0, '被安全检查挡住 %d 次' % item['blocked'])
            if item['boost']:
                notes.append('蹭不动加大最小速度 %d 次' % item['boost'])
            text = '%.2f' % item['time'] if item['time'] is not None else '--'
            lines.append('%-14s %7s  %-4s %s' % (step, text, item['result'], '；'.join(notes)))
            if item['result'] == '超时':
                timeouts.append(step)
            for name, prefix in groups:
                if item['time'] is not None and step.startswith(prefix):
                    totals[name] += item['time']
        lines.append('-' * 50)
        lines.append('  '.join('%s %.1f' % (name, totals[name]) for name, _ in groups)
                     + '  （路上 = 行驶 + 精定位 %.1f 秒）' % (totals['行驶'] + totals['精定位']))
        lines.append('总用时 %.1f 秒；超时：%s' % (self.match_time(), '、'.join(timeouts) if timeouts else '无'))
        with io.open(self._path('summary.txt'), 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(lines) + '\n')
        for line in lines:
            self.printer(to_native(line))
        return lines

    def shot(self, step_id, target, trigger, detail=''):
        """记录一次开火。trigger：'aimed'（瞄准开火）/'predicted'（预测开火）/'timeout'（超时兜底）。"""
        detail = to_text(detail).replace(',', ';')
        self._append('shots.csv', '%.2f,%s,%s,%s,%s\n' % (self.match_time(), step_id, target, trigger, detail))
        self.info('%s | 开火 target=%s trigger=%s %s' % (step_id, target, trigger, detail))

    def save_json(self, name, data):
        with io.open(self._path(name), 'w', encoding='utf-8') as handle:
            handle.write(to_text(json.dumps(data, ensure_ascii=False, indent=2)))

    def save_config(self, cfg):
        with io.open(self._path('config_snapshot.yaml'), 'w', encoding='utf-8') as handle:
            handle.write(to_text(yaml.safe_dump(cfg, allow_unicode=True, default_flow_style=False)))
            handle.flush()
            os.fsync(handle.fileno())

    def save_image(self, name, bgr_image):
        """保存图片（需要 OpenCV）。"""
        import cv2
        cv2.imwrite(to_native(self._path(name)), bgr_image)


def _is_number(text):
    try:
        float(text)
        return True
    except ValueError:
        return False
