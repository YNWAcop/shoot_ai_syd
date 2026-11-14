# -*- coding: utf-8 -*-
"""语音播报：按顺序播放几段预先录好的音频。

为什么用预录音频而不用在线 TTS：
  - 离线、确定，不依赖网络和讯飞账号（旧代码里讯飞账号密码是明文写死的）；
  - 规则要求"播报内容为统一内容"，官方公布内容后只需要替换音频文件。

播放在后台线程里进行，主循环用 is_playing() 查询是否播完，不会卡住整个程序。
规则要求播报时车必须静止，这一点由 steps.SayStep 保证（先停车再播）；
SayStep 超时会调用 stop()，stop() 保证返回后不会再有声音（不会出现"边播边开车"）。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import os
import subprocess
import threading

from .compat import to_native


class VoicePlayer(object):
    def __init__(self, audio_dir, player_command):
        """audio_dir：音频所在目录。player_command：播放命令，例如 ['mplayer', '-really-quiet']。"""
        self.audio_dir = os.path.expanduser(audio_dir)
        self.player_command = list(player_command)
        self._lock = threading.Lock()      # 保护 _process，防止"刚要启动播放器时被要求停止"的竞态
        self._thread = None
        self._process = None
        self._stop_event = None            # 每次播放一个独立的停止标志

    def full_path(self, file_name):
        return os.path.join(self.audio_dir, file_name)

    def missing_files(self, file_names):
        """返回不存在的音频文件列表（开局自检用）。"""
        return [name for name in file_names if not os.path.exists(self.full_path(name))]

    def start(self, file_names):
        """开始在后台依次播放。如果上一次还没播完，先把它停掉。"""
        self.stop()
        stop_event = threading.Event()
        self._stop_event = stop_event
        paths = [self.full_path(name) for name in file_names]
        self._thread = threading.Thread(target=self._play_all, args=(paths, stop_event))
        self._thread.daemon = True
        self._thread.start()

    def _play_all(self, paths, stop_event):
        devnull = open(os.devnull, 'r+b')
        try:
            for path in paths:
                if not os.path.exists(path):
                    continue
                command = [to_native(part) for part in self.player_command + [path]]
                with self._lock:
                    # 在锁里再检查一次：如果 stop() 已经被调用，就不要再启动播放器
                    if stop_event.is_set():
                        return
                    try:
                        # stdin 也重定向，避免播放器读终端按键（那个终端要用来输入 1）
                        self._process = subprocess.Popen(command, stdin=devnull, stdout=devnull, stderr=devnull)
                    except OSError:
                        # 播放器不存在等情况：跳过（开局自检会提前报出来）
                        self._process = None
                        continue
                    process = self._process
                process.wait()
                with self._lock:
                    self._process = None
        finally:
            devnull.close()

    def is_playing(self):
        return self._thread is not None and self._thread.is_alive()

    def stop(self):
        """停止播放，返回时保证播放器已经结束。"""
        if self._stop_event is not None:
            self._stop_event.set()
        with self._lock:
            if self._process is not None:
                try:
                    self._process.terminate()
                except OSError:
                    pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
