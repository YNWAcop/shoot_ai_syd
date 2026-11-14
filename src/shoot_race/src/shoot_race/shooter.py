# -*- coding: utf-8 -*-
"""串口发射器。

协议沿用旧程序（实测可用）：
    开始射击：55 01 12 00 00 00 01 69
    停止射击：55 01 11 00 00 00 01 68
发"开始"后隔 burst_s 秒再发"停止"，这段时间长短决定一次打出几发（需在车上实测）。

注意：旧启动脚本里的 abot_bringup/shoot.launch 也会打开同一个串口。用本包时不再启动它，
串口只由这里使用。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import time


def hex_to_bytes(text):
    """'55 01 12 ...' → 字节串。"""
    return bytes(bytearray.fromhex(text))


class Shooter(object):
    def __init__(self, port, baudrate, start_hex, stop_hex):
        self.port = port
        self.baudrate = baudrate
        self.start_bytes = hex_to_bytes(start_hex)
        self.stop_bytes = hex_to_bytes(stop_hex)
        self.serial = None
        self.shots_fired = 0          # 本局一共发了几次开火指令
        self.last_error = ''          # 最近一次串口错误（没有则为空）

    def open(self):
        """打开串口。返回 (成功与否, 说明文字)。"""
        try:
            import serial
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate,
                                        parity='N', bytesize=8, stopbits=1, timeout=0.1)
            return True, '串口 %s 已打开' % self.port
        except Exception as error:  # 串口不存在、权限不足、被占用等
            self.serial = None
            return False, '串口 %s 打不开：%s' % (self.port, error)

    def is_ready(self):
        return self.serial is not None and self.serial.is_open

    def fire_burst(self, burst_s):
        """开一次火：发"开始"，等 burst_s 秒，发"停止"。返回是否真的发出了指令。

        burst_s 一般 0.07~0.1 秒，这里直接阻塞等待，计时最准；对主循环的影响可以忽略。
        """
        if not self.is_ready():
            return False
        try:
            self.serial.write(self.start_bytes)
            time.sleep(burst_s)
        except Exception as error:
            self.last_error = '%s' % error
            sent = False
        else:
            sent = True
        # 不管上面是否出错，都要尝试发"停止"，防止一直连发
        try:
            self.serial.write(self.stop_bytes)
        except Exception as error:
            # USB 串口抖动等：记下错误、返回 False，让比赛流程继续，不把异常抛给主循环
            self.last_error = '%s' % error
            sent = False
        if sent:
            self.shots_fired += 1
        return sent

    def safe_stop(self):
        """程序退出时调用：再发一次"停止"并关闭串口。"""
        if self.is_ready():
            try:
                self.serial.write(self.stop_bytes)
                self.serial.close()
            except Exception:
                pass
