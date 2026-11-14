# -*- coding: utf-8 -*-
"""Python2 / Python3 兼容的小工具。

车上的 ROS Melodic 用 Python2.7 运行节点；本包所有模块都写成 2/3 通用：
  - 每个模块开头都有 from __future__ import ... unicode_literals，
    让 Python2 里的字符串字面量也是 unicode，中文拼接不会出 UnicodeDecodeError；
  - 少数只接受"原生字符串"的接口（如 Python2 下的 cv2.imread 文件路径、日志输出），
    用 to_native() 转换一下。
"""
from __future__ import absolute_import, division, print_function, unicode_literals

import sys

PY2 = sys.version_info[0] == 2


def drop_python3_paths():
    """Python2 下把 Python3 / anaconda 的库目录从模块搜索路径里去掉。

    车上 ~/.bashrc 往 PYTHONPATH 里加了 /usr/lib/python3/dist-packages 等目录，
    Python2 会先找到那里的 Python3 版 yaml，报 SyntaxError（实测 2026-10-16）。
    旧的 N8NNWC.sh 开头 unset PYTHONPATH 就是为了这个；这里在程序内部再保险一次，
    这样即使在没清理环境的终端里直接 rosrun，也不会导入错版本的库。
    必须在 import yaml 等第三方库之前调用。
    """
    if PY2:
        sys.path[:] = [p for p in sys.path if 'python3' not in p and 'anaconda' not in p]


def to_native(text):
    """转成当前 Python 版本的原生 str：Python2 下是 UTF-8 字节串，Python3 下是 str。"""
    if PY2 and isinstance(text, unicode):  # noqa: F821  (Python2 才有 unicode 类型)
        return text.encode('utf-8')
    if not PY2 and isinstance(text, bytes):
        return text.decode('utf-8')
    return text


def to_text(value):
    """转成 unicode 文本（Python2 的 unicode / Python3 的 str）。"""
    if isinstance(value, bytes):
        return value.decode('utf-8', 'replace')
    if PY2:
        return unicode(value)  # noqa: F821
    return str(value)
