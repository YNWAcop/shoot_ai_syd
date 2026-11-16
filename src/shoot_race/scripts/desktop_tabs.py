#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Ubuntu desktop tabs. Log viewers never initialize hardware or publish motion."""
from __future__ import print_function
import os
import sys
import time
import subprocess
import shlex
try:
    from shlex import quote
except ImportError:
    from pipes import quote

TABS = [('ROS master', 'roscore'), ('Chassis + IMU', 'chassis'),
        ('Shooting log', 'race'), ('Navigation', 'navigation'),
        ('Camera', 'camera'), ('AR + RViz', 'ar'),
        ('Image recognition', 'recognition')]


def command_for(args):
    return ' '.join(quote(arg) for arg in args)


def terminal_args(ws, logdir, race_args):
    helper = os.path.abspath(__file__)
    args = ['gnome-terminal', '--show-menubar']
    for index, (title, stream) in enumerate(TABS):
        args += ['--window' if index == 0 else '--tab', '--title=syd | ' + title,
                 '--working-directory=' + ws, '-e',
                 command_for(['/usr/bin/python2', helper, 'watch', logdir, stream])]
    args += ['--tab', '--active', '--title=syd | Competition - enter 1',
             '--working-directory=' + ws, '-e',
             command_for(['env', 'SYD_RVIZ=' + os.environ.get('SYD_RVIZ', 'true'),
                          '/usr/bin/python2', helper, 'run', ws, logdir] + race_args)]
    return args


def watch(logdir, stream):
    print('syd module log: ' + stream)
    print('Session: ' + logdir)
    if stream == 'race':
        print('Shooting is controlled by the competition process; this tab only displays its log.')
    path = os.path.join(logdir, stream + '.log')
    offset = 0
    deadline = time.time() + 60
    while True:
        if os.path.isfile(path):
            with open(path, 'rb') as handle:
                handle.seek(offset)
                data = handle.read()
                offset = handle.tell()
            if data:
                sys.stdout.write(data.decode('utf-8', 'replace').encode('utf-8') if sys.version_info[0] == 2 else data.decode('utf-8', 'replace'))
                sys.stdout.flush()
        if os.path.exists(os.path.join(logdir, 'finished')):
            break
        if time.time() > deadline and not os.path.exists(os.path.join(logdir, 'started')):
            print('Session did not start. Inspect the Competition tab.')
            break
        time.sleep(0.2)
    try:
        (raw_input if sys.version_info[0] == 2 else input)('\nSession ended. Press Enter to close this tab. ')
    except (EOFError, KeyboardInterrupt):
        pass


def main():
    mode = sys.argv[1]
    if mode == 'open':
        return subprocess.call(terminal_args(sys.argv[2], sys.argv[3], sys.argv[4:]))
    if mode == 'watch':
        try:
            watch(sys.argv[2], sys.argv[3])
        except KeyboardInterrupt:
            pass
        return 0
    if mode == 'run':
        ws, logdir = sys.argv[2:4]
        os.environ['SYD_SESSION_LOG_DIR'] = logdir
        os.environ['SYD_DESKTOP_MODULES'] = 'true'
        # Explicit env is also needed when gnome-terminal-server already exists.
        command = 'source "$1/env_syd.sh"; shift; exec bash "$@"'
        try:
            return subprocess.call(['bash', '-c', command, 'syd', ws,
                                    os.path.join(ws, 'run_syd_session.sh')] + sys.argv[4:])
        finally:
            open(os.path.join(logdir, 'finished'), 'a').close()
    raise ValueError('Unknown mode')


if __name__ == '__main__':
    sys.exit(main())
