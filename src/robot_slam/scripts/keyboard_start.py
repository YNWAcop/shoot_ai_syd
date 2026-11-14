# -*- coding: utf-8 -*-
"""Read an explicit terminal start command; no ROS or hardware dependencies."""
import select
import sys


def wait_for_start(is_shutdown, loginfo, logerr, stream=None, wait=None):
    """Wait for a line containing 1; return False on shutdown or input failure.

    /dev/tty gives a roslaunch child access to its controlling terminal even
    when standard input is redirected. The select timeout keeps Ctrl+C and
    ROS shutdown responsive. Injected stream/wait are for offline tests.
    """
    owned_stream = stream is None
    if owned_stream:
        try:
            stream = open('/dev/tty', 'r')
        except (IOError, OSError) as error:
            if not sys.stdin.isatty():
                logerr('No interactive terminal for start input: %s. Run the task in a terminal.', str(error))
                return False
            stream = sys.stdin
            owned_stream = False
    if wait is None:
        wait = select.select
    try:
        loginfo('Waiting for keyboard start: type 1 and press Enter...')
        while not is_shutdown():
            try:
                readable, _, _ = wait([stream], [], [], 0.2)
                if not readable:
                    continue
                line = stream.readline()
            except (IOError, OSError, ValueError, select.error) as error:
                if not is_shutdown():
                    logerr('Start input failed: %s', str(error))
                return False
            if is_shutdown():
                return False
            if line == '':
                logerr('Start input closed before confirmation.')
                return False
            if line.strip() == '1':
                loginfo('Keyboard start received')
                return True
            loginfo('Waiting: enter 1 and press Enter to start.')
        return False
    finally:
        if owned_stream:
            stream.close()
