#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Pure Python 2/3 tests of the keyboard gate; no ROS, TTY or hardware I/O."""
from __future__ import print_function

import errno
import os
import select
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, 'src', 'robot_slam', 'scripts')
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)
import keyboard_start


class Mock(object):
    """Small callable mock avoiding the Python 3-only unittest.mock import."""
    def __init__(self, side_effect=None):
        self.calls = []
        self.side_effect = side_effect

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        if isinstance(self.side_effect, BaseException):
            raise self.side_effect
        if self.side_effect is not None:
            return self.side_effect(*args, **kwargs)


class FakeStream(object):
    def __init__(self, lines, after_read=None, read_error=None, tty=False):
        self.lines = list(lines)
        self.after_read = after_read
        self.read_error = read_error
        self.read_count = 0
        self.closed = False
        self.tty = tty

    def isatty(self):
        return self.tty

    def readline(self):
        self.read_count += 1
        if self.read_error is not None:
            raise self.read_error
        line = self.lines.pop(0) if self.lines else ''
        if self.after_read is not None:
            self.after_read()
        return line

    def close(self):
        self.closed = True


class FakeWait(object):
    """select-compatible readiness mock, with optional initial timeout polls."""
    def __init__(self, timeouts=0, error=None, after_wait=None):
        self.timeouts = timeouts
        self.error = error
        self.after_wait = after_wait
        self.calls = []

    def __call__(self, readers, writers, errors, timeout):
        self.calls.append((list(readers), list(writers), list(errors), timeout))
        if self.error is not None:
            raise self.error
        if self.after_wait is not None:
            self.after_wait()
        if self.timeouts:
            self.timeouts -= 1
            return [], [], []
        return list(readers), [], []


class KeyboardStartTests(unittest.TestCase):
    def setUp(self):
        self.info = Mock()
        self.error = Mock()
        self.stopped = False

    def is_shutdown(self):
        return self.stopped

    def stop(self):
        self.stopped = True

    def run_gate(self, stream, wait=None):
        return keyboard_start.wait_for_start(
            self.is_shutdown, self.info, self.error,
            stream=stream, wait=wait if wait is not None else FakeWait())

    def test_only_exact_stripped_one_confirms(self):
        for line in ('1\n', ' 1 \n', '\t1\r\n'):
            stream = FakeStream([line])
            self.assertTrue(self.run_gate(stream))
            self.assertEqual(stream.read_count, 1)

    def test_other_input_prompts_and_keeps_waiting(self):
        rejected = ['\n', '0\n', '2\n', '01\n', '1.0\n', '1x\n', 'start\n']
        stream = FakeStream(rejected + ['1\n'])
        self.assertTrue(self.run_gate(stream))
        self.assertEqual(stream.read_count, len(rejected) + 1)
        self.assertGreaterEqual(len(self.info.calls), len(rejected) + 1)

    def test_eof_never_confirms(self):
        stream = FakeStream([])
        self.assertFalse(self.run_gate(stream))
        self.assertEqual(stream.read_count, 1)

    def test_invalid_input_then_eof_never_confirms(self):
        stream = FakeStream(['1.0\n'])
        self.assertFalse(self.run_gate(stream))
        self.assertEqual(stream.read_count, 2)

    def test_shutdown_before_wait_never_reads(self):
        self.stop()
        stream = FakeStream(['1\n'])
        wait = FakeWait()
        self.assertFalse(self.run_gate(stream, wait))
        self.assertEqual(stream.read_count, 0)
        self.assertEqual(wait.calls, [])

    def test_timeout_polls_then_confirmation(self):
        stream = FakeStream(['1\n'])
        wait = FakeWait(timeouts=2)
        self.assertTrue(self.run_gate(stream, wait))
        self.assertEqual(len(wait.calls), 3)
        self.assertEqual(stream.read_count, 1)
        for readers, writers, errors, timeout in wait.calls:
            self.assertEqual(readers, [stream])
            self.assertEqual(writers, [])
            self.assertEqual(errors, [])
            self.assertAlmostEqual(timeout, 0.2)

    def test_shutdown_during_empty_poll_never_confirms(self):
        stream = FakeStream(['1\n'])
        wait = FakeWait(timeouts=1, after_wait=self.stop)
        self.assertFalse(self.run_gate(stream, wait))
        self.assertEqual(stream.read_count, 0)
        self.assertEqual(len(wait.calls), 1)

    def test_shutdown_after_read_overrides_confirmation(self):
        stream = FakeStream(['1\n'], after_read=self.stop)
        self.assertFalse(self.run_gate(stream))
        self.assertEqual(stream.read_count, 1)

    def test_read_error_logs_and_never_confirms(self):
        stream = FakeStream([], read_error=IOError(errno.EIO, 'mock read error'))
        self.assertFalse(self.run_gate(stream))
        self.assertTrue(self.error.calls)

    def test_select_error_logs_and_never_reads(self):
        stream = FakeStream(['1\n'])
        wait = FakeWait(error=select.error(errno.EIO, 'mock select error'))
        self.assertFalse(self.run_gate(stream, wait))
        self.assertEqual(stream.read_count, 0)
        self.assertTrue(self.error.calls)

    def test_python2_distinct_select_error_is_handled(self):
        class LegacySelectError(Exception):
            pass

        original = keyboard_start.select.error
        keyboard_start.select.error = LegacySelectError
        try:
            stream = FakeStream(['1\n'])
            wait = FakeWait(error=LegacySelectError('mock Python 2 select error'))
            self.assertFalse(self.run_gate(stream, wait))
            self.assertEqual(stream.read_count, 0)
            self.assertTrue(self.error.calls)
        finally:
            keyboard_start.select.error = original

    def test_injected_stream_stays_open(self):
        stream = FakeStream(['1\n'])
        self.assertTrue(self.run_gate(stream))
        self.assertFalse(stream.closed)

    def call_with_fake_terminal(self, opener, wait, stdin=None):
        had_open = hasattr(keyboard_start, 'open')
        previous_open = getattr(keyboard_start, 'open', None)
        previous_stdin = keyboard_start.sys.stdin
        keyboard_start.open = opener
        keyboard_start.sys.stdin = stdin if stdin is not None else FakeStream([])
        try:
            return keyboard_start.wait_for_start(
                self.is_shutdown, self.info, self.error, wait=wait)
        finally:
            keyboard_start.sys.stdin = previous_stdin
            if had_open:
                keyboard_start.open = previous_open
            else:
                del keyboard_start.open

    def test_missing_terminal_and_non_tty_stdin_logs_and_exits(self):
        opener = Mock(side_effect=IOError(errno.ENXIO, 'no controlling terminal'))
        wait = FakeWait()
        stdin = FakeStream(['1\n'], tty=False)
        self.assertFalse(self.call_with_fake_terminal(opener, wait, stdin=stdin))
        self.assertEqual(len(opener.calls), 1)
        self.assertEqual(opener.calls[0][0], ('/dev/tty', 'r'))
        self.assertEqual(wait.calls, [])
        self.assertEqual(stdin.read_count, 0)
        self.assertFalse(stdin.closed)
        self.assertTrue(self.error.calls)

    def test_inherited_tty_stdin_confirms_and_stays_open(self):
        opener = Mock(side_effect=IOError(errno.ENXIO, 'no controlling terminal'))
        wait = FakeWait()
        stdin = FakeStream(['1\n'], tty=True)
        self.assertTrue(self.call_with_fake_terminal(opener, wait, stdin=stdin))
        self.assertEqual(opener.calls[0][0], ('/dev/tty', 'r'))
        self.assertEqual(stdin.read_count, 1)
        self.assertEqual(len(wait.calls), 1)
        self.assertEqual(wait.calls[0][0], [stdin])
        self.assertFalse(stdin.closed)

    def test_inherited_tty_stdin_eof_exits_and_stays_open(self):
        opener = Mock(side_effect=IOError(errno.ENXIO, 'no controlling terminal'))
        stdin = FakeStream([], tty=True)
        self.assertFalse(self.call_with_fake_terminal(opener, FakeWait(), stdin=stdin))
        self.assertEqual(stdin.read_count, 1)
        self.assertFalse(stdin.closed)
        self.assertTrue(self.error.calls)

    def test_owned_terminal_closes_after_confirmation(self):
        stream = FakeStream(['1\n'])
        stdin = FakeStream(['1\n'], tty=True)
        opener = Mock(side_effect=lambda *args, **kwargs: stream)
        self.assertTrue(self.call_with_fake_terminal(opener, FakeWait(), stdin=stdin))
        self.assertEqual(opener.calls[0][0], ('/dev/tty', 'r'))
        self.assertTrue(stream.closed)
        self.assertEqual(stdin.read_count, 0)
        self.assertFalse(stdin.closed)

    def test_owned_terminal_closes_after_eof(self):
        stream = FakeStream([])
        opener = Mock(side_effect=lambda *args, **kwargs: stream)
        self.assertFalse(self.call_with_fake_terminal(opener, FakeWait()))
        self.assertTrue(stream.closed)

    def test_owned_terminal_closes_after_select_error(self):
        stream = FakeStream(['1\n'])
        opener = Mock(side_effect=lambda *args, **kwargs: stream)
        wait = FakeWait(error=select.error(errno.EIO, 'mock select error'))
        self.assertFalse(self.call_with_fake_terminal(opener, wait))
        self.assertTrue(stream.closed)


if __name__ == '__main__':
    unittest.main()
