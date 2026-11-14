#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Offline startup regression tests; no ROS imports, serial access or motion."""
from __future__ import print_function

import ast
import io
import os
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_slam', 'scripts'))
from race_config import validate_tuning
SOURCE_PATH = os.path.join(ROOT, 'src', 'robot_slam', 'scripts', 'shoot_new.py')
with io.open(SOURCE_PATH, 'rb') as source_file:
    SOURCE = source_file.read()
TREE = ast.parse(SOURCE, filename=SOURCE_PATH)


class FakeROS(object):
    def __init__(self, params=None, confirm_start=True):
        self.params = dict(params or {})
        self.confirm_start = confirm_start
        self.start_checks = 0
        self.start_confirmed = False
        self.stopped = False
        self.events = []

    def init_node(self, *args, **kwargs):
        self.events.append(('init',))

    def get_param(self, name, default=None):
        assert name != '/start', 'The controller must not read the old voice gate'
        return self.params.get(name, default)

    def set_param(self, name, value):
        assert name != '/start', 'The controller must not write the old voice gate'
        self.params[name] = value
        self.events.append(('param', name, value))

    def loginfo(self, *args):
        pass

    def logerr(self, *args):
        self.events.append(('error',))

    def is_shutdown(self):
        return self.stopped

    def sleep(self, seconds):
        self.events.append(('sleep', seconds))

    def Rate(self, frequency):
        parent = self

        class FakeRate(object):
            def sleep(self):
                parent.stopped = True

        return FakeRate()


def load_functions(ros):
    """Compile the real configuration and main functions in a mocked context."""
    constant_names = {
        'AR_MARKER_ID_AREA2', 'AR_MARKER_ID_AREA3', 'NAVIGATION_GOALS',
        'target_id_rotating', 'target_id_moving', 'current_stage',
        'last_target_time', 'shoot_permission_flag', 'circular_target_shot', 'TUNING',
        'MAX_CONTROL_DT', 'NO_OBJECT_ID'
    }
    body = []
    for node in TREE.body:
        if isinstance(node, ast.ImportFrom) and node.module == 'numbers':
            body.append(node)
        elif isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id in constant_names
                   for target in node.targets):
                body.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in ('load_target_ids', 'load_tuning_config', 'main'):
            body.append(node)
    module = ast.Module(body=body)
    module.type_ignores = []
    context = {'rospy': ros, 'validate_tuning': validate_tuning}

    class FakeTime(object):
        @staticmethod
        def time():
            return 100.0

    class FakeNavigation(object):
        def __init__(self):
            assert context['current_stage'] == -1
            ros.events.append(('navigation_init',))

        def goto(self, goal):
            assert ros.start_confirmed
            assert context['current_stage'] == -1
            assert context['shoot_permission_flag'] == 0
            ros.events.append(('goto', tuple(goal)))

        def move_to_position(self, position):
            assert ros.start_confirmed
            assert context['current_stage'] == -1
            assert context['shoot_permission_flag'] == 0
            ros.events.append(('position', position))

    def fake_wait_for_start(is_shutdown, loginfo, logerr):
        assert is_shutdown == ros.is_shutdown
        assert loginfo == ros.loginfo
        assert logerr == ros.logerr
        assert context['current_stage'] == -1
        assert context['shoot_permission_flag'] == 0
        ros.start_checks += 1
        if is_shutdown() or not ros.confirm_start:
            ros.stopped = True
            return False
        ros.start_confirmed = True
        ros.events.append(('keyboard_start',))
        return True

    context['time'] = FakeTime()
    context['NavigationDemo'] = FakeNavigation
    context['wait_for_start'] = fake_wait_for_start
    context['initialize_serial_connection'] = lambda: ros.events.append(('serial_init',))
    eval(compile(module, SOURCE_PATH, 'exec'), context)
    return context


class FixedTargetTests(unittest.TestCase):
    def test_default_and_all_supported_target_pairs(self):
        context = load_functions(FakeROS())
        context['load_target_ids']()
        self.assertEqual((context['target_id_rotating'], context['target_id_moving']), (1, 6))
        for rotating in range(1, 6):
            for moving in range(6, 9):
                context = load_functions(FakeROS({
                    '~target_id_rotating': rotating, '~target_id_moving': moving
                }))
                context['load_target_ids']()
                self.assertEqual((context['target_id_rotating'], context['target_id_moving']),
                                 (rotating, moving))

    def test_invalid_target_configuration_stops_before_hardware(self):
        for parameter, invalid in (
                ('~target_id_rotating', 0), ('~target_id_rotating', 6),
                ('~target_id_rotating', True), ('~target_id_rotating', '1'),
                ('~target_id_rotating', 1.0), ('~target_id_rotating', None),
                ('~target_id_moving', 5), ('~target_id_moving', 9),
                ('~target_id_moving', False), ('~target_id_moving', '6')):
            ros = FakeROS({parameter: invalid})
            context = load_functions(ros)
            context['main']()
            self.assertIn(('error',), ros.events)
            self.assertNotIn(('serial_init',), ros.events)
            self.assertNotIn(('navigation_init',), ros.events)

    def test_keyboard_confirmation_and_delay_gate_navigation(self):
        ros = FakeROS()
        context = load_functions(ros)
        context['main']()
        self.assertEqual(ros.start_checks, 1)
        self.assertLess(ros.events.index(('keyboard_start',)), ros.events.index(('sleep', 5)))
        goto = ('goto', tuple(context['NAVIGATION_GOALS'][0]))
        self.assertLess(ros.events.index(('sleep', 5)), ros.events.index(goto))
        self.assertEqual((context['target_id_rotating'], context['target_id_moving']), (1, 6))
        self.assertEqual(context['current_stage'], 0)
        self.assertEqual(context['shoot_permission_flag'], 1)

    def test_shutdown_while_waiting_does_not_enter_race(self):
        ros = FakeROS(confirm_start=False)
        context = load_functions(ros)
        context['main']()
        self.assertEqual(ros.start_checks, 1)
        self.assertFalse(ros.start_confirmed)
        self.assertEqual(context['current_stage'], -1)
        self.assertEqual(context['shoot_permission_flag'], 0)
        self.assertFalse(any(event[0] == 'goto' for event in ros.events))
        self.assertNotIn(('sleep', 5), ros.events)

    def test_stale_voice_start_cannot_replace_keyboard_confirmation(self):
        ros = FakeROS({'/start': True}, confirm_start=False)
        context = load_functions(ros)
        context['main']()
        self.assertEqual(ros.start_checks, 1)
        self.assertIs(ros.params['/start'], True)
        self.assertFalse(ros.start_confirmed)
        self.assertEqual(context['current_stage'], -1)
        self.assertEqual(context['shoot_permission_flag'], 0)
        self.assertFalse(any(event[0] in ('goto', 'position') for event in ros.events))
        self.assertNotIn(('keyboard_start',), ros.events)
        self.assertNotIn(('sleep', 5), ros.events)

    def test_active_controller_has_no_number_recording_or_topic_override(self):
        literal_type = ast.Constant if hasattr(ast, 'Constant') else ast.Str
        topic_strings = [node.value if hasattr(node, 'value') else node.s
                         for node in ast.walk(TREE)
                         if isinstance(node, literal_type)]
        self.assertNotIn('audio_topic', topic_strings)
        self.assertNotIn('target_id_rotating', topic_strings)
        self.assertNotIn('target_id_moving', topic_strings)
        self.assertNotIn('/start', topic_strings)
        compile(SOURCE, SOURCE_PATH, 'exec')


if __name__ == '__main__':
    unittest.main(verbosity=2)
