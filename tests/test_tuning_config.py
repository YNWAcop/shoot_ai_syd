# -*- coding: utf-8 -*-
from __future__ import print_function
import ast
import copy
import unittest
from test_fixed_targets import TREE, FakeROS, load_functions
from race_config import validate_tuning


class Vector(object):
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Twist(object):
    def __init__(self):
        self.linear, self.angular = Vector(), Vector()


class Publisher(object):
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class Stamp(object):
    def __init__(self, seconds=0):
        self.seconds = float(seconds)

    def is_zero(self):
        return self.seconds == 0

    def __lt__(self, other):
        return self.seconds < other.seconds


class RuntimeROS(FakeROS):
    def __init__(self, params=None):
        FakeROS.__init__(self, params=params)
        self.sleeps = []
        self.clock_s = 100.0
        parent = self

        class ROSTime(Stamp):
            @staticmethod
            def now():
                return ROSTime(parent.clock_s)

        self.Time = ROSTime

    def Publisher(self, *args, **kwargs):
        return Publisher()

    def Subscriber(self, *args, **kwargs):
        return None

    def Duration(self, value):
        return value

    def get_time(self):
        return self.clock_s

    def logwarn(self, *args):
        pass

    def sleep(self, seconds):
        self.sleeps.append(seconds)


class MoveBase(object):
    def wait_for_server(self, timeout):
        self.server_timeout = timeout


def load_real_controller(ros):
    context = load_functions(ros)
    context['load_tuning_config']()
    context.update({'Twist': Twist, 'PoseWithCovarianceStamped': object,
                    'Point': object, 'String': object, 'AlvarMarkers': object,
                    'LaserScan': object})
    client = MoveBase()

    class ActionLib(object):
        @staticmethod
        def SimpleActionClient(*args):
            return client

    context.update({'actionlib': ActionLib, 'MoveBaseAction': object})
    classes = [node for node in TREE.body if isinstance(node, ast.ClassDef)
               and node.name in ('PIController', 'NavigationDemo')]
    module = ast.Module(body=classes)
    module.type_ignores = []
    eval(compile(module, 'shoot_new.py', 'exec'), context)
    return context, context['NavigationDemo'](), client


def make_ar_marker(marker_id, x=.024, y=-.475):
    marker = type('Marker', (object,), {})()
    marker.id = marker_id
    marker.pose = type('Stamped', (object,), {})()
    marker.pose.pose = type('Pose', (object,), {})()
    marker.pose.pose.position = Vector()
    marker.pose.pose.position.x = x
    marker.pose.pose.position.y = y
    return marker


def make_ar_message(ros, seconds, markers):
    header = type('Header', (object,), {'stamp': ros.Time(seconds)})()
    return type('Markers', (object,), {'header': header, 'markers': markers})()


class TuningTests(unittest.TestCase):
    def test_frozen_old_defaults(self):
        config = validate_tuning({})
        self.assertEqual(config['navigation']['goals'], [
            [1.00, -0.50, 0.0], [0.17, -0.55, 0.0], [0.91, -1.45, 0.0],
            [0.17, -1.65, 0.0], [0.98, -2.85, 0.0], [0.40, -2.85, 0.0],
            [0.25, -1.60, 0.0], [0.25, -2.65, 0.0]])
        self.assertEqual(config['positioning'], {
            'configs': [{'left_target': .6, 'back_target': 1.282},
                        {'right_target': .618, 'back_target': 1.282},
                        {'right_target': .618, 'back_target': 1.281},
                        {'right_target': .218, 'back_target': .266}, {'back_target': .4}],
            'move_speed_mps': .13, 'terminal_speed_mps': .13, 'back_only_speed_mps': .18,
            'tolerance_m': .02, 'interval_s': .1, 'max_iterations': 1000})
        self.assertEqual(config['ar'], {'offset_x_m': .024,
            'stage2': {'x_tolerance_m': .03, 'y_min_m': -.48, 'y_max_m': -.47,
                       'frames_required': 1, 'kp': 1.45, 'ki': .5, 'max_angular_rps': 1.5},
            'stage3': {'x_tolerance_m': .02, 'frames_required': 3,
                       'kp': 1.4, 'ki': .5, 'max_angular_rps': 1.0}})
        self.assertEqual(config['circular'], {'template_id': 31, 'image_center_x_px': 340.,
            'alignment_tolerance_px': .5, 'kp_rps_per_px': .01,
            'max_angular_rps': .3, 'lock_delay_s': 2., 'fallback_fire_s': 9.})
        self.assertEqual(config['timing'], {'start_delay_s': 5., 'post_position_settle_s': 1.})
        self.assertEqual(config['navigation']['server_timeout_s'], 60.)
        self.assertEqual(config['navigation']['goal_timeout_s'], 60.)

    def test_overrides_do_not_pollute_defaults(self):
        config = validate_tuning({'ar': {'stage2': {'kp': .7}},
                                  'timing': {'start_delay_s': 3}})
        config['navigation']['goals'][0][0] = 123
        self.assertEqual(validate_tuning({})['navigation']['goals'][0][0], 1.)
        self.assertEqual(validate_tuning({})['ar']['stage2']['kp'], 1.45)

    def test_invalid_tuning_stops_before_hardware(self):
        invalid = [{'unknown': 1}, {'positioning': {'move_speed_mps': 0}},
            {'positioning': {'max_iterations': True}}, {'ar': {'stage2': {'ki': -1}}},
            {'ar': {'stage2': {'y_min_m': 0, 'y_max_m': -1}}},
            {'circular': {'image_center_x_px': float('nan')}},
            {'navigation': {'goal_timeout_s': float('inf')}},
            {'navigation': {'goals': [[0, 0, 0]]}},
            {'positioning': {'configs': [{}, {}, {}, {}, {}]}},
            {'circular': {'template_id': 255}}, {'timing': {'start_delay_s': '5'}},
            {'timing': {'voice_start_delay_s': 5}},
            {'circular': {'fallback_fire_s': -1}},
            {'circular': {'fallback_fire_s': float('nan')}},
            {'circular': {'fallback_fire_s': True}}]
        for override in invalid:
            ros = FakeROS({'~tuning': override})
            context = load_functions(ros)
            context['main']()
            self.assertIn(('error',), ros.events)
            self.assertNotIn(('serial_init',), ros.events)
            self.assertNotIn(('navigation_init',), ros.events)

    def test_actual_controller_uses_ar_and_server_settings(self):
        ros = RuntimeROS({'~tuning': {'navigation': {'server_timeout_s': 12},
              'ar': {'offset_x_m': .05, 'stage2': {'kp': .7, 'ki': 0, 'max_angular_rps': .2}}}})
        context, controller, client = load_real_controller(ros)
        self.assertEqual(client.server_timeout, 12)
        self.assertEqual(controller.ar_offset_x, .05)
        self.assertEqual(controller.angle_controller_area2.kp, .7)
        self.assertEqual(controller.angle_controller_area2.ki, 0)
        self.assertEqual(controller.angle_controller_area2.max_output, .2)
        self.assertEqual(controller.angle_controller_area2.update(10, .1), .2)
        self.assertEqual(controller.angle_controller_area2.update(-10, .1), -.2)
        self.assertEqual(controller.angle_controller_area2.integral, 0)

    def test_actual_circular_callback_uses_center_gain_and_limit(self):
        ros = RuntimeROS({'~tuning': {'circular': {
            'template_id': 30, 'image_center_x_px': 320, 'kp_rps_per_px': .02,
            'max_angular_rps': .1, 'fallback_fire_s': 0}}})
        context, controller, client = load_real_controller(ros)
        context['current_stage'] = 0
        context['shoot_permission_flag'] = 1

        class Detection(object):
            x, z = 330, 30

        controller.circular_target_callback(Detection())
        self.assertAlmostEqual(controller.cmd_vel_pub.messages[-1].angular.z, -.1)

    def test_actual_circular_callback_cannot_act_before_race_activation(self):
        for stage, permission in ((-1, 0), (-1, 1), (0, 0)):
            for x, marker_id in ((350, 31), (340, 31), (0, 255)):
                ros = RuntimeROS()
                context, controller, client = load_real_controller(ros)
                context['current_stage'] = stage
                context['shoot_permission_flag'] = permission
                shots, navigation = [], []
                controller.fire_shot = lambda delay: shots.append(delay)
                controller.goto = lambda goal: navigation.append(('goto', goal))
                controller.move_to_position = lambda index: navigation.append(('position', index))
                detection = type('Detection', (object,), {'x': x, 'z': marker_id})()
                controller.circular_target_callback(detection)
                case = 'stage=%d permission=%d x=%d z=%d' % (stage, permission, x, marker_id)
                self.assertEqual(controller.cmd_vel_pub.messages, [], case)
                self.assertEqual(shots, [], case)
                self.assertEqual(navigation, [], case)
                self.assertEqual(ros.sleeps, [], case)
                self.assertEqual(context['current_stage'], stage, case)
                self.assertFalse(controller.find_cb_executed, case)

    def test_circular_timeout_fires_once_and_transitions_after_arrival(self):
        for x, marker_id in ((350, 31), (0, 255), (340, 77)):
            ros = RuntimeROS()
            context, controller, client = load_real_controller(ros)
            context['current_stage'] = 0
            context['shoot_permission_flag'] = 1
            context['last_target_time'] = 90.0
            shots, navigation = [], []
            controller.fire_shot = lambda delay: shots.append(delay)

            def record_goal(goal):
                self.assertEqual(context['current_stage'], 1)
                self.assertTrue(controller.find_cb_executed)
                self.assertTrue(context['circular_target_shot'])
                navigation.append(('goto', tuple(goal)))
                # A further queued callback during transit must not fire again.
                lost = type('Detection', (object,), {'x': 0, 'z': 255})()
                controller.circular_target_callback(lost)

            controller.goto = record_goal
            controller.move_to_position = lambda index: navigation.append(('position', index))
            detection = type('Detection', (object,), {'x': x, 'z': marker_id})()
            controller.circular_target_callback(detection)
            self.assertEqual(shots, [.07, .07])
            self.assertEqual(navigation, [
                ('goto', tuple(context['NAVIGATION_GOALS'][1])), ('position', 4),
                ('goto', tuple(context['NAVIGATION_GOALS'][6])),
                ('goto', tuple(context['NAVIGATION_GOALS'][2])), ('position', 1)])
            self.assertEqual(ros.sleeps, [.2, 1.2, 1.])
            self.assertEqual(len(controller.cmd_vel_pub.messages), 1)
            self.assertEqual(controller.cmd_vel_pub.messages[0].angular.z, 0)
            self.assertEqual(context['current_stage'], 2)
            self.assertEqual(controller.consecutive_aligned_frames, 0)

    def test_circular_timeout_boundary_and_disable_do_not_force_a_shot(self):
        for fallback, elapsed, x, marker_id in (
                (9., 9., 350, 31), (0., 100., 350, 31), (0., 100., 0, 255)):
            ros = RuntimeROS({'~tuning': {'circular': {'fallback_fire_s': fallback}}})
            context, controller, client = load_real_controller(ros)
            context['current_stage'] = 0
            context['shoot_permission_flag'] = 1
            context['last_target_time'] = 100. - elapsed
            shots, navigation = [], []
            controller.fire_shot = lambda delay: shots.append(delay)
            controller.goto = lambda goal: navigation.append(goal)
            controller.move_to_position = lambda index: navigation.append(index)
            detection = type('Detection', (object,), {'x': x, 'z': marker_id})()
            controller.circular_target_callback(detection)
            self.assertEqual(shots, [])
            self.assertEqual(navigation, [])
            self.assertEqual(context['current_stage'], 0)
            self.assertFalse(controller.find_cb_executed)

    def test_ar_stage_entry_resets_pi_counts_and_time(self):
        ros = RuntimeROS()
        context, controller, client = load_real_controller(ros)
        controller.angle_controller_area2.integral = 1.5
        controller.angle_controller_area3.integral = -2.
        controller.consecutive_aligned_frames = 7
        controller.last_time = 20.
        ros.clock_s = 123.
        controller.enter_ar_stage(3)
        self.assertEqual(context['current_stage'], 3)
        self.assertEqual(controller.stage_start_stamp.seconds, 123.)
        self.assertEqual(controller.last_time, 123.)
        self.assertEqual(controller.consecutive_aligned_frames, 0)
        self.assertEqual(controller.angle_controller_area2.integral, 0)
        self.assertEqual(controller.angle_controller_area3.integral, 0)

    def test_ar_callback_discards_frames_from_before_current_stage(self):
        ros = RuntimeROS()
        context, controller, client = load_real_controller(ros)
        controller.enter_ar_stage(2)
        processed = []
        controller._process_ar_area2 = lambda marker: processed.append(marker.id)
        marker = make_ar_marker(context['target_id_rotating'])
        controller.ar_marker_callback(make_ar_message(ros, 99., [marker]))
        self.assertEqual(processed, [])
        controller.ar_marker_callback(make_ar_message(ros, 100., [marker]))
        self.assertEqual(processed, [1])

    def test_same_ar_frame_cannot_process_next_stage_after_transition(self):
        ros = RuntimeROS()
        context, controller, client = load_real_controller(ros)
        controller.enter_ar_stage(2)
        shots = []
        controller.fire_shot = lambda delay: shots.append(delay)
        controller.goto = lambda goal: setattr(ros, 'clock_s', ros.clock_s + 1.)
        controller.move_to_position = lambda index: None
        context['time'].sleep = lambda seconds: None
        markers = [make_ar_marker(context['target_id_rotating']),
                   make_ar_marker(context['target_id_moving'])]
        controller.ar_marker_callback(make_ar_message(ros, 100., markers))
        self.assertEqual(shots, [.08])
        self.assertEqual(context['current_stage'], 3)
        self.assertEqual(controller.stage_start_stamp.seconds, 103.)
        self.assertEqual(controller.consecutive_aligned_frames, 0)

    def test_control_dt_clamps_long_gaps_and_backward_clock(self):
        ros = RuntimeROS()
        context, controller, client = load_real_controller(ros)
        controller.last_time = 10.
        self.assertEqual(controller.update_time(), (100., .2))
        ros.clock_s = 99.
        self.assertEqual(controller.update_time(), (99., 0.))
        ros.clock_s = 99.05
        self.assertAlmostEqual(controller.update_time()[1], .05)

    def test_actual_position_loop_uses_speed_interval_and_iteration_limit(self):
        ros = RuntimeROS({'~tuning': {'positioning': {
            'terminal_speed_mps': .06, 'interval_s': .25, 'max_iterations': 2}}})
        context, controller, client = load_real_controller(ros)
        context['right_distance'], context['back_distance'] = 1., 1.
        controller.move_to_position(3)
        self.assertEqual(len(controller.cmd_vel_pub.messages), 2)
        self.assertEqual(ros.sleeps, [.25, .25])
        self.assertAlmostEqual(controller.cmd_vel_pub.messages[0].linear.x, -.06)
        self.assertAlmostEqual(controller.cmd_vel_pub.messages[0].linear.y, -.06)

    def test_main_uses_configured_delays_and_first_goal(self):
        goals = validate_tuning({})['navigation']['goals']
        goals[0] = [1.2, -.4, 10.]
        ros = FakeROS({'~tuning': {'timing': {'start_delay_s': 3.,
                    'post_position_settle_s': .6}, 'navigation': {'goals': goals}}})
        context = load_functions(ros)
        context['main']()
        self.assertIn(('sleep', 3.), ros.events)
        self.assertIn(('sleep', .6), ros.events)
        self.assertIn(('goto', (1.2, -.4, 10.)), ros.events)
        self.assertLess(ros.events.index(('keyboard_start',)), ros.events.index(('sleep', 3.)))
        self.assertLess(ros.events.index(('sleep', 3.)), ros.events.index(('goto', (1.2, -.4, 10.))))

    def test_actual_ar_callback_uses_y_window_and_frame_count(self):
        ros = RuntimeROS({'~tuning': {'ar': {'offset_x_m': .04,
            'stage2': {'x_tolerance_m': .05, 'y_min_m': -.46,
                       'y_max_m': -.44, 'frames_required': 2}}}})
        context, controller, client = load_real_controller(ros)
        shots = []
        controller.fire_shot = lambda delay: shots.append(delay)
        controller.goto = lambda goal: None
        controller.move_to_position = lambda index: None
        context['time'].sleep = lambda duration: None
        marker = type('Marker', (object,), {})()
        marker.pose = type('Stamped', (object,), {})()
        marker.pose.pose = type('Pose', (object,), {})()
        marker.pose.pose.position = Vector()
        marker.pose.pose.position.x = .05
        marker.pose.pose.position.y = -.45
        controller._process_ar_area2(marker)
        self.assertEqual(shots, [])
        controller._process_ar_area2(marker)
        self.assertEqual(shots, [.08])

    def test_actual_goto_uses_configured_goal_timeout(self):
        ros = RuntimeROS({'~tuning': {'navigation': {'goal_timeout_s': 17}}})
        context, controller, client = load_real_controller(ros)

        class Goal(object):
            def __init__(self):
                self.target_pose = type('Stamped', (object,), {})()
                self.target_pose.header = type('Header', (object,), {})()
                self.target_pose.pose = type('Pose', (object,), {})()
                self.target_pose.pose.position = Vector()
                self.target_pose.pose.orientation = Vector()

        context['MoveBaseGoal'] = Goal
        context['GoalStatus'] = type('Status', (object,), {'SUCCEEDED': 3})
        context['pi'] = 3.141592653589793
        context['transformations'] = type('Transforms', (object,), {
            'quaternion_from_euler': staticmethod(lambda *args: (0, 0, 0, 1))})
        ros.Time = type('Time', (object,), {'now': staticmethod(lambda: 0)})
        client.send_goal = lambda *args: None
        timeouts = []
        client.wait_for_result = lambda timeout: timeouts.append(timeout) or True
        client.get_state = lambda: 3
        controller.goto([1.2, -.4, 10])
        self.assertEqual(timeouts, [17])


if __name__ == '__main__':
    unittest.main(verbosity=2)
