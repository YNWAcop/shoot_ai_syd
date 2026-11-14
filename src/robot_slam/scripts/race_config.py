# -*- coding: utf-8 -*-
"""Pure configuration validation, compatible with Python 2 and Python 3."""
import copy
import math
from numbers import Integral, Real

DEFAULT_TUNING = {
    'navigation': {
        'goals': [[1.00, -0.50, 0.0], [0.17, -0.55, 0.0],
                  [0.91, -1.45, 0.0], [0.17, -1.65, 0.0],
                  [0.98, -2.85, 0.0], [0.40, -2.85, 0.0],
                  [0.25, -1.60, 0.0], [0.25, -2.65, 0.0]],
        'server_timeout_s': 60.0, 'goal_timeout_s': 60.0,
    },
    'positioning': {
        'configs': [{'left_target': 0.6, 'back_target': 1.282},
                    {'right_target': 0.618, 'back_target': 1.282},
                    {'right_target': 0.618, 'back_target': 1.281},
                    {'right_target': 0.218, 'back_target': 0.266},
                    {'back_target': 0.40}],
        'move_speed_mps': 0.13, 'terminal_speed_mps': 0.13,
        'back_only_speed_mps': 0.18, 'tolerance_m': 0.02,
        'interval_s': 0.1, 'max_iterations': 1000,
    },
    'ar': {
        'offset_x_m': 0.024,
        'stage2': {'x_tolerance_m': 0.030, 'y_min_m': -0.480,
                   'y_max_m': -0.470, 'frames_required': 1,
                   'kp': 1.45, 'ki': 0.5, 'max_angular_rps': 1.5},
        'stage3': {'x_tolerance_m': 0.02, 'frames_required': 3,
                   'kp': 1.40, 'ki': 0.5, 'max_angular_rps': 1.0},
    },
    'circular': {
        'template_id': 31, 'image_center_x_px': 340.0,
        'alignment_tolerance_px': 0.5, 'kp_rps_per_px': 0.01,
        'max_angular_rps': 0.3, 'lock_delay_s': 2.0,
        'fallback_fire_s': 9.0,
    },
    'timing': {'start_delay_s': 5.0, 'post_position_settle_s': 1.0},
}


def _merge(default, override, path):
    if not isinstance(override, dict):
        raise ValueError('%s must be a dictionary' % path)
    result = copy.deepcopy(default)
    for key, value in override.items():
        if key not in default:
            raise ValueError('Unknown parameter: %s.%s' % (path, key))
        if isinstance(default[key], dict):
            result[key] = _merge(default[key], value, path + '.' + key)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _number(value, path, minimum=None, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('%s must be a number' % path)
    number = float(value)
    if math.isnan(number) or math.isinf(number):
        raise ValueError('%s must be finite' % path)
    if positive and number <= 0:
        raise ValueError('%s must be greater than zero' % path)
    if minimum is not None and number < minimum:
        raise ValueError('%s must be at least %s' % (path, minimum))
    return number


def _integer(value, path, minimum):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
        raise ValueError('%s must be an integer >= %d' % (path, minimum))
    return int(value)


def validate_tuning(override):
    """Merge partial overrides and reject invalid input before hardware setup."""
    config = _merge(DEFAULT_TUNING, override, 'tuning')
    navigation = config['navigation']
    goals = navigation['goals']
    if not isinstance(goals, list) or len(goals) != 8:
        raise ValueError('tuning.navigation.goals must contain exactly 8 goals')
    for index, goal in enumerate(goals):
        if not isinstance(goal, list) or len(goal) != 3:
            raise ValueError('Goal %d must contain [x_m, y_m, yaw_degrees]' % index)
        goals[index] = [_number(value, 'tuning.navigation.goals[%d]' % index)
                        for value in goal]
    for key in ('server_timeout_s', 'goal_timeout_s'):
        navigation[key] = _number(navigation[key], 'tuning.navigation.' + key, positive=True)

    positioning = config['positioning']
    positions = positioning['configs']
    if not isinstance(positions, list) or len(positions) != 5:
        raise ValueError('tuning.positioning.configs must contain exactly 5 entries')
    for index, position in enumerate(positions):
        expected = set(DEFAULT_TUNING['positioning']['configs'][index])
        if not isinstance(position, dict) or set(position) != expected:
            raise ValueError('Position %d must use keys %s' % (index, sorted(expected)))
        for key in expected:
            position[key] = _number(position[key], 'tuning.positioning.configs[%d].%s' % (index, key), positive=True)
    for key in ('move_speed_mps', 'terminal_speed_mps', 'back_only_speed_mps',
                'tolerance_m', 'interval_s'):
        positioning[key] = _number(positioning[key], 'tuning.positioning.' + key, positive=True)
    positioning['max_iterations'] = _integer(positioning['max_iterations'], 'tuning.positioning.max_iterations', 1)

    ar = config['ar']
    ar['offset_x_m'] = _number(ar['offset_x_m'], 'tuning.ar.offset_x_m')
    for stage in ('stage2', 'stage3'):
        settings = ar[stage]
        prefix = 'tuning.ar.' + stage + '.'
        for key in ('x_tolerance_m', 'max_angular_rps'):
            settings[key] = _number(settings[key], prefix + key, positive=True)
        for key in ('kp', 'ki'):
            settings[key] = _number(settings[key], prefix + key, minimum=0)
        settings['frames_required'] = _integer(settings['frames_required'], prefix + 'frames_required', 1)
    stage2 = ar['stage2']
    for key in ('y_min_m', 'y_max_m'):
        stage2[key] = _number(stage2[key], 'tuning.ar.stage2.' + key)
    if stage2['y_min_m'] > stage2['y_max_m']:
        raise ValueError('tuning.ar.stage2.y_min_m must not exceed y_max_m')

    circular = config['circular']
    circular['template_id'] = _integer(circular['template_id'], 'tuning.circular.template_id', 0)
    if circular['template_id'] == 255:
        raise ValueError('template_id 255 is reserved for target loss')
    for key in ('image_center_x_px', 'alignment_tolerance_px', 'kp_rps_per_px', 'lock_delay_s',
                'fallback_fire_s'):
        circular[key] = _number(circular[key], 'tuning.circular.' + key, minimum=0)
    circular['max_angular_rps'] = _number(circular['max_angular_rps'], 'tuning.circular.max_angular_rps', positive=True)
    for key in config['timing']:
        config['timing'][key] = _number(config['timing'][key], 'tuning.timing.' + key, minimum=0)
    return config
