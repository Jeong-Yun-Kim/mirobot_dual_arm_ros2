"""Validate configured and detected targets without clamping or changing them."""

import math

import yaml

BOUND_NAMES = [f'workspace_{axis}_{edge}' for axis in 'xyz' for edge in ('min', 'max')]
DEFAULT_BOUNDS = [140.0, 290.0, -270.0, 270.0, 40.0, 300.0]
TARGET_NAMES = [
    'master_put_l',
    'master_put_r',
    'master_wait',
    'master_home',
    'slave_lid',
    'slave_box',
    'slave_conveyor',
    'slave_home',
]


def validate_bounds(bounds):
    if len(bounds) != 6 or not all(
        isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
        for x in bounds
    ):
        raise ValueError('workspace bounds require six finite numbers')
    for i, axis in enumerate('xyz'):
        if bounds[2 * i] > bounds[2 * i + 1]:
            raise ValueError(f'workspace_{axis}_min exceeds max')


def validate_xyz(name, xyz, bounds):
    validate_bounds(bounds)
    if not isinstance(xyz, (tuple, list)) or len(xyz) != 3:
        raise ValueError(f'{name}: expected exactly three XYZ values, got {xyz!r}')
    for i, value in enumerate(xyz):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ValueError(f'{name}: {"XYZ"[i]} must be finite, got {value!r}')
        low, high = bounds[2 * i: 2 * i + 2]
        if not low <= value <= high:
            raise ValueError(f'{name}: {"XYZ"[i]}={value} outside [{low}, {high}]')


def validate_targets(settings, bounds):
    validate_bounds(bounds)
    for name in TARGET_NAMES:
        target = settings[name]
        validate_xyz(name, target, bounds)
        validate_xyz(name + '.travel', [target[0], target[1], settings['z_travel']], bounds)
    # Pick height applies to camera coordinates, which are checked before each motion.
    validate_xyz('z_pick', [bounds[0], bounds[2], settings['z_pick']], bounds)


def validate_launch_settings(path):
    with open(path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    shared = config.get('/**', {}).get('ros__parameters', {})
    bounds = [
        shared.get(name, default) for name, default in zip(BOUND_NAMES, DEFAULT_BOUNDS)
    ]
    # Node-specific limits must not silently differ from the sequencer's shared limits.
    for section, values in config.items():
        params = values.get('ros__parameters', {})
        for name, value in zip(BOUND_NAMES, bounds):
            if name in params and params[name] != value:
                raise ValueError(f'{section}.{name}: differs from shared driver workspace')
    settings = dict(shared)
    settings.update(config.get('order_yolo_sequencer', {}).get('ros__parameters', {}))
    validate_targets(settings, bounds)
