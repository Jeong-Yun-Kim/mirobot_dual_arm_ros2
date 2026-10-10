"""Exercise failure paths with fake publishers and no ROS or device initialization."""

import json
from pathlib import Path
from unittest.mock import Mock

from mirobot_master_slave.conveyor_controller_node import ConveyorController
from mirobot_master_slave.coordinates import DEFAULT_BOUNDS
from mirobot_master_slave.coordinates import TARGET_NAMES
from mirobot_master_slave.coordinates import validate_launch_settings
from mirobot_master_slave.coordinates import validate_targets
from mirobot_master_slave.mirobot_roundtrip_sequencer import RoundTripSequencer
from mirobot_master_slave.mirobot_roundtrip_sequencer import State
from mirobot_master_slave.mirobot_sequential_replicator import MirobotSequential
from mirobot_master_slave.order_yolo_sequencer import OrderYoloSequencer
from mirobot_master_slave.sequence_support import MotionCheck
from mirobot_master_slave.server_bridge import MrOrderBridge
import pytest
from std_msgs.msg import Bool
from std_msgs.msg import String


@pytest.fixture
def clock(monkeypatch):
    current = [100.0]
    monkeypatch.setattr('time.monotonic', lambda: current[0])
    monkeypatch.setattr(
        'time.sleep', lambda delay: current.__setitem__(0, current[0] + max(delay, 0.01))
    )
    return current


def node_without_ros(cls):
    node = cls.__new__(cls)
    node.get_logger = Mock(return_value=Mock())
    node.create_publisher = Mock(side_effect=lambda *args: Mock())
    node.create_subscription = Mock()
    node.setup_sequence()
    node.begin_sequence()
    return node


@pytest.mark.parametrize(
    'cls,state',
    [
        (RoundTripSequencer, value)
        for value in State
        if value not in (State.IDLE, State.FAILED)
    ]
    + [
        (MirobotSequential, value)
        for value in (
            'WAIT_MASTER_DONE',
            'WAIT_SLAVE_DONE',
            'WAIT_SLAVE_HOME_DONE',
            'WAIT_MASTER_HOME_DONE',
        )
    ],
)
@pytest.mark.parametrize('failure', ['timeout', 'missing', 'stale', 'idle_only'])
def test_failed_phase_never_publishes_next_motion(cls, state, failure, clock):
    # Set up without calling either hardware-facing or rclpy constructors.
    node = cls.__new__(cls)
    node.master_pose_topic = '/mirobot1/target_pose_xyz'
    node.get_logger = Mock(return_value=Mock())
    node.create_publisher = Mock(side_effect=lambda *args: Mock())
    node.create_subscription = Mock()
    node.setup_sequence()
    node.begin_sequence()
    node.state = state
    node.motion = MotionCheck(2.0, 1.0, 0.1)
    node.cmd_time = clock[0]
    node.min_wait = 0.0
    for name in (
        'pub_slave_cmd',
        'pub_master_cmd',
        'pub_slave_pose',
        'pub_slave_home',
        'pub_master_home',
    ):
        setattr(node, name, Mock())
    if failure == 'stale':
        node.motion.update(True)
        node.motion.update(False)
    if failure in ('timeout', 'idle_only'):
        clock[0] += 3
        node.motion.update(failure == 'timeout')
    else:
        clock[0] += 1.1
    node.tick()
    assert not node.sequence_running
    result = json.loads(node.pub_seq_result.publish.call_args.args[0].data)
    assert result['ok'] is False and result['state'] == 'failed'
    for name in (
        'pub_slave_cmd',
        'pub_master_cmd',
        'pub_slave_pose',
        'pub_slave_home',
        'pub_master_home',
    ):
        getattr(node, name).publish.assert_not_called()


def test_each_roundtrip_phase_needs_a_new_busy_idle_pair(clock):
    node = RoundTripSequencer.__new__(RoundTripSequencer)
    node.master_pose_topic = '/mirobot1/target_pose_xyz'
    node.get_logger = Mock(return_value=Mock())
    node.create_publisher = Mock(side_effect=lambda *args: Mock())
    node.create_subscription = Mock()
    node.setup_sequence()
    node.begin_sequence()
    node.state = State.WAIT_MASTER_BUSY
    node.motion = MotionCheck(10, 1, 0.1)
    node._publish_slave_pose = Mock()
    node._publish_slave_home = Mock()
    node._publish_master_home = Mock()
    for expected in (
        State.WAIT_SLAVE_DONE,
        State.WAIT_SLAVE_HOME_DONE,
        State.WAIT_MASTER_HOME_DONE,
        State.IDLE,
    ):
        node.motion.update(False)
        node.tick()
        assert node.sequence_running
        node.motion.update(True)
        node.motion.update(False)
        clock[0] += 0.2
        node.tick()
        assert node.state == expected
    assert json.loads(node.pub_seq_result.publish.call_args.args[0].data)['ok'] is True
    node._publish_slave_pose.assert_called_once()
    node._publish_slave_home.assert_called_once()
    node._publish_master_home.assert_called_once()


def bridge():
    node = MrOrderBridge.__new__(MrOrderBridge)
    node.current_id = 7
    node.request_id = 'request-7'
    node.sequence_id = None
    node.estop = False
    node.active = True
    node.pub_done = Mock()
    node.pub_failed = Mock()
    node._raw = Mock()
    node._trigger_master = Mock()
    node.get_logger = Mock(return_value=Mock())
    return node


def result(state, ok, request='request-7'):
    return String(
        data=json.dumps(
            {
                'request_id': request,
                'sequence_id': 'token',
                'state': state,
                'ok': ok,
                'reason': 'test failure',
            }
        )
    )


def test_activity_falling_or_failure_does_not_mean_success():
    node = bridge()
    node.on_result(result('started', None))
    node.on_seq(Bool(data=False))
    node.pub_done.publish.assert_not_called()
    node.on_result(result('failed', False))
    node.pub_done.publish.assert_not_called()
    assert json.loads(node.pub_failed.publish.call_args.args[0].data)['id'] == 7
    assert node.current_id is None


def test_only_matching_explicit_success_preserves_legacy_done_shape():
    node = bridge()
    node.on_result(result('started', None, 'old-request'))
    node.on_result(result('succeeded', True, 'old-request'))
    node.pub_done.publish.assert_not_called()
    node.on_result(result('started', None))
    node.on_result(result('succeeded', True))
    assert json.loads(node.pub_done.publish.call_args.args[0].data) == {'id': 7}


@pytest.mark.parametrize('command', ['', ' ', 'STATR', 'RESUME', 'unknown'])
@pytest.mark.parametrize('stopped', [True, False])
def test_unknown_stop_command_changes_nothing(command, stopped):
    node = bridge()
    node.estop = stopped
    node.on_estop(String(data=command))
    assert node.estop == stopped and node.current_id == 7
    node._raw.assert_not_called()
    node._trigger_master.assert_not_called()


def test_stop_then_start_never_replays_order_or_reports_success():
    node = bridge()
    node.on_result(result('started', None))
    node.on_estop(String(data='ESTOP'))
    node.on_estop(String(data='START'))
    node.on_result(result('succeeded', True))
    assert [c.args[0] for c in node._raw.call_args_list] == ['!', '~']
    node._trigger_master.assert_not_called()
    node.pub_done.publish.assert_not_called()


def test_stop_cancels_motion_gate_even_after_start(clock):
    node = node_without_ros(OrderYoloSequencer)
    node.current_order = {'id': 7}
    node.pub_failed = Mock()
    node.pub_master_raw = Mock()
    node.pub_slave_raw = Mock()
    node.pub_master = Mock()
    node.motion = {'master': MotionCheck(2, 1, 0)}
    node.on_sequence_stop(String(data='ESTOP'))
    node.on_sequence_stop(String(data='START'))
    with pytest.raises(RuntimeError):
        node.move_master(200, 0, 100)
    node.pub_master.publish.assert_not_called()


@pytest.mark.parametrize('robot', ['master', 'slave'])
def test_order_wait_timeout_raises_instead_of_returning_success(robot, clock):
    node = node_without_ros(OrderYoloSequencer)
    node.motion = {robot: MotionCheck(2, 1, 0)}
    with pytest.raises(TimeoutError, match='status missing'):
        getattr(node, 'wait_' + robot)()


def valid_settings():
    return {
        **{name: [200.0, 0.0, 100.0] for name in TARGET_NAMES},
        'z_pick': 55.0,
        'z_travel': 120.0,
    }


@pytest.mark.parametrize(
    'xyz',
    [
        [-20, 0, 60],
        [230, 100, 20],
        [200, 0],
        [float('nan'), 0, 60],
        [200, float('inf'), 60],
    ],
)
def test_invalid_coordinates_are_rejected(xyz):
    settings = valid_settings()
    settings['master_put_l'] = xyz
    with pytest.raises(ValueError, match='master_put_l'):
        validate_targets(settings, DEFAULT_BOUNDS)


def test_measured_in_range_configuration_passes_and_original_example_fails():
    validate_targets(valid_settings(), DEFAULT_BOUNDS)
    path = Path(__file__).resolve().parents[1] / 'config/dual_arm.yaml'
    with pytest.raises(ValueError, match='master_put_l'):
        validate_launch_settings(path)


def test_dynamic_target_is_checked_before_publish():
    node = node_without_ros(OrderYoloSequencer)
    node.bounds = DEFAULT_BOUNDS
    node.pub_master = Mock()
    with pytest.raises(ValueError, match='master target'):
        node.move_master(-20, 0, 60)
    node.pub_master.publish.assert_not_called()


def test_conveyor_failure_does_not_restart_it():
    node = ConveyorController.__new__(ConveyorController)
    node.sequence_id = 'token'
    node.seq_active = True
    node.send = Mock()
    node.cb_seq(Bool(data=False))
    node.send.assert_not_called()
    node.cb_result(result('failed', False))
    node.send.assert_called_once_with('STOP')


def test_driver_workspace_mismatch_stops_order_before_motion(clock):
    from rclpy.parameter import Parameter

    node = node_without_ros(OrderYoloSequencer)
    node.bounds = DEFAULT_BOUNDS
    response = Mock(values=[Parameter('bound', value=float(v)).get_parameter_value()
                            for v in [0, 290, -270, 270, 40, 300]])
    future = Mock(done=Mock(return_value=True), result=Mock(return_value=response))
    client = Mock(wait_for_service=Mock(return_value=True), call_async=Mock(return_value=future))
    node.driver_clients = [client]
    with pytest.raises(ValueError, match='differs from sequence'):
        node.check_driver_bounds()


def test_order_worker_reports_wait_failure_without_followup(clock):
    node = node_without_ros(OrderYoloSequencer)
    node.current_order = {'id': 7, 'red': 1}
    node.settings = valid_settings()
    node.bounds = DEFAULT_BOUNDS
    node.check_driver_bounds = Mock()
    node.wait_cube = Mock(return_value={'x': 200.0, 'y': 0.0})
    node.motion = {'master': MotionCheck(2, 1, 0), 'slave': MotionCheck(2, 1, 0)}
    node.pub_master = Mock()
    node.pub_slave = Mock()
    node.pub_master_raw = Mock()
    node.pub_done = Mock()
    node.pub_failed = Mock()
    node.run_sequence()
    assert node.pub_master.publish.call_count == 1
    node.pub_slave.publish.assert_not_called()
    node.pub_master_raw.publish.assert_not_called()
    node.pub_done.publish.assert_not_called()
    assert json.loads(node.pub_failed.publish.call_args.args[0].data)['ok'] is False


def test_request_and_success_are_correlated_end_to_end(clock):
    node = RoundTripSequencer.__new__(RoundTripSequencer)
    node.master_pose_topic = '/mirobot1/target_pose_xyz'
    node.get_logger = Mock(return_value=Mock())
    node.create_publisher = Mock(side_effect=lambda *args: Mock())
    node.create_subscription = Mock()
    node.setup_sequence()
    node.state = State.IDLE
    node.motion = MotionCheck(10, 1, 0.1)
    node._publish_slave_pose = Mock()
    node._publish_slave_home = Mock()
    node._publish_master_home = Mock()
    server = bridge()
    node.pub_seq_result.publish.side_effect = server.on_result
    node.on_sequence_request(String(data=json.dumps({
        'request_id': 'request-7', 'xyz': [200.0, 0.0, 120.0]})))
    node.pub_requested_pose.publish.assert_called_once()
    for _ in range(4):
        node.motion.update(True)
        node.motion.update(False)
        clock[0] += 0.2
        node.tick()
    assert json.loads(server.pub_done.publish.call_args.args[0].data) == {'id': 7}
    server.pub_failed.publish.assert_not_called()


def test_vision_launch_checks_settings_before_any_node(tmp_path, monkeypatch):
    import importlib.util

    from launch import LaunchContext
    from launch.actions import OpaqueFunction
    from launch_ros.actions import Node
    import yaml

    package = Path(__file__).resolve().parents[1]
    path = package / 'launch/mirobot_master_slave.launch.py'
    spec = importlib.util.spec_from_file_location('checked_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    control = tmp_path / 'mirobot_description'
    (control / 'urdf').mkdir(parents=True)
    (control / 'urdf/mirobot_urdf_2.urdf').write_text('<robot name="test"/>')
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(control if name == 'mirobot_description' else package))
    actions = module.generate_launch_description().entities
    guard = next(i for i, action in enumerate(actions) if isinstance(action, OpaqueFunction))
    assert all(i > guard for i, action in enumerate(actions) if isinstance(action, Node))
    context = LaunchContext()
    context.launch_configurations.update(
        mode='vision', params=str(package / 'config/dual_arm.yaml'))
    with pytest.raises(ValueError, match='master_put_l'):
        actions[guard].execute(context)
    config = yaml.safe_load((package / 'config/dual_arm.yaml').read_text())
    config['order_yolo_sequencer']['ros__parameters'].update(valid_settings())
    measured = tmp_path / 'synthetic-in-range.yaml'
    measured.write_text(yaml.safe_dump(config))
    context.launch_configurations['params'] = str(measured)
    assert actions[guard].execute(context) == []
