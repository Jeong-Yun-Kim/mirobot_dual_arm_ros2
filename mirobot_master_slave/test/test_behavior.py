from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Pose
from mirobot_master_slave.mirobot_roundtrip_sequencer import RoundTripSequencer
from mirobot_master_slave.mirobot_roundtrip_sequencer import State
from mirobot_master_slave.mirobot_sequential_replicator import MirobotSequential
from mirobot_master_slave.mirobot_slave_replicator import MirobotSlaveReplicator
from std_msgs.msg import Bool


def test_busy_is_inverted_and_own_return_command_is_ignored():
    state = SimpleNamespace(master_idle_since=None, seen_master_busy=False)
    MirobotSequential.cb_master_idle(state, Bool(data=True))
    assert state.master_idle is False and state.seen_master_busy
    MirobotSequential.cb_master_idle(state, Bool(data=False))
    assert state.master_idle is True and state.master_idle_since is not None
    state.state = 'WAIT_MASTER_HOME_DONE'
    MirobotSequential.cb_master_cmd(state, Pose())
    assert state.state == 'WAIT_MASTER_HOME_DONE'


def test_replication_and_roundtrip_transition_without_ros_publish():
    target = Pose()
    target.position.x = 200.0
    state = SimpleNamespace(pub=Mock(), get_logger=lambda: Mock())
    MirobotSlaveReplicator.cb(state, target)
    state.pub.publish.assert_called_once_with(target)
    state = SimpleNamespace(
        state=State.WAIT_MASTER_DONE,
        timeout_sec=45,
        sequence_running=True,
        require_running=Mock(),
        motion=Mock(check=Mock(return_value=True)),
        _elapsed=lambda: 1.0,
        _saw_master_busy_edge=True,
        master_busy=False,
        settle_time=0.0,
        _publish_slave_pose=Mock(),
        _enter=Mock(),
    )
    RoundTripSequencer.tick(state)
    state._publish_slave_pose.assert_called_once()
    state._enter.assert_called_once_with(State.WAIT_SLAVE_DONE)
