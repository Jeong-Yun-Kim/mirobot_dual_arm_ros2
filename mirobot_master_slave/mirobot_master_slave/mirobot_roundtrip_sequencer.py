#!/usr/bin/env python3
from enum import Enum

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .sequence_support import MotionCheck
from .sequence_support import SequenceLifecycle


class State(Enum):
    IDLE = 0
    WAIT_MASTER_BUSY = 1
    WAIT_MASTER_DONE = 2
    WAIT_SLAVE_DONE = 3
    WAIT_SLAVE_HOME_DONE = 4
    WAIT_MASTER_HOME_DONE = 5
    FAILED = 6


class RoundTripSequencer(SequenceLifecycle, Node):
    """
    Run master, slave, slave-home, then master-home after confirmed completion.

    (1) 마스터 목표 Pose를 감지(/mirobot1/target_pose_xyz)
      (2) 마스터가 Busy->Idle 될 때까지 기다림
      (3) 슬레이브에 같은 Pose publish (/mirobot2/target_pose_xyz)
      (4) 슬레이브 Busy->Idle 기다림
      (5) 슬레이브 원위치(조인트 0) publish (/mirobot2/target_joint_states)
      (6) 슬레이브 Busy->Idle 기다림
      (7) 마스터 원위치(조인트 0) publish (/mirobot1/target_joint_states)
      (8) 마스터 Busy->Idle 기다림
    + (추가) /sequence_active (Bool) publish:
        시퀀스 진행 중 True, IDLE 복귀 시 False
    """

    def __init__(self):
        super().__init__('mirobot_roundtrip_sequencer')

        self.declare_parameter('master_pose_topic', '/mirobot1/target_pose_xyz')
        self.declare_parameter('slave_pose_topic', '/mirobot2/target_pose_xyz')
        self.declare_parameter('master_busy_topic', '/mirobot1/is_busy')
        self.declare_parameter('slave_busy_topic', '/mirobot2/is_busy')
        self.declare_parameter('master_home_topic', '/mirobot1/target_joint_states')
        self.declare_parameter('slave_home_topic', '/mirobot2/target_joint_states')

        self.declare_parameter('sequence_active_topic', '/sequence_active')

        self.declare_parameter('home_joint_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter(
            'home_joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        )

        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('status_timeout', 1.0)
        self.declare_parameter('settle_time', 0.4)  # Idle 판단 후 안정화 대기

        self.master_pose_topic = self.get_parameter('master_pose_topic').value
        self.slave_pose_topic = self.get_parameter('slave_pose_topic').value
        self.master_busy_topic = self.get_parameter('master_busy_topic').value
        self.slave_busy_topic = self.get_parameter('slave_busy_topic').value
        self.master_home_topic = self.get_parameter('master_home_topic').value
        self.slave_home_topic = self.get_parameter('slave_home_topic').value
        self.sequence_active_topic = self.get_parameter('sequence_active_topic').value

        self.home_pos = list(self.get_parameter('home_joint_positions').value)
        self.home_names = list(self.get_parameter('home_joint_names').value)

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.settle_time = float(self.get_parameter('settle_time').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # pubs
        self.pub_slave_pose = self.create_publisher(Pose, self.slave_pose_topic, qos)
        self.pub_slave_home = self.create_publisher(JointState, self.slave_home_topic, qos)
        self.pub_master_home = self.create_publisher(
            JointState, self.master_home_topic, qos
        )
        self.setup_sequence()
        self.motion = MotionCheck(
            self.timeout_sec, self.get_parameter('status_timeout').value, self.settle_time
        )

        # subs
        self.create_subscription(Pose, self.master_pose_topic, self.on_master_cmd, qos)
        self.create_subscription(Bool, self.master_busy_topic, self.on_master_busy, qos)
        self.create_subscription(Bool, self.slave_busy_topic, self.on_slave_busy, qos)

        self.state = State.IDLE
        self.master_busy = False
        self.slave_busy = False

        self._target_pose: Pose = None

        self.create_timer(0.05, self.tick)

        self.get_logger().info(
            'RoundTripSequencer started\n'
            f' master_pose_topic={self.master_pose_topic}\n'
            f' slave_pose_topic={self.slave_pose_topic}\n'
            f' master_busy_topic={self.master_busy_topic}\n'
            f' slave_busy_topic={self.slave_busy_topic}\n'
            f' slave_home_topic={self.slave_home_topic}\n'
            f' master_home_topic={self.master_home_topic}\n'
            f' sequence_active_topic={self.sequence_active_topic}\n'
            f' timeout={self.timeout_sec}s, settle_time={self.settle_time}s'
        )

    def _enter(self, st: State):
        self.state = st
        self.motion.reset()

    def on_master_cmd(self, msg: Pose):
        if not self.begin_sequence():
            return
        self._target_pose = msg
        self._enter(State.WAIT_MASTER_BUSY)

    def on_master_busy(self, msg: Bool):
        self.master_busy = bool(msg.data)
        if self.state in (
            State.WAIT_MASTER_BUSY,
            State.WAIT_MASTER_DONE,
            State.WAIT_MASTER_HOME_DONE,
        ):
            self.motion.update(msg.data)

    def on_slave_busy(self, msg: Bool):
        self.slave_busy = bool(msg.data)
        if self.state in (State.WAIT_SLAVE_DONE, State.WAIT_SLAVE_HOME_DONE):
            self.motion.update(msg.data)

    def _publish_slave_pose(self):
        if self._target_pose is None:
            return
        self.pub_slave_pose.publish(self._target_pose)
        self.get_logger().info('[SLAVE START] publish slave target_pose_xyz')

    def _publish_slave_home(self):
        js = JointState()
        js.name = self.home_names
        js.position = self.home_pos
        self.pub_slave_home.publish(js)
        self.get_logger().info('[SLAVE HOME] publish slave target_joint_states (home)')

    def _publish_master_home(self):
        js = JointState()
        js.name = self.home_names
        js.position = self.home_pos
        self.pub_master_home.publish(js)
        self.get_logger().info('[MASTER HOME] publish master target_joint_states (home)')

    def abort_sequence(self, reason):
        self.state = State.FAILED
        self.get_logger().error(reason)
        self.finish_sequence(False, reason)

    def tick(self):
        if not self.sequence_running:
            return
        try:
            self.require_running()
            if not self.motion.check():
                return
            if self.state in (State.WAIT_MASTER_BUSY, State.WAIT_MASTER_DONE):
                self._enter(State.WAIT_SLAVE_DONE)
                self._publish_slave_pose()
            elif self.state == State.WAIT_SLAVE_DONE:
                self._enter(State.WAIT_SLAVE_HOME_DONE)
                self._publish_slave_home()
            elif self.state == State.WAIT_SLAVE_HOME_DONE:
                self._enter(State.WAIT_MASTER_HOME_DONE)
                self._publish_master_home()
            elif self.state == State.WAIT_MASTER_HOME_DONE:
                self.state = State.IDLE
                self.finish_sequence(True)
        except (TimeoutError, RuntimeError) as exc:
            self.abort_sequence(str(exc))


def main():
    rclpy.init()
    node = RoundTripSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
