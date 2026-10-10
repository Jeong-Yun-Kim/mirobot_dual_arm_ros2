#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import time
from typing import List, Optional

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .sequence_support import MotionCheck
from .sequence_support import SequenceLifecycle


def _now() -> float:
    return time.monotonic()


class StopDetector:
    """
    joint_states 기반 정지 판정기.

    - position이 들어올 때마다 속도(|dq|/dt)와 변위(|dq|)를 계산
    - 속도와 변위가 임계값 아래로 '연속' settle_time 동안 유지되면 stopped=True
    """

    def __init__(self, vel_eps: float, delta_eps: float, settle_time: float):
        self.vel_eps = float(vel_eps)  # rad/s
        self.delta_eps = float(delta_eps)  # rad
        self.settle_time = float(settle_time)

        self._last_pos: Optional[List[float]] = None
        self._last_t: Optional[float] = None

        self._still_since: Optional[float] = None
        self._moving_seen: bool = False

        self.last_msg_time: Optional[float] = None

        # 디버깅용
        self.last_speed = 0.0
        self.last_delta = 0.0

    def reset_for_new_goal(self):
        self._still_since = None
        self._moving_seen = False
        # last_pos/last_t는 유지(센서 continuity)해도 되고, 리셋해도 되지만
        # 리셋하면 첫 샘플에서 속도 계산이 안 되므로 유지하는 게 보통 더 안정적임.

    def update(self, msg: JointState):
        t = _now()
        self.last_msg_time = t

        pos = list(msg.position) if msg.position is not None else []
        if len(pos) == 0:
            return

        if (
            self._last_pos is None
            or self._last_t is None
            or len(self._last_pos) != len(pos)
        ):
            self._last_pos = pos
            self._last_t = t
            self._still_since = None
            self.last_speed = 0.0
            self.last_delta = 0.0
            return

        dt = max(1e-3, (t - self._last_t))
        # L2 norm of delta
        delta_sq = 0.0
        for a, b in zip(pos, self._last_pos):
            d = a - b
            delta_sq += d * d
        delta = delta_sq**0.5
        speed = delta / dt

        self.last_delta = float(delta)
        self.last_speed = float(speed)

        # moving/still 판단
        moving = (speed > self.vel_eps) or (delta > self.delta_eps)

        if moving:
            self._moving_seen = True
            self._still_since = None
        else:
            if self._still_since is None:
                self._still_since = t

        self._last_pos = pos
        self._last_t = t

    def moving_seen(self) -> bool:
        return self._moving_seen

    def stopped_stable(self) -> bool:
        if self._still_since is None:
            return False
        return (_now() - self._still_since) >= self.settle_time


class MirobotSequential(SequenceLifecycle, Node):
    """
    목표가 /mirobot1/target_pose_xyz 로 들어오면, 1) 마스터가 목표로 이동 완료(정지 판정) -> 슬레이브에 동일 목표 publish.

      2) 슬레이브 이동 완료(정지 판정) ->
         - enable_round_trip=True이면: 슬레이브를 home으로, 그 다음 마스터를 home으로
         - 아니면 종료(IDLE)

    정지 판정은:
      - is_idle 토픽이 정상적으로 변하면(is_idle 사용)
      - joint_states는 보조 관측만 하며 busy/idle 완료 확인을 대체하지 않는다.
    """

    def __init__(self):
        super().__init__('mirobot_status_joint_sequential_replicator')

        # ---------------- Params ----------------
        self.declare_parameter('master_pose_topic', '/mirobot1/target_pose_xyz')
        self.declare_parameter('slave_pose_topic', '/mirobot2/target_pose_xyz')

        self.declare_parameter('master_busy_topic', '/mirobot1/is_busy')
        self.declare_parameter('slave_busy_topic', '/mirobot2/is_busy')

        self.declare_parameter('master_joint_topic', '/mirobot1/joint_states')
        self.declare_parameter('slave_joint_topic', '/mirobot2/joint_states')

        # 멈춤판정 튜닝
        self.declare_parameter('settle_time', 1.0)  # 연속 안정 시간(초)
        self.declare_parameter('vel_eps', 0.03)  # rad/s
        self.declare_parameter('delta_eps', 0.003)  # rad (샘플 간 변위)
        self.declare_parameter(
            'joint_timeout', 1.0
        )  # joint_states가 이 시간 이상 안 오면 불신

        # 타이밍/안전장치
        self.declare_parameter('poll_hz', 30.0)
        self.declare_parameter('timeout', 45.0)  # 단계별 timeout
        self.declare_parameter('min_wait', 0.20)  # cmd 후 최소 대기
        self.declare_parameter('status_timeout', 1.0)

        # 왕복 동작
        self.declare_parameter('enable_round_trip', True)
        self.declare_parameter('home_x', 200.0)
        self.declare_parameter('home_y', 140.0)
        self.declare_parameter('home_z', 100.0)

        # ---------------- Read ----------------
        self.master_pose_topic = self.get_parameter('master_pose_topic').value
        self.slave_pose_topic = self.get_parameter('slave_pose_topic').value

        self.master_busy_topic = self.get_parameter('master_busy_topic').value
        self.slave_busy_topic = self.get_parameter('slave_busy_topic').value

        self.master_joint_topic = self.get_parameter('master_joint_topic').value
        self.slave_joint_topic = self.get_parameter('slave_joint_topic').value

        self.settle_time = float(self.get_parameter('settle_time').value)
        self.vel_eps = float(self.get_parameter('vel_eps').value)
        self.delta_eps = float(self.get_parameter('delta_eps').value)
        self.joint_timeout = float(self.get_parameter('joint_timeout').value)

        self.poll_hz = float(self.get_parameter('poll_hz').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.min_wait = float(self.get_parameter('min_wait').value)
        self.status_timeout = float(self.get_parameter('status_timeout').value)

        self.enable_round_trip = bool(self.get_parameter('enable_round_trip').value)

        self.home_pose = Pose()
        self.home_pose.position.x = float(self.get_parameter('home_x').value)
        self.home_pose.position.y = float(self.get_parameter('home_y').value)
        self.home_pose.position.z = float(self.get_parameter('home_z').value)

        # ---------------- QoS ----------------
        # Pose command는 보통 RELIABLE이 안전
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # joint_states는 sensor_data(best effort)로 받는 게 호환성 좋음
        qos_js = qos_profile_sensor_data

        # ---------------- Pub/Sub ----------------
        self.sub_master_cmd = self.create_subscription(
            Pose, self.master_pose_topic, self.cb_master_cmd, qos_cmd
        )

        self.sub_master_idle = self.create_subscription(
            Bool, self.master_busy_topic, self.cb_master_idle, 10
        )
        self.sub_slave_idle = self.create_subscription(
            Bool, self.slave_busy_topic, self.cb_slave_idle, 10
        )

        self.sub_master_js = self.create_subscription(
            JointState, self.master_joint_topic, self.cb_master_js, qos_js
        )
        self.sub_slave_js = self.create_subscription(
            JointState, self.slave_joint_topic, self.cb_slave_js, qos_js
        )

        self.pub_slave_cmd = self.create_publisher(Pose, self.slave_pose_topic, qos_cmd)
        self.pub_master_cmd = self.create_publisher(Pose, self.master_pose_topic, qos_cmd)

        # ---------------- State ----------------
        self.state = 'IDLE'
        self.cmd_time: Optional[float] = None
        self.target_pose: Optional[Pose] = None

        self.master_idle: Optional[bool] = None
        self.slave_idle: Optional[bool] = None

        self.master_idle_since: Optional[float] = None
        self.slave_idle_since: Optional[float] = None

        self.seen_master_busy = False
        self.seen_slave_busy = False

        self.master_stop = StopDetector(self.vel_eps, self.delta_eps, self.settle_time)
        self.slave_stop = StopDetector(self.vel_eps, self.delta_eps, self.settle_time)

        self.setup_sequence()
        self.motion = MotionCheck(self.timeout, self.status_timeout, self.settle_time)

        self.timer = self.create_timer(1.0 / self.poll_hz, self.tick)

        self.get_logger().info(
            'SequentialReplicator (is_idle + joint_states) started\n'
            f' master_pose_topic={self.master_pose_topic}\n'
            f' slave_pose_topic={self.slave_pose_topic}\n'
            f' master_busy_topic={self.master_busy_topic}\n'
            f' slave_busy_topic={self.slave_busy_topic}\n'
            f' master_joint_topic={self.master_joint_topic}\n'
            f' slave_joint_topic={self.slave_joint_topic}\n'
            f' settle_time={self.settle_time:.2f}s, vel_eps={self.vel_eps:.3f}, '
            f'delta_eps={self.delta_eps:.4f}\n'
            f' joint_timeout={self.joint_timeout:.2f}s, '
            f'status_timeout={self.status_timeout:.2f}s\n'
            f' enable_round_trip={self.enable_round_trip}, '
            f'home=({self.home_pose.position.x:.1f},{self.home_pose.position.y:.1f},'
            f'{self.home_pose.position.z:.1f})'
        )

    # ---------------- Callbacks ----------------
    def cb_master_cmd(self, msg: Pose):
        if self.state not in ('IDLE', 'FAILED'):
            return  # Ignore own return-home publication and overlapping commands.
        if not self.begin_sequence():
            return
        self.motion.reset()
        self.target_pose = copy.deepcopy(msg)
        self.cmd_time = _now()
        self.state = 'WAIT_MASTER_DONE'

        # 새 목표 시작: busy/idle/stop 판정 리셋
        self.seen_master_busy = False
        self.master_idle_since = None
        self.master_stop.reset_for_new_goal()

        self.seen_slave_busy = False
        self.slave_idle_since = None
        self.slave_stop.reset_for_new_goal()

        self.get_logger().info(
            f'[MASTER CMD] X={msg.position.x:.1f}, Y={msg.position.y:.1f}, Z={msg.position.z:.1f}'
        )

    def cb_master_idle(self, msg: Bool):
        t = _now()
        self.master_idle = not bool(msg.data)
        if hasattr(self, 'motion') and 'MASTER' in self.state:
            self.motion.update(msg.data)

        if self.master_idle is False:
            self.seen_master_busy = True
            self.master_idle_since = None
        else:
            if self.master_idle_since is None:
                self.master_idle_since = t

    def cb_slave_idle(self, msg: Bool):
        t = _now()
        self.slave_idle = not bool(msg.data)
        if hasattr(self, 'motion') and 'SLAVE' in self.state:
            self.motion.update(msg.data)

        if self.slave_idle is False:
            self.seen_slave_busy = True
            self.slave_idle_since = None
        else:
            if self.slave_idle_since is None:
                self.slave_idle_since = t

    def cb_master_js(self, msg: JointState):
        self.master_stop.update(msg)
        if self.master_stop.moving_seen():
            self.seen_master_busy = True

    def cb_slave_js(self, msg: JointState):
        self.slave_stop.update(msg)
        if self.slave_stop.moving_seen():
            self.seen_slave_busy = True

    def _reset_idle(self):
        self.state = 'IDLE'
        self.cmd_time = None
        self.target_pose = None

    def abort_sequence(self, reason):
        self.state = 'FAILED'
        self.cmd_time = None
        self.get_logger().error(reason)
        self.finish_sequence(False, reason)

    def _next_motion(self, state, publisher, pose):
        self.require_running()
        self.state = state
        self.cmd_time = _now()
        self.motion.reset()
        self.master_stop.reset_for_new_goal()
        self.slave_stop.reset_for_new_goal()
        publisher.publish(pose)

    def tick(self):
        if not self.sequence_running:
            return
        try:
            self.require_running()
            done = self.motion.check()
            if _now() - self.cmd_time < self.min_wait or not done:
                return
            if self.state == 'WAIT_MASTER_DONE':
                self._next_motion('WAIT_SLAVE_DONE', self.pub_slave_cmd, self.target_pose)
            elif self.state == 'WAIT_SLAVE_DONE':
                if self.enable_round_trip:
                    self._next_motion(
                        'WAIT_SLAVE_HOME_DONE', self.pub_slave_cmd, self.home_pose
                    )
                else:
                    self._reset_idle()
                    self.finish_sequence(True)
            elif self.state == 'WAIT_SLAVE_HOME_DONE':
                self._next_motion(
                    'WAIT_MASTER_HOME_DONE', self.pub_master_cmd, self.home_pose
                )
            elif self.state == 'WAIT_MASTER_HOME_DONE':
                self._reset_idle()
                self.finish_sequence(True)
        except (TimeoutError, RuntimeError) as exc:
            self.abort_sequence(str(exc))


def main():
    rclpy.init()
    node = MirobotSequential()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
