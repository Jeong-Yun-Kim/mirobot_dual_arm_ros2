"""Share fail-closed sequence results and measured busy/idle transitions."""

import json
import math
import threading
import time
import uuid

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from std_msgs.msg import String


class MotionCheck:
    """Require fresh busy then stable idle feedback for each individual command."""

    def __init__(self, timeout, freshness, settle):
        self.timeout = float(timeout)
        self.freshness = float(freshness)
        self.settle = float(settle)
        if (
            not all(math.isfinite(v) for v in (self.timeout, self.freshness, self.settle))
            or self.timeout <= 0
            or self.freshness <= 0
            or self.settle < 0
        ):
            raise ValueError(
                'timeouts must be positive and settle time nonnegative and finite'
            )
        self.reset()

    def reset(self):
        self.started = time.monotonic()
        self.received = None
        self.saw_busy = False
        self.idle_since = None

    def update(self, busy):
        now = time.monotonic()
        self.received = now
        if busy:
            self.saw_busy = True
            self.idle_since = None
        elif self.saw_busy and self.idle_since is None:
            self.idle_since = now

    def check(self):
        now = time.monotonic()
        if now - self.started > self.timeout:
            raise TimeoutError('motion completion timeout (busy then idle required)')
        reference = self.started if self.received is None else self.received
        if now - reference > self.freshness:
            raise TimeoutError('motion status missing or stale')
        return (
            self.saw_busy
            and self.idle_since is not None
            and now - self.idle_since >= self.settle
        )


class SequenceLifecycle:
    """Keep activity separate from a correlated success or failure result."""

    def setup_sequence(self):
        self.sequence_lock = threading.RLock()
        self.sequence_running = False
        self.sequence_stopped = False
        self.sequence_id = None
        self.request_id = None
        active_topic = getattr(self, 'sequence_active_topic', '/sequence_active')
        self.pub_seq_active = self.create_publisher(Bool, active_topic, 10)
        self.pub_seq_result = self.create_publisher(String, '/sequence_result', 10)
        self.create_subscription(String, '/mr_estop', self.on_sequence_stop, 10)
        if hasattr(self, 'on_master_cmd') or hasattr(self, 'cb_master_cmd'):
            self.pub_requested_pose = self.create_publisher(
                Pose, self.master_pose_topic, 10
            )
            self.create_subscription(
                String, '/sequence_request', self.on_sequence_request, 10
            )

    def on_sequence_request(self, msg):
        try:
            request = json.loads(msg.data)
            xyz = request['xyz']
            if (
                not isinstance(xyz, list)
                or len(xyz) != 3
                or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz)
            ):
                raise ValueError('request xyz must contain three finite numbers')
            token = request['request_id']
            if not isinstance(token, str) or not token:
                raise ValueError('request_id must be a nonempty string')
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warning(f'Ignoring invalid sequence request: {exc}')
            return
        if self.sequence_running or self.sequence_stopped:
            return
        self.request_id = token
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        pose.orientation.w = 1.0
        callback = getattr(self, 'on_master_cmd', None) or self.cb_master_cmd
        callback(pose)
        if self.sequence_running:
            self.pub_requested_pose.publish(pose)

    def begin_sequence(self):
        with self.sequence_lock:
            if self.sequence_stopped or self.sequence_running:
                return False
            self.sequence_id = uuid.uuid4().hex
            self.sequence_running = True
            self.publish_result('started', None, '')
            self.pub_seq_active.publish(Bool(data=True))
            return True

    def publish_result(self, state, ok, reason):
        self.pub_seq_result.publish(
            String(
                data=json.dumps(
                    {
                        'sequence_id': self.sequence_id,
                        'request_id': self.request_id,
                        'state': state,
                        'ok': ok,
                        'reason': reason,
                    }
                )
            )
        )

    def finish_sequence(self, ok, reason=''):
        with self.sequence_lock:
            if not self.sequence_running:
                return
            self.sequence_running = False
            self.publish_result('succeeded' if ok else 'failed', ok, reason)
            self.pub_seq_active.publish(Bool(data=False))

    def on_sequence_stop(self, msg):
        command = (msg.data or '').strip().upper()
        with self.sequence_lock:
            if command == 'ESTOP':
                self.sequence_stopped = True
                self.abort_sequence('ESTOP')
            elif command == 'START':
                self.sequence_stopped = False
            else:
                self.get_logger().warning(f'Ignoring unknown stop command: {command!r}')

    def require_running(self):
        if self.sequence_stopped or not self.sequence_running:
            raise RuntimeError('sequence stopped or already failed')
