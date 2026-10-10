#!/usr/bin/env python3
import json
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class MrOrderBridge(Node):

    def __init__(self):
        super().__init__('mr_order_bridge')

        self.create_subscription(String, '/mr_move', self.on_move, 10)
        self.create_subscription(String, '/mr_estop', self.on_estop, 10)
        self.create_subscription(Bool, '/sequence_active', self.on_seq, 10)
        self.create_subscription(String, '/sequence_result', self.on_result, 10)

        self.pub_request = self.create_publisher(String, '/sequence_request', 10)
        self.pub_done = self.create_publisher(String, '/mr_done', 10)
        self.pub_failed = self.create_publisher(String, '/mr_failed', 10)
        self.sequence_id = None
        self.request_id = None

        # serial_bridge.py에 추가한 토픽
        self.pub_raw_master = self.create_publisher(String, '/mirobot1/raw_cmd', 10)
        self.pub_raw_slave = self.create_publisher(String, '/mirobot2/raw_cmd', 10)

        self.declare_parameter('trigger_xyz', [200.0, 0.0, 120.0])
        self.current_id = None
        self.estop = False
        self.active = False

    def _raw(self, s: str):
        m = String()
        m.data = s
        self.pub_raw_master.publish(m)
        self.pub_raw_slave.publish(m)

    def _trigger_master(self):
        self.request_id = uuid.uuid4().hex
        self.pub_request.publish(
            String(
                data=json.dumps(
                    {
                        'request_id': self.request_id,
                        'xyz': list(self.get_parameter('trigger_xyz').value),
                    }
                )
            )
        )

    def fail_order(self, reason):
        if self.current_id is not None:
            self.pub_failed.publish(
                String(
                    data=json.dumps({'id': self.current_id, 'ok': False, 'reason': reason})
                )
            )
        self.current_id = None
        self.sequence_id = None

    def on_estop(self, msg: String):
        command = (msg.data or '').strip().upper()
        if command == 'ESTOP':
            self.estop = True
            self._raw('!')
            self.fail_order('ESTOP')
        elif command == 'START':
            self.estop = False
            self._raw('~')
            # A cancelled job requires a new order; START never repeats its pose.
        else:
            self.get_logger().warning(f'Ignoring unknown stop command: {command!r}')

    def on_move(self, msg: String):
        if self.estop or self.current_id is not None or self.active:
            return
        try:
            data = json.loads(msg.data)
            order_id = int(data['id'])
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning('Ignoring invalid /mr_move order')
            return
        self.current_id = order_id
        self.sequence_id = None
        self._trigger_master()

    def on_seq(self, msg: Bool):
        self.active = bool(msg.data)
        # Activity falling is not evidence that the job succeeded.

    def on_result(self, msg: String):
        try:
            result = json.loads(msg.data)
            token = result['sequence_id']
            state = result['state']
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning('Ignoring invalid sequence result')
            return
        if (
            self.current_id is None
            or self.estop
            or result.get('request_id') != self.request_id
        ):
            return
        if state == 'started' and self.sequence_id is None:
            self.sequence_id = token
        elif token and token == self.sequence_id:
            if state == 'succeeded' and result.get('ok') is True:
                self.pub_done.publish(String(data=json.dumps({'id': self.current_id})))
                self.current_id = None
                self.sequence_id = None
            elif state == 'failed' and result.get('ok') is False:
                self.fail_order(result.get('reason', 'sequence failed'))


def main():
    rclpy.init()
    node = MrOrderBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
