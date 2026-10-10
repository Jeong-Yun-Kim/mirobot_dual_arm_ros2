#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Bool
from std_msgs.msg import String


class ConveyorController(Node):
    """
    ROS2 -> Arduino Serial bridge.

    Subscribe:
      /sequence_active (Bool): True when 전체 로봇 시퀀스 진행중
      /mirobot1/is_busy (Bool): True while master is moving (busy rising edge로 STOP)

    Serial to Arduino:
      "STOP"  -> 컨베이어 정지
      "START" -> 컨베이어 정방향 일정속도 구동
    """

    def __init__(self):
        super().__init__('conveyor_controller')

        # ---- params ----
        self.declare_parameter('port', '/dev/ttyACM0')  # 아두이노 포트
        self.declare_parameter('baud', 115200)
        self.declare_parameter('startup_state', 'START')  # START or STOP
        self.declare_parameter('debug_rx', True)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.debug_rx = bool(self.get_parameter('debug_rx').value)

        # ---- serial ----
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.2)
        time.sleep(1.5)  # Arduino auto-reset 대기
        self.get_logger().info(f'Connected Arduino: {port} @ {baud}')

        # ---- internal states ----
        self.sequence_id = None
        self.seq_active = False
        self.master_busy = False
        self.prev_master_busy = False

        # ---- init conveyor ----
        startup_state = str(self.get_parameter('startup_state').value).upper()
        if startup_state == 'STOP':
            self.send('STOP')
        else:
            self.send('START')

        # ---- subscribers ----
        self.create_subscription(Bool, '/sequence_active', self.cb_seq, 10)
        self.create_subscription(String, '/sequence_result', self.cb_result, 10)
        self.create_subscription(Bool, '/mirobot1/is_busy', self.cb_busy, 10)

        # ---- timer ----
        self.create_timer(0.2, self.timer_tick)  # RX 로그용 / 상태 유지용

    def send(self, cmd: str):
        cmd = cmd.strip().upper()
        if cmd not in ('START', 'STOP'):
            self.get_logger().warn(f'Unknown cmd: {cmd}')
            return
        msg = (cmd + '\n').encode('utf-8')
        self.ser.write(msg)
        self.ser.flush()
        self.get_logger().info(f'[TX->Arduino] {cmd}')

    def cb_seq(self, msg: Bool):
        self.seq_active = bool(msg.data)

    def cb_result(self, msg: String):
        try:
            result = json.loads(msg.data)
            token = result['sequence_id']
            if result['state'] == 'started':
                self.sequence_id = token
            elif token and token == self.sequence_id:
                if result.get('ok') is True and result['state'] == 'succeeded':
                    self.send('START')
                    self.sequence_id = None
                elif result.get('ok') is False and result['state'] == 'failed':
                    self.send('STOP')
                    self.sequence_id = None
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning('Ignoring invalid sequence result')

    def cb_busy(self, msg: Bool):
        self.prev_master_busy = self.master_busy
        self.master_busy = bool(msg.data)

        # 마스터가 움직이기 시작하는 순간(0->1) 컨베이어 정지
        if (not self.prev_master_busy) and self.master_busy:
            self.get_logger().info('[MASTER] busy rising -> STOP conveyor')
            self.send('STOP')

    def timer_tick(self):
        if not self.debug_rx:
            return
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                self.get_logger().info(f'[Arduino] {line}')
        except Exception:
            pass


def main():
    rclpy.init()
    node = ConveyorController()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
