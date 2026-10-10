#!/usr/bin/env python3
import json
import threading
import time

from geometry_msgs.msg import Pose
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from std_msgs.msg import Bool
from std_msgs.msg import String

from .coordinates import BOUND_NAMES
from .coordinates import DEFAULT_BOUNDS
from .coordinates import TARGET_NAMES
from .coordinates import validate_targets
from .coordinates import validate_xyz
from .sequence_support import MotionCheck
from .sequence_support import SequenceLifecycle

# ===================== 설정 =====================
Z_TRAVEL = 120.0
Z_PICK = 55.0
Z_PLACE = 60.0

# 마스터 기준
MASTER_PUT_L = (-20.0, -250.0, Z_PLACE)
MASTER_PUT_R = (20.0, -250.0, Z_PLACE)
MASTER_WAIT = (0.0, -100.0, Z_TRAVEL)
MASTER_HOME = (0.0, 0.0, Z_TRAVEL)

# 슬레이브 기준
SLAVE_LID = (230.0, 100.0, 20.0)
SLAVE_BOX = (240.0, 0.0, 70.0)
SLAVE_CONVEYOR = (0.0, -200.0, 60.0)
SLAVE_HOME = (0.0, 0.0, Z_TRAVEL)

# ===== 펌프 G-code (serial_bridge의 /raw_cmd로 그대로 전송됨) =====
PUMP_ON_CMD = 'M3S1000'  # 필요하면 800~1200 등으로 조절
PUMP_OFF_CMD = 'M3S0'  # 환경에 따라 "M5"가 더 잘 먹으면 바꿔도 됨

# 펌프 ON/OFF 후 안정화 시간(너무 길면 느려지고, 너무 짧으면 흡착 실패 가능)
PUMP_ON_DELAY = 0.15
PUMP_OFF_DELAY = 0.10


def pose_xyz(x, y, z) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    return p


class OrderYoloSequencer(SequenceLifecycle, Node):

    def __init__(self):
        super().__init__('order_yolo_sequencer')
        global Z_TRAVEL, Z_PICK, MASTER_PUT_L, MASTER_PUT_R
        global MASTER_WAIT, MASTER_HOME, SLAVE_LID, SLAVE_BOX
        global SLAVE_CONVEYOR, SLAVE_HOME, PUMP_ON_CMD, PUMP_OFF_CMD
        global PUMP_ON_DELAY, PUMP_OFF_DELAY
        self.declare_parameter('z_travel', 120.0)
        Z_TRAVEL = self.get_parameter('z_travel').value
        self.declare_parameter('z_pick', 55.0)
        Z_PICK = self.get_parameter('z_pick').value
        self.declare_parameter('master_put_l', [-20.0, -250.0, 60.0])
        MASTER_PUT_L = self.get_parameter('master_put_l').value
        self.declare_parameter('master_put_r', [20.0, -250.0, 60.0])
        MASTER_PUT_R = self.get_parameter('master_put_r').value
        self.declare_parameter('master_wait', [0.0, -100.0, 120.0])
        MASTER_WAIT = self.get_parameter('master_wait').value
        self.declare_parameter('master_home', [0.0, 0.0, 120.0])
        MASTER_HOME = self.get_parameter('master_home').value
        self.declare_parameter('slave_lid', [230.0, 100.0, 20.0])
        SLAVE_LID = self.get_parameter('slave_lid').value
        self.declare_parameter('slave_box', [240.0, 0.0, 70.0])
        SLAVE_BOX = self.get_parameter('slave_box').value
        self.declare_parameter('slave_conveyor', [0.0, -200.0, 60.0])
        SLAVE_CONVEYOR = self.get_parameter('slave_conveyor').value
        self.declare_parameter('slave_home', [0.0, 0.0, 120.0])
        SLAVE_HOME = self.get_parameter('slave_home').value
        self.declare_parameter('pump_on_cmd', 'M3S1000')
        PUMP_ON_CMD = self.get_parameter('pump_on_cmd').value
        self.declare_parameter('pump_off_cmd', 'M3S0')
        PUMP_OFF_CMD = self.get_parameter('pump_off_cmd').value
        self.declare_parameter('pump_on_delay', 0.15)
        PUMP_ON_DELAY = self.get_parameter('pump_on_delay').value
        self.declare_parameter('pump_off_delay', 0.1)
        PUMP_OFF_DELAY = self.get_parameter('pump_off_delay').value

        self.bounds = []
        for name, default in zip(BOUND_NAMES, DEFAULT_BOUNDS):
            self.declare_parameter(name, default)
            self.bounds.append(self.get_parameter(name).value)
        self.settings = {name: self.get_parameter(name).value for name in TARGET_NAMES}
        self.settings.update(z_pick=Z_PICK, z_travel=Z_TRAVEL)
        validate_targets(self.settings, self.bounds)
        self.setup_sequence()
        self.driver_clients = [
            self.create_client(GetParameters, f'/mirobot{i}/serial_bridge/get_parameters')
            for i in (1, 2)
        ]
        self.motion = {
            'master': MotionCheck(25.0, 1.0, 0.1),
            'slave': MotionCheck(25.0, 1.0, 0.1),
        }
        self.pub_slave_raw = self.create_publisher(String, '/mirobot2/raw_cmd', 10)
        self.pub_failed = self.create_publisher(String, '/mr_failed', 10)

        # ===== 입력 =====
        self.sub_cubes = self.create_subscription(
            String, '/vision/cubes', self.cb_cubes, 10
        )
        self.sub_order = self.create_subscription(String, '/mr_move', self.cb_order, 10)

        # busy
        self.sub_master_busy = self.create_subscription(
            Bool, '/mirobot1/is_busy', self.cb_master_busy, 10
        )
        self.sub_slave_busy = self.create_subscription(
            Bool, '/mirobot2/is_busy', self.cb_slave_busy, 10
        )

        # ===== 출력 =====
        # ✅ Pose 로 보내야 serial_bridge가 정상 동작
        self.pub_master = self.create_publisher(Pose, '/mirobot1/target_pose_xyz', 10)
        self.pub_slave = self.create_publisher(Pose, '/mirobot2/target_pose_xyz', 10)

        # ✅ 펌프/원시 G-code는 raw_cmd로 전송
        self.pub_master_raw = self.create_publisher(String, '/mirobot1/raw_cmd', 10)

        self.pub_done = self.create_publisher(String, '/mr_done', 10)

        # ===== 상태 =====
        self.cubes = []
        self.master_busy = False
        self.slave_busy = False

        self.current_order = None
        self.is_running = False

        self.get_logger().info('Order Yolo Sequencer READY')

    # ================= 콜백 =================
    def cb_cubes(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.cubes = data.get('cubes', [])
        except Exception:
            self.cubes = []

    def cb_master_busy(self, msg: Bool):
        self.master_busy = bool(msg.data)
        self.motion['master'].update(msg.data)

    def cb_slave_busy(self, msg: Bool):
        self.slave_busy = bool(msg.data)
        self.motion['slave'].update(msg.data)

    def cb_order(self, msg: String):
        if self.is_running or self.sequence_stopped:
            return
        try:
            order = json.loads(msg.data)
            if not isinstance(order, dict):
                raise ValueError('order must be a JSON object')
            int(order['id'])
            for color in ('red', 'green', 'blue'):
                if int(order.get(color, 0)) < 0:
                    raise ValueError('negative count')
        except (ValueError, KeyError, TypeError):
            self.get_logger().error('Invalid /mr_move json')
            return
        if not self.begin_sequence():
            return
        self.current_order = order
        self.is_running = True
        threading.Thread(target=self.run_sequence, daemon=True).start()

    def abort_sequence(self, reason):
        if self.sequence_running:
            self.pub_failed.publish(
                String(
                    data=json.dumps(
                        {
                            'id': int(self.current_order.get('id', -1)),
                            'ok': False,
                            'reason': reason,
                        }
                    )
                )
            )
        self.finish_sequence(False, reason)

    def on_sequence_stop(self, msg):
        command = (msg.data or '').strip().upper()
        with self.sequence_lock:
            SequenceLifecycle.on_sequence_stop(self, msg)
            if command in ('ESTOP', 'START'):
                raw = String(data='!' if command == 'ESTOP' else '~')
                self.pub_master_raw.publish(raw)
                self.pub_slave_raw.publish(raw)

    def check_driver_bounds(self):
        for client in self.driver_clients:
            if not client.wait_for_service(timeout_sec=2.0):
                raise RuntimeError(
                    'driver parameter service unavailable; workspace unverified'
                )
            future = client.call_async(GetParameters.Request(names=BOUND_NAMES))
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                self.require_running()
                time.sleep(0.01)
            if not future.done() or future.result() is None:
                raise RuntimeError('driver workspace query failed')
            bounds = [parameter_value_to_python(value) for value in future.result().values]
            if bounds != self.bounds:
                raise ValueError(
                    f'driver workspace {bounds} differs from sequence {self.bounds}'
                )

    def wait_motion(self, robot, timeout):
        self.motion[robot].timeout = timeout
        while True:
            self.require_running()
            if self.motion[robot].check():
                return
            time.sleep(0.01)

    def wait_master(self, timeout=25.0):
        self.wait_motion('master', timeout)

    def wait_slave(self, timeout=25.0):
        self.wait_motion('slave', timeout)

    def move_master(self, x, y, z):
        with self.sequence_lock:
            self.require_running()
            validate_xyz('master target', [x, y, z], self.bounds)
            self.motion['master'].reset()
            self.pub_master.publish(pose_xyz(x, y, z))

    def move_slave(self, x, y, z):
        with self.sequence_lock:
            self.require_running()
            validate_xyz('slave target', [x, y, z], self.bounds)
            self.motion['slave'].reset()
            self.pub_slave.publish(pose_xyz(x, y, z))

    def pump_on(self):
        with self.sequence_lock:
            self.require_running()
            self.pub_master_raw.publish(String(data=PUMP_ON_CMD))
        time.sleep(PUMP_ON_DELAY)

    def pump_off(self):
        with self.sequence_lock:
            self.require_running()
            self.pub_master_raw.publish(String(data=PUMP_OFF_CMD))
        time.sleep(PUMP_OFF_DELAY)

    def wait_cube(self, color: str, timeout=3.0):
        # color는 "red/green/blue" 기준으로 찾음
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.require_running()
            for c in self.cubes:
                name = str(c.get('color', '')).lower()
                if color in name:
                    return c
            time.sleep(0.05)
        return None

    # ================= 핵심 시퀀스 =================
    def run_sequence(self):
        self.is_running = True
        try:
            validate_targets(self.settings, self.bounds)
            self.check_driver_bounds()
            self.require_running()
            targets = []
            targets += ['red'] * int(self.current_order.get('red', 0))
            targets += ['green'] * int(self.current_order.get('green', 0))
            targets += ['blue'] * int(self.current_order.get('blue', 0))

            put_positions = [MASTER_PUT_L, MASTER_PUT_R]

            # 1) 마스터: (위→아래→위) + (위→아래→위) 반복
            for idx, color in enumerate(targets):
                cube = self.wait_cube(color)
                if cube is None:
                    raise RuntimeError(f'{color} cube not found')

                cx, cy = float(cube['x']), float(cube['y'])

                # ✅ 반드시 위에서 접근
                self.move_master(cx, cy, Z_TRAVEL)
                self.wait_master()

                # ✅ 반드시 아래로 하강(픽업)
                self.move_master(cx, cy, Z_PICK)
                self.wait_master()

                # ✅ 펌프 ON (픽업 지점에서)
                self.pump_on()

                # ✅ 다시 들어올림
                self.move_master(cx, cy, Z_TRAVEL)
                self.wait_master()

                # 박스 슬롯으로 이동 (위에서)
                px, py, pz = put_positions[idx % 2]
                self.move_master(px, py, Z_TRAVEL)
                self.wait_master()

                # 내려가서 놓기
                self.move_master(px, py, pz)
                self.wait_master()

                # ✅ 펌프 OFF (드롭 지점에서)
                self.pump_off()

                # 다시 들어올림
                self.move_master(px, py, Z_TRAVEL)
                self.wait_master()

            # 마스터 대기 자세(슬레이브와 간섭 방지)
            self.move_master(MASTER_WAIT[0], MASTER_WAIT[1], MASTER_WAIT[2])
            self.wait_master()

            self.get_logger().info('Master job completed')

            # 2) 슬레이브: 마스터 완전히 멈춘 뒤에만 실행
            # 뚜껑 위치
            self.move_slave(SLAVE_LID[0], SLAVE_LID[1], Z_TRAVEL)
            self.wait_slave()
            self.move_slave(SLAVE_LID[0], SLAVE_LID[1], SLAVE_LID[2])
            self.wait_slave()
            self.move_slave(SLAVE_LID[0], SLAVE_LID[1], Z_TRAVEL)
            self.wait_slave()

            # 박스 위치
            self.move_slave(SLAVE_BOX[0], SLAVE_BOX[1], Z_TRAVEL)
            self.wait_slave()
            self.move_slave(SLAVE_BOX[0], SLAVE_BOX[1], SLAVE_BOX[2])
            self.wait_slave()
            self.move_slave(SLAVE_BOX[0], SLAVE_BOX[1], Z_TRAVEL)
            self.wait_slave()

            # 컨베이어 위치
            self.move_slave(SLAVE_CONVEYOR[0], SLAVE_CONVEYOR[1], Z_TRAVEL)
            self.wait_slave()
            self.move_slave(SLAVE_CONVEYOR[0], SLAVE_CONVEYOR[1], SLAVE_CONVEYOR[2])
            self.wait_slave()
            self.move_slave(SLAVE_CONVEYOR[0], SLAVE_CONVEYOR[1], Z_TRAVEL)
            self.wait_slave()

            # 슬레이브 홈
            self.move_slave(SLAVE_HOME[0], SLAVE_HOME[1], SLAVE_HOME[2])
            self.wait_slave()

            # 3) 마스터 홈(마지막)
            self.move_master(MASTER_HOME[0], MASTER_HOME[1], MASTER_HOME[2])
            self.wait_master()

            with self.sequence_lock:
                # DONE
                self.require_running()
                self.pub_done.publish(
                    String(
                        data=json.dumps(
                            {'id': int(self.current_order.get('id', -1)), 'status': 'DONE'},
                            ensure_ascii=False,
                        )
                    )
                )
                self.get_logger().info('Order DONE')
                self.finish_sequence(True)

        except Exception as exc:
            self.get_logger().error(str(exc))
            self.abort_sequence(str(exc))
        finally:
            self.is_running = False


def main():
    rclpy.init()
    node = OrderYoloSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
