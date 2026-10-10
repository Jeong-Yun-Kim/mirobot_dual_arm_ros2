# mirobot_dual_arm_ros2

## 간단한 소개

미로봇 두 대의 명령 복제와 순차·왕복 동작을 구성한다.
컨베이어·서버 명령 연결과 비전 결과를 사용하는 주문 시퀀스를 선택 기능으로 보존한다.
공통 로봇 모델과 드라이버는 별도 control 저장소를 사용한다.

## 권장 실행 환경

Ubuntu 22.04, ROS2 Humble, Python 3.10을 권장한다.
기본 제어에는 미로봇 두 대를 사용하며, 컨베이어 모드에는 START·STOP 명령을 처리하는 Arduino를 추가한다.
비전 모드는 vision 저장소의 카메라·모델·보정 준비를 추가하며, 같은 기본 환경에서 장비 없는 빌드와 제한된 연결 검증만 수행한다.

## 시나리오

왕복 모드는 다음 순서로 처리하며 실제 장비 동작은 미검증 상태이다.

1. 마스터 XYZ 명령을 받아 마스터의 busy 상태를 확인한다.
2. 마스터 완료 후 같은 목표를 슬레이브에 발행한다.
3. 슬레이브 완료 후 슬레이브의 관절 원점 명령을 발행한다.
4. 슬레이브 복귀 후 마스터의 관절 원점 명령을 발행한다.
5. 마스터 복귀 후 `/sequence_active`를 해제한다.

복제 모드는 마스터 명령을 즉시 복사하며, 순차 모드는 완료를 기다린 후 복사하고 설정한 XYZ 복귀점을 사용한다.
비전 모드는 `/vision/cubes`와 `/mr_move`를 받아 마스터 피킹·투입 후 슬레이브 위치 시퀀스를 처리하지만 슬레이브 흡착 제어는 구현하지 않는다.

## 패키지 구조

| 경로 | 역할 |
| --- | --- |
| `mirobot_master_slave/mirobot_master_slave` | 복제·순차·왕복·주문 시퀀스와 CLI를 구현한다. |
| `mirobot_master_slave/launch` | 드라이버 두 개와 선택한 시퀀스 하나를 실행한다. |
| `mirobot_master_slave/config/dual_arm.yaml` | 작업 범위, 복귀점과 비전 시퀀스 좌표를 지정한다. |
| `dependencies.repos` | 공통 control 저장소와 C++ serial 의존성을 지정한다. |
| `vision.repos` | 비전 모드에만 필요한 vision 저장소를 지정한다. |

## 빌드 및 실행 방법

ROS2 Humble이 설치된 새 터미널에서 다음 순서로 구성한다.

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool python3-venv
mkdir -p ~/mirobot_dual_arm_ws/src
cd ~/mirobot_dual_arm_ws
git clone https://github.com/Jeong-Yun-Kim/mirobot_dual_arm_ros2.git src/mirobot_dual_arm_ros2
vcs import src < src/mirobot_dual_arm_ros2/dependencies.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build
source install/setup.bash
```

`rosdep`을 처음 사용하는 시스템은 `sudo rosdep init`을 먼저 실행한다.

장치 없는 launch 구성을 확인하며, 두 번째 명령은 시리얼을 차단한 드라이버만 실행하고 자동 초기 관절 명령은 발행하지 않는다.

```bash
ros2 launch mirobot_master_slave mirobot_master_slave.launch.py --show-args
ros2 launch mirobot_master_slave mirobot_master_slave.launch.py dry_run:=true mode:=none
```

실제 두 대의 장비에는 다음 명령을 사용하며, `mode`는 `replicate`, `sequential`, `roundtrip` 중 하나만 선택한다.
순차·왕복·비전 시퀀스는 각 이동의 새 busy→idle 상태를 확인하며, 시간 초과·상태 미수신·정지 시 실패로 종료하고 다음 이동을 발행하지 않는다.
`dry_run`은 완료 피드백을 모사하지 않으므로 자동 시퀀스의 정상 완료 시험에는 사용하지 않는다.

```bash
ros2 launch mirobot_master_slave mirobot_master_slave.launch.py   port1:=/dev/ttyUSB0 port2:=/dev/ttyUSB1 baud:=115200 mode:=roundtrip
```

별도 터미널에서 명령을 입력하며, `initialize_joints:=true`는 시작 시 관절 0 명령을 발행하므로 의도한 경우에만 지정한다.

```bash
source /opt/ros/humble/setup.bash
source ~/mirobot_dual_arm_ws/install/setup.bash
ros2 run mirobot_master_slave master_cli
```

선택 기능은 왕복 모드에서 다음과 같이 실행하며, `server_bridge`에는 외부 `/mr_move`·`/mr_estop` 발행자가 필요하다.
Arduino 펌웨어와 외부 서버는 이 저장소에 포함하지 않는다.
`/mr_estop`은 `ESTOP`과 `START`만 처리하며, 알 수 없는 입력은 무시한다.
`START`는 정지 잠금을 해제하지만 취소된 작업을 자동 재실행하지 않으므로 새 주문을 전송해야 한다.
`server_bridge`는 내부 `/sequence_request`·`/sequence_result`의 요청 ID와 성공 결과를 확인한 경우에만 기존 형식의 `/mr_done`을 발행한다.
실패는 `/mr_failed`에 `id`, `ok: false`, `reason`으로 알리므로 외부 서버에서 해당 실패 토픽을 처리해야 한다.
`/sequence_active`가 false인 사실만으로 성공을 판정하지 않으며 컨베이어도 실패 시 재가동하지 않는다.

```bash
ros2 run mirobot_master_slave conveyor_controller --ros-args   -p port:=/dev/ttyACM0 -p baud:=115200 -p startup_state:=STOP
ros2 run mirobot_master_slave server_bridge
```

비전 모드는 아래 의존성을 추가하고 vision README의 가상환경·YOLO·가중치·보정 절차를 이 워크스페이스에도 적용한 후 `python -m colcon build`로 빌드한다.

```bash
cd ~/mirobot_dual_arm_ws
vcs import src < src/mirobot_dual_arm_ros2/vision.repos
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r src/mirobot_vision_ros2/vision_robot/requirements.txt
python -m colcon build
source install/setup.bash
cp src/mirobot_vision_ros2/vision_robot/config/vision.yaml vision.local.yaml
cp src/mirobot_dual_arm_ros2/mirobot_master_slave/config/dual_arm.yaml dual_arm.local.yaml
nano vision.local.yaml dual_arm.local.yaml
```

실제 비전 모드에서는 준비한 설정으로 인식 발행기와 시퀀스를 별도 터미널에서 실행하며, `server_bridge`나 다른 복제 모드는 함께 실행하지 않는다.
원본 예시 좌표 중 X≤20 또는 Z=20인 위치는 기본 드라이버 범위 밖이므로 비전 모드의 launch 사전 검사에서 실행을 차단한다.
측정한 위치를 `dual_arm.yaml`에 입력하며 범위를 임의로 넓혀 검사만 통과시키지 않는다.
배열 길이·유한값·접근 높이·작업 범위를 검사하고, 주문 시작 전에 두 드라이버의 실제 범위도 조회하여 불일치나 응답 실패 시 중단한다.
실시간 인식 좌표도 명령 발행 전에 검사하므로 범위 밖 좌표를 조용히 제한하여 계속 진행하지 않는다.

```bash
ros2 launch vision_robot vision_cubes.launch.py params:="$HOME/mirobot_dual_arm_ws/vision.local.yaml"
ros2 launch mirobot_master_slave mirobot_master_slave.launch.py mode:=vision   params:="$HOME/mirobot_dual_arm_ws/dual_arm.local.yaml"
```

`dual_arm.local.yaml`은 설치된 `config/dual_arm.yaml`을 복사하여 준비한다.
RViz는 측정 위치가 아닌 관절 명령을 표시한다.
관련 노드는 같은 `ROS_DOMAIN_ID`에서 실행하며, `rviz:=true`의 두 로봇 간 0.5m 배치는 시각화용 예시로 사용한다.

## 참고 목록

- [Jeong-Yun-Kim/Mirobot_ros2](https://github.com/Jeong-Yun-Kim/Mirobot_ros2)의 소스를 분리하여 사용한다.
- 기존 파일의 저작권과 라이선스 선언을 보존하며, 상충하거나 미정인 선언은 새 라이선스로 확정하지 않는다.
