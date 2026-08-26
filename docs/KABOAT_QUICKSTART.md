# KABOAT 자율주행 시스템 실행 가이드

## 개요
이 문서는 KABOAT 시뮬레이터와 자율주행 시스템을 실행하는 방법을 설명합니다.

## 사전 요구사항
- ROS2 Humble
- Gazebo Garden (gz-sim)
- vrx_ws 워크스페이스 빌드 완료
- ros-mcp-server 설치

---

## 1단계: 환경 설정

### 터미널 열기
새 터미널을 열고 ROS2 환경을 설정합니다:

```bash
source /opt/ros/humble/setup.bash
source /home/yune/vrx_ws/install/setup.bash
```

### Python 경로 문제 해결 (Linuxbrew 사용 시)
Linuxbrew Python이 설치된 경우, ROS2 명령 실행 전에 시스템 Python을 우선 사용하도록 설정:

```bash
export PATH="/usr/bin:$PATH"
```

---

## 2단계: 시뮬레이터 실행

### 시뮬레이터 시작
```bash
ros2 launch kaboat_pkg kaboat_sim.launch.py
```

시뮬레이터가 시작되면:
- Gazebo GUI가 열림
- WAM-V 보트가 물 위에 스폰됨
- 약 8초 후 보트가 플랫폼에서 릴리즈됨

### 시뮬레이터 옵션
```bash
# 헤드리스 모드 (GUI 없음)
ros2 launch kaboat_pkg kaboat_sim.launch.py headless:=True

# 일시정지 상태로 시작
ros2 launch kaboat_pkg kaboat_sim.launch.py paused:=True
```

---

## 3단계: rosbridge 서버 실행

ros-mcp-server가 ROS2와 통신하려면 rosbridge가 필요합니다.

### 새 터미널에서 실행
```bash
# 시스템 Python 사용 (Linuxbrew 충돌 방지)
export PATH="/usr/bin:$PATH"

# rosapi 시작
nohup /usr/bin/python3 /opt/ros/humble/lib/rosapi/rosapi_node \
    --ros-args -r __node:=rosapi > /tmp/rosapi.log 2>&1 &

# rosbridge 시작
nohup /usr/bin/python3 /opt/ros/humble/lib/rosbridge_server/rosbridge_websocket \
    --ros-args -p port:=9090 > /tmp/rosbridge.log 2>&1 &
```

### 연결 확인
```bash
# 포트 확인
ss -tlnp | grep 9090
# 출력: LISTEN ... 0.0.0.0:9090 ...
```

---

## 4단계: 센서 브릿지 실행

Gazebo 센서 데이터를 ROS2로 브릿지합니다.

### 센서 브릿지 시작
```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat" \
  "/world/kaboat_course/model/wamv/link/wamv/imu_wamv_link/sensor/imu_wamv_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU" \
  "/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan" \
  "/world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image" \
  "/wamv/thrusters/left/thrust@std_msgs/msg/Float64@gz.msgs.Double" \
  "/wamv/thrusters/right/thrust@std_msgs/msg/Float64@gz.msgs.Double" \
  &
```

### 보트 릴리즈 (필요시)
보트가 플랫폼에 고정되어 있으면 수동 릴리즈:
```bash
gz topic -t /vrx/release -m gz.msgs.Empty -p ''
```

---

## 5단계: 시스템 확인

### ROS2 토픽 확인
```bash
ros2 topic list
```

예상 출력:
```
/clock
/wamv/joint_states
/wamv/pose
/wamv/thrusters/left/thrust
/wamv/thrusters/right/thrust
/world/kaboat_course/model/wamv/.../navsat
...
```

### Gazebo에서 직접 GPS 확인
```bash
gz topic -e -t /world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat -n 1
```

### 스러스터 테스트 (ROS2)
```bash
# 전진
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/msg/Float64 "{data: 100}" --once
ros2 topic pub /wamv/thrusters/right/thrust std_msgs/msg/Float64 "{data: 100}" --once
```

---

## 6단계: ros-mcp-server 연결

### Claude Code에서 연결
ros-mcp-server MCP 도구를 사용하여 연결:

```
connect_to_robot(ip="127.0.0.1", port=9090)
```

### 토픽 목록 확인
```
get_topics()
```

### 스러스터 제어
```
publish_for_durations(
    topic="/wamv/thrusters/left/thrust",
    msg_type="std_msgs/msg/Float64",
    messages=[{"data": 100}],
    durations=[2],
    rate_hz=10
)
```

---

## 빠른 시작 스크립트

모든 단계를 한 번에 실행하는 스크립트:

```bash
#!/bin/bash
# start_kaboat.sh

# 환경 설정
export PATH="/usr/bin:$PATH"
source /opt/ros/humble/setup.bash
source /home/yune/vrx_ws/install/setup.bash

# 시뮬레이터 시작 (백그라운드)
ros2 launch kaboat_pkg kaboat_sim.launch.py &
sleep 10

# rosbridge 시작
nohup /usr/bin/python3 /opt/ros/humble/lib/rosapi/rosapi_node \
    --ros-args -r __node:=rosapi > /tmp/rosapi.log 2>&1 &
nohup /usr/bin/python3 /opt/ros/humble/lib/rosbridge_server/rosbridge_websocket \
    --ros-args -p port:=9090 > /tmp/rosbridge.log 2>&1 &
sleep 2

# 센서 브릿지 시작
ros2 run ros_gz_bridge parameter_bridge \
  "/wamv/thrusters/left/thrust@std_msgs/msg/Float64@gz.msgs.Double" \
  "/wamv/thrusters/right/thrust@std_msgs/msg/Float64@gz.msgs.Double" \
  &
sleep 3

# 보트 릴리즈
gz topic -t /vrx/release -m gz.msgs.Empty -p ''

echo "KABOAT 시스템 준비 완료!"
echo "rosbridge: ws://localhost:9090"
```

---

## 문제 해결

### rosbridge 연결 실패
```bash
# 포트 확인
ss -tlnp | grep 9090

# 프로세스 재시작
pkill -f rosbridge_websocket
pkill -f rosapi_node
# 3단계 다시 실행
```

### 보트가 움직이지 않음
```bash
# 릴리즈 확인
gz topic -t /vrx/release -m gz.msgs.Empty -p ''

# 시뮬레이션 일시정지 해제
gz service -s /world/kaboat_course/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req 'pause: false'
```

### Python 모듈 오류
```bash
# Linuxbrew Python 대신 시스템 Python 사용
export PATH="/usr/bin:$PATH"
```

---

## 토픽 참조

### 제어 토픽 (ROS2 → Gazebo)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/wamv/thrusters/left/thrust` | `std_msgs/Float64` | 좌측 스러스터 (-250~250 N) |
| `/wamv/thrusters/right/thrust` | `std_msgs/Float64` | 우측 스러스터 (-250~250 N) |

### 센서 토픽 (Gazebo → ROS2)
| Gazebo 토픽 | ROS2 타입 | 설명 |
|-------------|-----------|------|
| `.../navsat/navsat` | `NavSatFix` | GPS 위치 |
| `.../imu_wamv_sensor/imu` | `Imu` | IMU 데이터 |
| `.../lidar_wamv_sensor/scan` | `LaserScan` | LiDAR 스캔 |
| `.../front_left_camera_sensor/image` | `Image` | 전방 좌측 카메라 |

### Gazebo 직접 접근
```bash
# GPS 읽기
gz topic -e -t /world/kaboat_course/model/wamv/link/wamv/gps_wamv_link/sensor/navsat/navsat -n 1

# 시뮬레이션 상태
gz topic -e -t /stats -n 1
```

---

## 자율주행 시스템 (장애물 회피)

KABOAT 자율주행 시스템은 SeaNU_KABOAT2024 기반의 Cost 함수 장애물 회피 알고리즘을 사용합니다.

### 자율주행 패키지 빌드

```bash
cd ~/vrx_ws
colcon build --packages-select kaboat_autonomous
source install/setup.bash
```

### 자율주행 시스템 실행

```bash
# 터미널 1: 시뮬레이터 (2단계 참조)
ros2 launch kaboat_pkg kaboat_sim.launch.py

# 터미널 2: 자율주행 노드
ros2 launch kaboat_autonomous autonomous.launch.py

# 터미널 3: 통합 시각화 (선택)
ros2 run kaboat_autonomous integrated_visualizer
```

### 시각화 사용법

**Global Map (왼쪽)**
- 보트 위치: 빨간 점
- 이동 궤적: 파란 선
- 헤딩 방향: 초록 선
- **클릭하여 목적지 설정**

**Polar Map (오른쪽)**
- LiDAR 데이터: 파란 점 (극좌표)
- 안전 구역: 반투명 파란 영역
- 전방 (0°): 초록 선
- 명령 방향: 빨간 선 (psi_error)

### 수동 웨이포인트 설정

```bash
# ROS2 토픽으로 직접 웨이포인트 전송
ros2 topic pub /waypoint_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'world'}, point: {x: -520.0, y: 160.0, z: 0.0}}" --once
```

### 자율주행 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/command` | `Float32MultiArray` | [psi_error, tau_x, max_sat] |
| `/waypoint_goal` | `PointStamped` | 목적지 좌표 (클릭/수동) |

### 파라미터 조정

`kaboat_autonomous/config/settings.py`:

```python
# PD 제어
KP = 50.0              # 비례 계수
KD = 10.0              # 미분 계수
MAX_THRUST = 500.0     # 최대 각속도 (rad/s)

# 장애물 회피
BOAT_WIDTH = 2.5       # 보트 폭 (m)
AVOID_RANGE = 5.0      # 회피 거리 (m)
GOAL_RANGE = 3.0       # 도착 판정 거리 (m)
```

파라미터 변경 후 패키지 재빌드 필요:
```bash
cd ~/vrx_ws
colcon build --packages-select kaboat_autonomous
source install/setup.bash
```

### 장애물 회피 알고리즘

1. **안전 구역 계산**: LiDAR 360° 스캔으로 AVOID_RANGE 내 장애물 감지
2. **Cost 함수 적용**: 목표 방향과 장애물 거리 기반 최적 조향각 계산
3. **경로 판단**: 목적지까지 직진 가능 여부 판단
4. **PD 제어**: 조향 오차를 좌/우 스러스터 차동 출력으로 변환

---

## 빠른 테스트 (통합 실행)

### 전체 시스템 실행

```bash
# 터미널 1: 시뮬레이터
source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash
ros2 launch kaboat_pkg kaboat_sim.launch.py

# 터미널 2: 자율주행 + 시각화 (통합)
source /opt/ros/humble/setup.bash
source ~/vrx_ws/install/setup.bash
ros2 launch kaboat_autonomous autonomous.launch.py

# 터미널 3: 토픽 모니터링 (선택)
ros2 topic echo /command
```

### 클릭으로 목적지 설정

1. 시각화 창의 **왼쪽 Global Map** 클릭
2. 터미널에 `=== WAYPOINT RECEIVED ===` 로그 확인
3. `/command published: ...` 로그가 1초마다 출력
4. 보트가 클릭한 위치로 이동 시작

### 진단 명령어

```bash
# 노드 상태 확인
ros2 node list | grep -E "mission|motor|visualizer"

# 웨이포인트 수신 확인
ros2 topic echo /waypoint_goal

# 명령 발행 확인
ros2 topic echo /command

# 스러스터 출력 확인
ros2 topic echo /wamv/thrusters/left/thrust
ros2 topic echo /wamv/thrusters/right/thrust
```

### 수동 웨이포인트 전송

```bash
# 특정 좌표로 이동 명령
ros2 topic pub /waypoint_goal geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'map'}, point: {x: 10.0, y: 20.0, z: 0.0}}" --once
```

### 기존 노드 정리

```bash
# 모든 kaboat 관련 프로세스 종료
pkill -f kaboat_autonomous
pkill -f ros_gz_bridge
```

---

## LiDAR 설정

LiDAR 높이: **1.5m** (기본 0.45m에서 상향)

설정 파일: `vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro`
```xml
<xacro:lidar name="lidar_wamv" type="16_beam" z="1.5" post_z_from="1.0"/>
```

변경 후 재빌드 필요:
```bash
cd ~/vrx_ws
colcon build --packages-select wamv_gazebo --merge-install
```

---

## 다음 단계
- [호핑투어 미션 실행](docs/hopping_tour.md)
- [웨이포인트 네비게이션](docs/navigation.md)
- [장애물 회피](docs/obstacle_avoidance.md)
