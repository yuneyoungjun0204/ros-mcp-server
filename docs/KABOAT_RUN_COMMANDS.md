# KABOAT 보트 구동 커맨드 모음

각 블록을 새 터미널을 열고 그대로 붙여넣어 실행. (`connect_to_robot` 한 줄만 예외 — Claude Code 대화창에서 호출)

> ⚠️ **2번(수동 조종)과 3번(LLM 자율주행)은 둘 다 스러스터를 직접 제어합니다. 동시에 켜지 말고 둘 중 하나만 사용하세요.**

---

## 1. 시뮬레이터 켜기

```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
ros2 launch kaboat_pkg kaboat_sim.launch.py
```

Gazebo GUI가 열리고 WAM-V가 스폰된 뒤 약 8초 후 플랫폼에서 릴리즈됨.

---

## 2. 수동 조종 (WASD)

**터미널 A** — 스러스터 브릿지 + 보트 릴리즈:
```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
ros2 launch kaboat_autonomous keyboard_teleop.launch.py
```

**터미널 B** — 키 입력 (반드시 별도 터미널에서 직접 실행. `ros2 launch`는 자식 프로세스에 stdin을 연결해주지 않아 키 입력을 못 받음):
```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
ros2 run kaboat_autonomous keyboard_teleop
```

조작: `w` 전진 · `s` 후진 · `a` 좌회전 · `d` 우회전 · `x`/`space` 정지 · `q`/`Ctrl+C` 종료

---

## 3. LLM 연결 및 활성화

**터미널 C** — rosbridge (ros-mcp-server가 붙는 곳):
```zsh
export PATH="/usr/bin:$PATH"
nohup /usr/bin/python3 /opt/ros/humble/lib/rosapi/rosapi_node \
    --ros-args -r __node:=rosapi > /tmp/rosapi.log 2>&1 &
nohup /usr/bin/python3 /opt/ros/humble/lib/rosbridge_server/rosbridge_websocket \
    --ros-args -p port:=9090 > /tmp/rosbridge.log 2>&1 &
```

**터미널 D** — 자율주행 스택 전체 기동 (센서/스러스터 브릿지, motor_controller, mission_runner, llm_interface, action_dispatcher, sensor_fusion, integrated_visualizer, 보트 릴리즈까지 한 번에):
```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
ros2 launch kaboat_autonomous autonomous.launch.py
```

**Claude Code 대화창** (터미널 아님, MCP 도구 호출로 LLM 활성화):
```
connect_to_robot(ip="127.0.0.1", port=9090)
```
연결되면 `/boat_status`(구독, 1Hz JSON)로 상태를 읽고 `/llm_waypoint`(목표 설정) · `/llm_command`(긴급 직접 제어) · `/llm_override`(오버라이드 on/off)로 제어.

---

## 4. 모듈들 시각화 실행

3번(`autonomous.launch.py`)을 이미 띄웠다면 `integrated_visualizer`는 자동으로 함께 실행됨 — 별도 실행 불필요.
2번(수동 조종)만 켜둔 상태에서 시각화만 따로 보고 싶을 때:
```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
ros2 run kaboat_autonomous integrated_visualizer
```

- **Global Map (왼쪽)**: 보트 위치(빨간 점) · 궤적(파란 선) · 헤딩(초록 선) · 클릭하여 목적지 설정
- **Polar Map (오른쪽)**: LiDAR(파란 점) · 안전 구역 · 전방(초록 선) · 명령 방향(빨간 선)

---

## 5. YOLO/HSV 객체 감지

시뮬레이터(1번) 실행 후 객체 감지 노드 실행:

**터미널 E** — 감지 노드 실행 (HSV 모드, 기본값):
```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
cd ~/ros-mcp-server/kaboat_llm/perception
python3 yolo_detector.py
```

**터미널 F** — 감지 결과 확인 (JSON):
```zsh
source /opt/ros/humble/setup.zsh
ros2 topic echo /detected_objects
```

**터미널 G** — 시각화 이미지 확인 (rqt):
```zsh
source /opt/ros/humble/setup.zsh
rqt_image_view /detection_image
```

### HSV 모드로 실행 (VRX 부표 최적화, 권장)

```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
cd ~/ros-mcp-server/kaboat_llm/perception
python3 yolo_detector.py
```

VRX 부표 색상 감지: `red`, `green`, `blue`, `yellow`, `black` (30ms 이하, 실시간)

### ONNX 모드로 실행 (RoboBoat 대회 모델, 실험적)

```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
cd ~/ros-mcp-server/kaboat_llm/perception
python3 yolo_detector.py --ros-args \
    -p use_yolo:=true \
    -p yolo_model:=/home/yune/ros-mcp-server/kaboat_llm/perception/models/sign-simplified.onnx
```

> ⚠️ **참고**: `sign-simplified.onnx`는 OAK-D 전용 모델입니다. 25개 클래스(부표, 도킹 등)를 지원하지만 출력 형식이 특수하여 추가 개발이 필요합니다.

### YOLOv8 모드로 실행 (범용 COCO 모델)

```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
cd ~/ros-mcp-server/kaboat_llm/perception
python3 yolo_detector.py --ros-args \
    -p use_yolo:=true \
    -p yolo_model:=/home/yune/ros-mcp-server/kaboat_llm/models/yolov8n.pt \
    -p confidence_threshold:=0.5
```

### 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|---------|-------|------|
| `use_yolo` | `false` | YOLO 사용 여부 (false=HSV 색상 감지) |
| `yolo_model` | `''` | YOLO 모델 파일 경로 (.pt) |
| `confidence_threshold` | `0.5` | YOLO 감지 신뢰도 임계값 |
| `camera_topic` | `/wamv/sensors/camera/image_raw` | 카메라 토픽 |
| `enabled_colors` | `['red','green','blue','yellow','black']` | HSV 감지 색상 |
| `min_area` | `500` | HSV 최소 감지 영역 (픽셀) |
| `publish_rate` | `10.0` | 감지 발행 주기 (Hz) |

### 발행 토픽

| 토픽 | 타입 | 내용 |
|-----|------|------|
| `/detected_objects` | `std_msgs/String` | JSON 감지 결과 (label, bbox, confidence 등) |
| `/detection_image` | `sensor_msgs/Image` | 박스가 그려진 시각화 이미지 |

> ⚠️ **카메라 브리지 필요**: YOLO/HSV 감지가 작동하려면 `autonomous.launch.py`(3번)를 먼저 실행하여 카메라 브리지가 활성화되어야 합니다.

---

## 6. 3D LiDAR 클러스터링

LiDAR 포인트클라우드에서 객체 클러스터를 감지하고 RViz에서 시각화:

**터미널 H** — 클러스터 시각화 노드:
```zsh
source /opt/ros/humble/setup.zsh
source ~/vrx_ws/install/setup.zsh
export PATH="/usr/bin:$PATH"
ros2 run kaboat_autonomous cluster_visualizer
```

**터미널 I** — 클러스터 결과 확인 (JSON):
```zsh
source /opt/ros/humble/setup.zsh
ros2 topic echo /lidar_clusters
```

**RViz에서 시각화**:
```zsh
source /opt/ros/humble/setup.zsh
rviz2
```
- Fixed Frame: `wamv/lidar_wamv_link` 또는 노드 시작 로그에 출력된 frame_id
- Add → By topic → `/wamv/sensors/lidar/points` (PointCloud2)
- Add → By topic → `/lidar_clusters_markers` (MarkerArray)

### 발행 토픽

| 토픽 | 타입 | 내용 |
|-----|------|------|
| `/lidar_clusters` | `std_msgs/String` | JSON (mode, raw clusters, stable clusters) |
| `/lidar_clusters_markers` | `MarkerArray` | RViz 시각화 마커 |

### 동작 모드

| 모드 | 조건 | 마커 색상 |
|-----|------|----------|
| **3D 모드** | PointCloud2 수신 중 | 노란(raw) / 초록(안정화) |
| **2D 폴백** | PointCloud2 없음 → LaserScan 사용 | 주황(raw) / 청록(안정화) |
