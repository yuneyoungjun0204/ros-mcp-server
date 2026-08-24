# ROS MCP Server 명령어 정리

## 환경 설정

### 필수 패키지 설치
```bash
# rosbridge_server 설치 (sudo 필요)
sudo apt update && sudo apt install -y ros-humble-rosbridge-server

# image_tools 설치 (이미지 예제용)
sudo apt install -y ros-humble-image-tools

# image_transport_plugins 설치
sudo apt install -y ros-humble-image-transport-plugins

# turtlesim 설치
sudo apt install -y ros-humble-turtlesim
```

### Python 가상환경 설정
```bash
# uv 설치
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.local/bin/env

# 가상환경 생성
uv venv .venv --python 3.10

# 가상환경 활성화
source .venv/bin/activate

# 프로젝트 의존성 설치
uv pip install -e ".[dev]"
```

### MCP 서버 Claude Code에 추가
```bash
claude mcp add ros-mcp -s user -- uvx ros-mcp --transport=stdio
```

---

## 예제 실행

### 1. Turtlesim 예제 (거북이 시뮬레이터)

```bash
# ROS2 환경 활성화
source /opt/ros/humble/setup.bash

# 방법 1: launch 파일 사용
cd examples/1_turtlesim
ros2 launch ros_mcp_turtlesim.launch.py

# 방법 2: 수동 실행
# 터미널 1: rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 터미널 2: turtlesim
ros2 run turtlesim turtlesim_node
```

### 2. 이미지 예제 (Synthetic Camera - 카메라 없이 테스트)

```bash
# ROS2 환경 활성화
source /opt/ros/humble/setup.bash

# 방법 1: launch 파일 사용
cd examples/8_images
ros2 launch ros_mcp_images_demo.launch.py

# 방법 2: 수동 실행
# 터미널 1: rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 터미널 2: 합성 카메라 (버거 이미지)
ros2 run image_tools cam2image --ros-args -p burger_mode:=true

# 터미널 3: 이미지 뷰어
ros2 run image_tools showimage

# 터미널 4: 이미지 압축
ros2 run image_transport republish raw in:=/image out:=/image/compressed
```

### 3. RealSense 카메라 예제 (실제 카메라 필요)

```bash
# RealSense 패키지 설치
sudo apt install ros-humble-realsense2-camera

# 실행
cd examples/8_images
ros2 launch ros_mcp_images_demo_realsense.launch.py
```

---

## ROS2 기본 명령어

### 토픽 관련
```bash
# 토픽 목록
ros2 topic list

# 토픽 정보
ros2 topic info /topic_name

# 토픽 데이터 확인
ros2 topic echo /topic_name

# 토픽 발행
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

### 서비스 관련
```bash
# 서비스 목록
ros2 service list

# 서비스 타입 확인
ros2 service type /service_name

# 서비스 호출
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3}"
```

### 노드 관련
```bash
# 노드 목록
ros2 node list

# 노드 정보
ros2 node info /node_name
```

---

## MCP를 통한 자연어 명령 예시

### Turtlesim 제어
```
로봇에 연결해서 토픽과 서비스 목록을 보여줘
거북이를 앞으로 이동시켜
거북이를 왼쪽으로 회전시켜
거북이로 사각형을 그려
거북이의 펜 색상을 빨간색으로 바꿔
거북이를 중앙으로 리셋해
```

### 이미지 분석
```
/image 토픽에서 이미지를 캡처해줘
이미지에 뭐가 보여?
햄버거가 몇 개야?
이미지를 분석해줘
```

---

## 문제 해결

### 포트 충돌 (9090)
```bash
# 포트 사용 프로세스 확인
lsof -i :9090

# 포트 강제 해제
fuser -k 9090/tcp
```

### ROS2 데몬 문제
```bash
# 데몬 재시작
ros2 daemon stop
ros2 daemon start
```

### 프로세스 정리
```bash
# 모든 ROS 관련 프로세스 종료
pkill -f rosbridge
pkill -f turtlesim
pkill -f cam2image
```

---

## 프로젝트 구조

```
ros-mcp-server/
├── ros_mcp/              # MCP 서버 코드
│   ├── tools/            # MCP 도구들
│   │   ├── connection.py # 로봇 연결
│   │   ├── topics.py     # 토픽 구독/발행
│   │   ├── services.py   # 서비스 호출
│   │   ├── images.py     # 이미지 처리
│   │   └── ...
│   └── utils/            # 유틸리티
├── examples/             # 예제들
│   ├── 1_turtlesim/      # 거북이 예제
│   ├── 5_docker_turtlesim/ # Docker 예제
│   ├── 8_images/         # 이미지 예제
│   └── ...
└── docs/                 # 문서
```

---

## 주요 MCP 도구

| 도구 | 설명 |
|------|------|
| `connect_to_robot` | rosbridge에 연결 |
| `get_topics` | 토픽 목록 조회 |
| `subscribe_once` | 토픽에서 메시지 1개 수신 |
| `subscribe_for_duration` | 일정 시간 동안 메시지 수신 |
| `publish_once` | 메시지 1회 발행 |
| `publish_for_durations` | 연속 메시지 발행 |
| `view_saved_image` | 저장된 이미지 보기 |
| `get_services` | 서비스 목록 조회 |
| `call_service` | 서비스 호출 |
