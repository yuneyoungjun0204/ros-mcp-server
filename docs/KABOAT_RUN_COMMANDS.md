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
