#!/usr/bin/env python3
"""
Gemini MCP Bridge - Gemini를 ros-mcp처럼 ROS2와 연동

Claude + MCP와 동일한 방식으로 Gemini가 ROS2 도구를 직접 호출할 수 있도록 함.
Gemini Function Calling을 사용하여 rosbridge WebSocket으로 명령 전달.

사용법:
    from gemini_mcp_bridge import GeminiMCPBridge

    bridge = GeminiMCPBridge(api_key="your-api-key")
    bridge.connect("127.0.0.1", 9090)

    # 단일 명령
    response = bridge.chat("현재 ROS 토픽 목록을 보여줘")

    # 자율 루프
    bridge.run_autonomous_loop(interval=0.5)
"""
import json
import time
import threading
from typing import Optional, Dict, Callable
from pathlib import Path

import websocket
import google.generativeai as genai
from google.generativeai.types import FunctionDeclaration, Tool


# ============================================================
# WebSocket Manager (ros-mcp 방식 그대로)
# ============================================================

class RosbridgeClient:
    """rosbridge WebSocket 클라이언트"""

    def __init__(self, ip: str = "127.0.0.1", port: int = 9090, timeout: float = 5.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.ws: Optional[websocket.WebSocket] = None
        self.lock = threading.RLock()

    def connect(self) -> bool:
        """rosbridge에 연결"""
        with self.lock:
            if self.ws and self.ws.connected:
                return True
            try:
                url = f"ws://{self.ip}:{self.port}"
                self.ws = websocket.create_connection(url, timeout=self.timeout)
                print(f"[RosbridgeClient] Connected to {url}")
                return True
            except Exception as e:
                print(f"[RosbridgeClient] Connection failed: {e}")
                self.ws = None
                return False

    def disconnect(self):
        """연결 종료"""
        with self.lock:
            if self.ws:
                try:
                    self.ws.close()
                except:
                    pass
                self.ws = None

    def send(self, message: dict) -> bool:
        """메시지 전송"""
        with self.lock:
            if not self.connect():
                return False
            try:
                self.ws.send(json.dumps(message))
                return True
            except Exception as e:
                print(f"[RosbridgeClient] Send error: {e}")
                self.disconnect()
                return False

    def receive(self, timeout: Optional[float] = None) -> Optional[dict]:
        """메시지 수신"""
        with self.lock:
            if not self.ws:
                return None
            try:
                self.ws.settimeout(timeout or self.timeout)
                raw = self.ws.recv()
                return json.loads(raw)
            except Exception as e:
                print(f"[RosbridgeClient] Receive error: {e}")
                return None

    def request(self, message: dict, timeout: Optional[float] = None) -> dict:
        """요청-응답"""
        if not self.send(message):
            return {"error": "send failed"}
        response = self.receive(timeout)
        if response is None:
            return {"error": "no response"}
        return response


# ============================================================
# ROS MCP Tools - Gemini Function 형식으로 정의
# ============================================================

ROS_TOOLS = [
    FunctionDeclaration(
        name="get_topics",
        description="ROS에서 사용 가능한 모든 토픽 목록을 가져옵니다.",
        parameters={"type": "object", "properties": {}, "required": []}
    ),
    FunctionDeclaration(
        name="get_topic_type",
        description="특정 토픽의 메시지 타입을 가져옵니다.",
        parameters={
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "토픽 이름 (예: /cmd_vel)"}
            },
            "required": ["topic"]
        }
    ),
    FunctionDeclaration(
        name="subscribe_to_topic",
        description="토픽을 구독하고 최신 메시지를 가져옵니다.",
        parameters={
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "구독할 토픽 이름"},
                "msg_type": {"type": "string", "description": "메시지 타입 (예: std_msgs/String)"},
                "timeout": {"type": "number", "description": "대기 시간(초), 기본값 2.0"}
            },
            "required": ["topic", "msg_type"]
        }
    ),
    FunctionDeclaration(
        name="publish_to_topic",
        description="토픽에 메시지를 발행합니다.",
        parameters={
            "type": "object",
            "properties": {
                "topic": {"type": "string", "description": "발행할 토픽 이름"},
                "msg_type": {"type": "string", "description": "메시지 타입"},
                "message": {"type": "object", "description": "발행할 메시지 내용 (JSON 객체)"}
            },
            "required": ["topic", "msg_type", "message"]
        }
    ),
    FunctionDeclaration(
        name="call_service",
        description="ROS 서비스를 호출합니다.",
        parameters={
            "type": "object",
            "properties": {
                "service": {"type": "string", "description": "서비스 이름"},
                "service_type": {"type": "string", "description": "서비스 타입"},
                "args": {"type": "object", "description": "서비스 인자"}
            },
            "required": ["service", "service_type"]
        }
    ),
    FunctionDeclaration(
        name="get_boat_status",
        description="보트의 현재 상태(위치, 헤딩, 장애물)를 가져옵니다.",
        parameters={"type": "object", "properties": {}, "required": []}
    ),
    FunctionDeclaration(
        name="send_action_command",
        description="action_dispatcher에 명령을 보냅니다 (navigate_avoid, orbit, gate_pass, dorodori, align, hover, backward, stop 등).",
        parameters={
            "type": "object",
            "properties": {
                "action": {"type": "string", "description": "액션 이름"},
                "params": {"type": "object", "description": "액션 파라미터"}
            },
            "required": ["action"]
        }
    ),
    FunctionDeclaration(
        name="navigate_between_points",
        description="2D LiDAR 스캔의 두 점 사이 중점으로 직선 통과합니다. 부표/게이트 사이를 통과할 때 필수! get_boat_status()의 nearest_obstacles에서 왼쪽/오른쪽 물체의 scan_idx를 찾아 사용하세요.",
        parameters={
            "type": "object",
            "properties": {
                "left_idx": {"type": "integer", "description": "왼쪽 물체의 LiDAR 스캔 인덱스 (270-360 범위, 예: 315)"},
                "right_idx": {"type": "integer", "description": "오른쪽 물체의 LiDAR 스캔 인덱스 (0-90 범위, 예: 45)"},
                "extend_dist": {"type": "number", "description": "통과 후 연장 거리(m). 기본값 10"}
            },
            "required": ["left_idx", "right_idx"]
        }
    ),
    FunctionDeclaration(
        name="find_obstacles_in_range",
        description="특정 각도 범위 내에서 가장 가까운 LiDAR 점을 찾습니다. 이미지에서 '왼쪽에 부표'를 감지했다면 find_obstacles_in_range(270, 360)으로 해당 스캔 인덱스를 찾으세요.",
        parameters={
            "type": "object",
            "properties": {
                "angle_min": {"type": "integer", "description": "시작 각도 (0-360, 0=전방)"},
                "angle_max": {"type": "integer", "description": "끝 각도 (0-360)"},
                "max_range": {"type": "number", "description": "최대 탐색 거리(m). 기본값 30"}
            },
            "required": ["angle_min", "angle_max"]
        }
    ),
]


# ============================================================
# Gemini MCP Bridge
# ============================================================

class GeminiMCPBridge:
    """Gemini를 ros-mcp처럼 사용하는 브릿지"""

    SYSTEM_PROMPT = """당신은 KABOAT 자율주행 보트의 AI 조종사입니다.
매 턴마다 get_boat_status()로 상태를 확인하고, 즉시 액션을 실행하세요.

=== 미션 웨이포인트 (로컬 좌표) ===
1. start: (2.7, -6.8) - 시작점
2. gate_start: (-1.6, 6.3) - 게이트 통과 시작
3. gate_end: (-3.4, 87.8) - 게이트 통과 끝
4. buoy_orbit: (-1.0, 116.0) - 부표선회
5. hopping: (43.7, 105.6) - 호핑투어
6. dock: (49.2, 16.3) - 도킹

=== 풀 미션 순서 ===
1. GATE: navigate_avoid로 gate_start(-1.6, 6.3) 이동 → 게이트 감지 시 gate_pass
2. BUOY: navigate_avoid로 buoy_orbit(-1.0, 116.0) 이동 → 부표 감지 시 orbit
3. HOPPING: navigate_avoid로 hopping(43.7, 105.6) 이동
4. DOCK: navigate_avoid로 dock(49.2, 16.3) 이동 → 접근 후 stop

=== 액션 명령어 ===
- navigate_avoid: 장애물 회피 이동 (goal_x, goal_y)
- navigate_direct: 직진 이동 (goal_x, goal_y)
- orbit: 부표 선회 (lidar_idx, radius, direction, laps)
- gate_pass: 게이트 통과 (left_idx, right_idx)
- align: 헤딩 정렬 (heading) - 클러스터 방향으로 정렬
- align_to_cluster: 클러스터 ID로 정렬 (cluster_id)
- dorodori: 좌우 탐색 - 아무것도 안 보일 때만 사용
- backward: 후진 (duration)
- stop: 정지

=== 2D LiDAR 스캔 기반 통과 (핵심 기능) ===
navigate_between_points(left_idx, right_idx): 두 스캔 점 사이로 직선 통과
find_obstacles_in_range(angle_min, angle_max): 각도 범위 내 장애물 찾기

★ LiDAR 각도 규칙 ★
- 0° = 전방, 90° = 우측, 180° = 후방, 270° = 좌측
- 스캔 인덱스 ≈ 각도 (360개 점)

★ 부표 사이 통과 워크플로우 ★
1. find_obstacles_in_range(270, 360) → 왼쪽 장애물의 scan_idx 확인
2. find_obstacles_in_range(0, 90) → 오른쪽 장애물의 scan_idx 확인
3. navigate_between_points(left_scan_idx, right_scan_idx) 실행!

예시: 왼쪽 315°, 오른쪽 45°에 부표 감지
→ navigate_between_points(315, 45)

★★★ 부표 2개 보이면 무조건 navigate_between_points 사용! ★★★

=== 판단 규칙 (우선순위 순서대로!) ===
★★★ 최우선: 양쪽에 장애물 보이면 → navigate_between_points ★★★

1. get_boat_status() 호출하여 clusters 확인
2. 양쪽(좌+우)에 장애물 있으면:
   - find_obstacles_in_range(270, 360) → left_idx
   - find_obstacles_in_range(0, 90) → right_idx
   - navigate_between_points(left_idx, right_idx) 실행!
3. 전방에만 장애물 → align 후 접근 또는 orbit
4. 장애물 없음 → navigate_direct로 전진

⚠️ dorodori 사용 금지:
- 클러스터가 1개라도 보이면 dorodori 절대 사용 금지
- 전방 50m 내에 아무것도 없을 때만 dorodori 허용

⚠️ 핵심: 부표/게이트 보이면 그 사이로 직선 통과!
- navigate_between_points = 직선 통과 (장애물 회피 없음)
- 경로 이탈 금지!

절대 멈추지 말고 계속 다음 액션을 실행하세요."""

    def __init__(self, api_key: Optional[str] = None):
        # API 키 로드
        if api_key is None:
            api_key = self._load_api_key()

        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel(
            model_name="gemini-3.6-flash",
            tools=[Tool(function_declarations=ROS_TOOLS)],
            system_instruction=self.SYSTEM_PROMPT
        )
        self.chat_session = None
        self.rosbridge = RosbridgeClient()
        self._autonomous_running = False
        self._last_boat_status = {}

    def _load_api_key(self) -> str:
        """환경변수 또는 .env에서 API 키 로드"""
        import os
        key = os.environ.get('GOOGLE_API_KEY') or os.environ.get('GEMINI_API_KEY')
        if key:
            return key

        for env_path in [
            Path('/home/yune/ros-mcp-server/kaboat_llm/web/.env'),
            Path.home() / '.env',
        ]:
            if env_path.exists():
                for line in env_path.read_text().strip().split('\n'):
                    if '=' in line and not line.startswith('#'):
                        k, v = line.split('=', 1)
                        if k.strip() in ('GOOGLE_API_KEY', 'GEMINI_API_KEY'):
                            return v.strip()
        raise ValueError("Gemini API key not found")

    def connect(self, ip: str = "127.0.0.1", port: int = 9090) -> bool:
        """rosbridge에 연결"""
        self.rosbridge.ip = ip
        self.rosbridge.port = port
        return self.rosbridge.connect()

    def disconnect(self):
        """연결 종료"""
        self._autonomous_running = False
        self.rosbridge.disconnect()

    # ============================================================
    # ROS Tool 구현
    # ============================================================

    def _execute_function(self, name: str, args: dict) -> dict:
        """Gemini function call 실행"""
        try:
            if name == "get_topics":
                return self._get_topics()
            elif name == "get_topic_type":
                return self._get_topic_type(args.get("topic", ""))
            elif name == "subscribe_to_topic":
                return self._subscribe_to_topic(
                    args.get("topic", ""),
                    args.get("msg_type", ""),
                    args.get("timeout", 2.0)
                )
            elif name == "publish_to_topic":
                return self._publish_to_topic(
                    args.get("topic", ""),
                    args.get("msg_type", ""),
                    args.get("message", {})
                )
            elif name == "call_service":
                return self._call_service(
                    args.get("service", ""),
                    args.get("service_type", ""),
                    args.get("args", {})
                )
            elif name == "get_boat_status":
                return self._get_boat_status()
            elif name == "send_action_command":
                return self._send_action_command(
                    args.get("action", ""),
                    args.get("params", {})
                )
            elif name == "navigate_between_points":
                return self._navigate_between_points(
                    args.get("left_idx", 315),
                    args.get("right_idx", 45),
                    args.get("extend_dist", 10.0)
                )
            elif name == "find_obstacles_in_range":
                return self._find_obstacles_in_range(
                    args.get("angle_min", 0),
                    args.get("angle_max", 360),
                    args.get("max_range", 30.0)
                )
            else:
                return {"error": f"Unknown function: {name}"}
        except Exception as e:
            return {"error": str(e)}

    def _get_topics(self) -> dict:
        """토픽 목록 조회"""
        msg = {
            "op": "call_service",
            "service": "/rosapi/topics",
            "type": "rosapi/Topics",
            "args": {},
            "id": "get_topics"
        }
        response = self.rosbridge.request(msg)
        if "values" in response:
            return response["values"]
        return response

    def _get_topic_type(self, topic: str) -> dict:
        """토픽 타입 조회"""
        msg = {
            "op": "call_service",
            "service": "/rosapi/topic_type",
            "type": "rosapi/TopicType",
            "args": {"topic": topic},
            "id": f"topic_type_{topic}"
        }
        response = self.rosbridge.request(msg)
        if "values" in response:
            return response["values"]
        return response

    def _subscribe_to_topic(self, topic: str, msg_type: str, timeout: float = 2.0) -> dict:
        """토픽 구독 및 메시지 수신"""
        sub_msg = {
            "op": "subscribe",
            "topic": topic,
            "type": msg_type,
            "id": f"sub_{topic}"
        }
        if not self.rosbridge.send(sub_msg):
            return {"error": "subscribe failed"}

        # 메시지 대기
        response = self.rosbridge.receive(timeout=timeout)

        # 구독 해제
        unsub_msg = {"op": "unsubscribe", "topic": topic, "id": f"unsub_{topic}"}
        self.rosbridge.send(unsub_msg)

        if response and "msg" in response:
            return {"topic": topic, "message": response["msg"]}
        return {"error": "no message received", "timeout": timeout}

    def _publish_to_topic(self, topic: str, msg_type: str, message: dict) -> dict:
        """토픽에 메시지 발행"""
        pub_msg = {
            "op": "publish",
            "topic": topic,
            "type": msg_type,
            "msg": message,
            "id": f"pub_{topic}"
        }
        if self.rosbridge.send(pub_msg):
            return {"success": True, "topic": topic}
        return {"error": "publish failed"}

    def _call_service(self, service: str, service_type: str, args: dict = None) -> dict:
        """서비스 호출"""
        msg = {
            "op": "call_service",
            "service": service,
            "type": service_type,
            "args": args or {},
            "id": f"srv_{service}"
        }
        return self.rosbridge.request(msg)

    def _get_boat_status(self) -> dict:
        """보트 상태 조회"""
        result = self._subscribe_to_topic("/boat_status", "std_msgs/String", timeout=2.0)
        if "message" in result and "data" in result["message"]:
            try:
                status = json.loads(result["message"]["data"])
                self._last_boat_status = status
                return status
            except:
                pass
        return self._last_boat_status or {"error": "no status available"}

    def _send_action_command(self, action: str, params: dict = None) -> dict:
        """action_dispatcher에 명령 전송"""
        cmd = {"action": action}
        if params:
            cmd.update(params)

        return self._publish_to_topic(
            "/llm_action",
            "std_msgs/String",
            {"data": json.dumps(cmd)}
        )

    def _navigate_between_points(self, left_idx: int, right_idx: int, extend_dist: float = 10.0) -> dict:
        """2D LiDAR 스캔 두 점 사이로 직선 통과"""
        cmd = {
            "action": "pass_between_clusters",
            "left_idx": left_idx,
            "right_idx": right_idx,
            "extend_dist": extend_dist
        }
        self._publish_to_topic(
            "/llm_action",
            "std_msgs/String",
            {"data": json.dumps(cmd)}
        )
        return {
            "status": "command_sent",
            "action": "pass_between_points",
            "left_idx": left_idx,
            "right_idx": right_idx,
            "extend_dist": extend_dist,
            "description": f"LiDAR idx[{left_idx}] ↔ idx[{right_idx}] 사이 직선 통과 시작"
        }

    def _find_obstacles_in_range(self, angle_min: int, angle_max: int, max_range: float = 30.0) -> dict:
        """특정 각도 범위에서 가장 가까운 LiDAR 점 찾기

        LiDAR 각도 규칙:
        - 0° = 전방, 90° = 우측, 180° = 후방, 270° = 좌측
        - 스캔 인덱스 = 각도 (360개 점 기준)
        """
        status = self._get_boat_status()
        if "error" in status:
            return status

        # clusters에서 해당 각도 범위의 물체 찾기
        clusters = status.get("clusters", [])
        found = []

        for i, c in enumerate(clusters):
            angle = c.get("center_angle", 0)
            dist = c.get("center_dist", 999)

            # 각도 범위 체크 (270-360 범위 처리)
            in_range = False
            if angle_min <= angle_max:
                in_range = angle_min <= angle <= angle_max
            else:  # 예: 270-45 (270~360, 0~45)
                in_range = angle >= angle_min or angle <= angle_max

            if in_range and dist <= max_range:
                found.append({
                    "cluster_id": i,
                    "scan_idx": int(angle),  # 각도 ≈ 스캔 인덱스
                    "angle": round(angle, 1),
                    "distance": round(dist, 2)
                })

        # 거리순 정렬
        found.sort(key=lambda x: x["distance"])

        return {
            "angle_range": f"{angle_min}°-{angle_max}°",
            "max_range": max_range,
            "found_count": len(found),
            "obstacles": found,
            "nearest": found[0] if found else None,
            "hint": "navigate_between_points(left_idx, right_idx)에 scan_idx 사용"
        }

    # ============================================================
    # Chat Interface
    # ============================================================

    def chat(self, user_message: str, max_iterations: int = 10, callback: Optional[Callable] = None) -> str:
        """사용자 메시지 처리 (function calling 루프)

        Args:
            user_message: 사용자 메시지
            max_iterations: 최대 반복 횟수
            callback: function call 콜백 (name, args, result)
        """
        if self.chat_session is None:
            self.chat_session = self.model.start_chat()

        response = self.chat_session.send_message(user_message)

        # Function calling 루프
        for _ in range(max_iterations):
            # 응답에서 parts 확인
            parts = response.candidates[0].content.parts

            # 텍스트 응답 확인
            text_response = None
            function_calls = []

            for part in parts:
                # 텍스트가 있으면 저장
                if hasattr(part, 'text') and part.text:
                    text_response = part.text
                # function_call이 있으면 수집
                if hasattr(part, 'function_call') and part.function_call:
                    fc = part.function_call
                    function_calls.append({
                        "name": fc.name,
                        "args": dict(fc.args) if fc.args else {}
                    })

            # function call이 없으면 텍스트 반환
            if not function_calls:
                return text_response or str(response)

            # 모든 function call 실행
            from google.generativeai.types import content_types
            function_responses = []
            for fc in function_calls:
                result = self._execute_function(fc["name"], fc["args"])
                print(f"[GeminiMCP] {fc['name']}({fc['args']}) -> {json.dumps(result, ensure_ascii=False)[:200]}")

                # 콜백 호출
                if callback:
                    callback(fc["name"], fc["args"], result)

                function_responses.append(
                    content_types.to_part({
                        "function_response": {
                            "name": fc["name"],
                            "response": {"result": result}
                        }
                    })
                )

            # 결과를 Gemini에 전달
            response = self.chat_session.send_message(function_responses)

        # 최종 텍스트 추출
        try:
            for part in response.candidates[0].content.parts:
                if hasattr(part, 'text') and part.text:
                    return part.text
            return str(response)
        except:
            return str(response)

    def reset_chat(self):
        """대화 세션 초기화"""
        self.chat_session = None

    # ============================================================
    # Autonomous Loop
    # ============================================================

    def run_autonomous_loop(
        self,
        interval: float = 0.5,
        mission: str = "gate_search",
        on_decision: Optional[Callable[[str], None]] = None
    ):
        """자율 판단 루프 실행"""
        self._autonomous_running = True
        self.reset_chat()

        prompt = f"""자율 모드를 시작합니다. 현재 미션: {mission}

반복적으로 다음을 수행하세요:
1. get_boat_status()로 현재 상태 확인
2. 상황 분석 및 다음 행동 결정
3. send_action_command()로 명령 실행
4. 결과 보고

미션 목표:
- gate_search: 빨간/초록 게이트를 찾아 통과
- buoy_orbit: 빨간 부표 주변 시계방향 선회
- hopping_tour: 웨이포인트 순회
- docking: 도킹 스테이션에 정박

현재 상태를 확인하고 첫 번째 행동을 결정하세요."""

        print(f"[GeminiMCP] Autonomous loop started (interval={interval}s, mission={mission})")

        while self._autonomous_running:
            try:
                start_time = time.time()

                # Gemini에게 판단 요청
                response = self.chat(prompt if self.chat_session is None else "다음 행동을 결정하세요.")

                if on_decision:
                    on_decision(response)
                else:
                    print(f"[GeminiMCP] Decision: {response[:200]}...")

                # 간격 유지
                elapsed = time.time() - start_time
                if elapsed < interval:
                    time.sleep(interval - elapsed)

            except KeyboardInterrupt:
                print("[GeminiMCP] Autonomous loop interrupted")
                break
            except Exception as e:
                print(f"[GeminiMCP] Error in loop: {e}")
                time.sleep(1)

        self._autonomous_running = False
        print("[GeminiMCP] Autonomous loop stopped")

    def stop_autonomous(self):
        """자율 루프 중지"""
        self._autonomous_running = False


# ============================================================
# CLI / Test
# ============================================================

def main():
    """테스트 실행"""
    import argparse
    parser = argparse.ArgumentParser(description="Gemini MCP Bridge")
    parser.add_argument("--ip", default="127.0.0.1", help="rosbridge IP")
    parser.add_argument("--port", type=int, default=9090, help="rosbridge port")
    parser.add_argument("--autonomous", action="store_true", help="Run autonomous loop")
    parser.add_argument("--mission", default="gate_search", help="Mission name")
    parser.add_argument("--interval", type=float, default=0.5, help="Loop interval")
    args = parser.parse_args()

    bridge = GeminiMCPBridge()

    if not bridge.connect(args.ip, args.port):
        print("Failed to connect to rosbridge")
        return

    try:
        if args.autonomous:
            bridge.run_autonomous_loop(
                interval=args.interval,
                mission=args.mission
            )
        else:
            # 대화 모드
            print("Gemini MCP Bridge ready. Type 'quit' to exit.")
            while True:
                user_input = input("\nYou: ").strip()
                if user_input.lower() in ('quit', 'exit', 'q'):
                    break
                if not user_input:
                    continue

                response = bridge.chat(user_input)
                print(f"\nGemini: {response}")
    finally:
        bridge.disconnect()


if __name__ == "__main__":
    main()
