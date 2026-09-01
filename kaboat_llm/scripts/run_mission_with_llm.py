#!/usr/bin/env python3
"""
KABOAT 미션 실행 (LLM 판단 루프)

1. /boat_status 구독하여 현재 상태 수신
2. Gemini Flash에게 상황 전달 + 판단 요청
3. LLM 출력(JSON Tool Call)을 ROS 토픽으로 발행
4. 반복

사용법:
1. VRX 시뮬레이션 실행
2. rosbridge 실행: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
3. llm_interface 실행: ros2 run kaboat_autonomous llm_interface
4. mission_runner 실행: ros2 run kaboat_autonomous mission_runner
5. 이 스크립트 실행: python run_mission_with_llm.py --mission gate_navigation
"""

import os
import sys
import json
import time
import argparse
from datetime import datetime
import google.generativeai as genai

# rosbridge 클라이언트
try:
    import roslibpy
except ImportError:
    print("[ERROR] roslibpy 필요: pip install roslibpy")
    sys.exit(1)

# KABOAT 미션 프롬프트 (기존 구현 사용)
sys.path.insert(0, '/home/yune/vrx_ws/src/kaboat_autonomous')
try:
    from kaboat_autonomous.mission_prompt import (
        generate_full_prompt,
        format_status_for_llm,
        format_sensor_data_for_llm,
    )
    MISSION_PROMPT_AVAILABLE = True
    print("[OK] mission_prompt.py 로드 완료")
except ImportError as e:
    print(f"[WARN] mission_prompt.py 로드 실패: {e}")
    MISSION_PROMPT_AVAILABLE = False


# === 설정 ===
GOOGLE_API_KEY = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
ROSBRIDGE_HOST = os.environ.get("ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.environ.get("ROSBRIDGE_PORT", 9090))


class KABOATMissionRunner:
    """LLM 기반 미션 실행기"""

    def __init__(self, mission_type: str, model_name: str = "gemini-flash-lite-latest",
                 mission_params: dict = None):
        self.mission_type = mission_type
        self.model_name = model_name
        self.mission_params = mission_params or {}
        self.boat_status = None
        self.lidar_summary = {}
        self.running = False
        self.decision_interval = 2.0  # LLM 판단 주기 (초)
        self.decision_needed = False
        self.current_action = None
        self.last_action_result = None

        # Gemini 설정
        if not GOOGLE_API_KEY:
            raise ValueError("GOOGLE_API_KEY 환경변수 필요")
        genai.configure(api_key=GOOGLE_API_KEY)

        # 시스템 프롬프트 생성 (mission_prompt.py 사용)
        if MISSION_PROMPT_AVAILABLE:
            system_prompt = generate_full_prompt(mission_type, mission_params)
            print(f"[OK] 시스템 프롬프트 생성 ({len(system_prompt)} chars)")
        else:
            system_prompt = f"당신은 KABOAT 자율주행 선박 AI입니다. 미션: {mission_type}"
            print("[WARN] 기본 프롬프트 사용")

        self.model = genai.GenerativeModel(
            model_name=model_name,
            system_instruction=system_prompt,
            generation_config={
                "temperature": 0.1,
                "max_output_tokens": 512,
            }
        )

        # ROS 연결
        self.ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)

        # 토픽
        self.status_sub = roslibpy.Topic(self.ros, '/boat_status', 'std_msgs/String')
        self.sensor_sub = roslibpy.Topic(self.ros, '/sensor_fusion', 'std_msgs/String')
        self.action_pub = roslibpy.Topic(self.ros, '/llm_action', 'std_msgs/String')  # action_dispatcher용
        self.waypoint_pub = roslibpy.Topic(self.ros, '/llm_waypoint', 'geometry_msgs/PointStamped')
        self.command_pub = roslibpy.Topic(self.ros, '/llm_command', 'std_msgs/Float32MultiArray')

        # 미션 상태
        self.current_waypoint_idx = 0
        self.mission_complete = False
        self.last_decision_time = 0
        self.action_history = []

    def connect(self):
        """ROS 연결"""
        print(f"[INFO] Connecting to rosbridge at {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}...")
        self.ros.run()

        if not self.ros.is_connected:
            raise ConnectionError("rosbridge 연결 실패")

        print("[OK] Connected to rosbridge")

        # 상태 구독
        self.status_sub.subscribe(self._status_callback)
        self.sensor_sub.subscribe(self._sensor_callback)
        print("[OK] Subscribed to /boat_status, /sensor_fusion")

    def _status_callback(self, msg):
        """보트 상태 수신 (/boat_status)"""
        try:
            self.boat_status = json.loads(msg['data'])
            # obstacles를 lidar_summary로 변환
            obs = self.boat_status.get('obstacles', {})
            self.lidar_summary = {
                'front_clear': obs.get('front', 999) > 10,
                'closest_obstacle': {
                    'distance': obs.get('closest', 999),
                    'angle': 0
                },
                'front_distribution': [
                    {'sector': '정면', 'distance': obs.get('front')},
                    {'sector': '좌측 전방', 'distance': obs.get('front_left')},
                    {'sector': '우측 전방', 'distance': obs.get('front_right')},
                    {'sector': '좌현', 'distance': obs.get('left')},
                    {'sector': '우현', 'distance': obs.get('right')},
                ],
                'clusters': [],
            }
        except:
            pass

    def _sensor_callback(self, msg):
        """센서 융합 데이터 수신 (/sensor_fusion)"""
        try:
            data = json.loads(msg['data'])
            self.decision_needed = data.get('decision_needed', False)
            self.current_action = data.get('current_action')
            self.last_action_result = data.get('last_action_result')

            # lidar_summary 업데이트 (더 상세한 정보)
            if 'lidar_summary' in data:
                self.lidar_summary.update(data['lidar_summary'])

            # 디버그: decision_needed 변경 시 출력
            print(f"  [SENSOR] decision_needed={self.decision_needed}, action={self.current_action}")
        except Exception as e:
            print(f"  [SENSOR ERROR] {e}")

    def _format_status_for_llm(self) -> str:
        """LLM에게 전달할 상태 문자열"""
        if not self.boat_status:
            return "상태 정보 없음"

        # mission_prompt.py의 format_status_for_llm 사용
        if MISSION_PROMPT_AVAILABLE and self.lidar_summary:
            return format_status_for_llm(self.boat_status, self.lidar_summary)

        # fallback: 간단한 포맷
        pos = self.boat_status.get('position', {})
        obs = self.boat_status.get('obstacles', {})
        wp = self.boat_status.get('waypoint')
        status = self.boat_status.get('status', {})

        lines = [
            f"위치: ({pos.get('x', 0):.1f}, {pos.get('y', 0):.1f})",
            f"헤딩: {pos.get('heading_deg', 0):.1f}°",
            f"장애물: 전방 {obs.get('front', 999):.1f}m, 좌측 {obs.get('left', 999):.1f}m, 우측 {obs.get('right', 999):.1f}m",
            f"가장 가까운 장애물: {obs.get('closest', 999):.1f}m",
        ]

        if wp:
            lines.append(f"현재 목표: ({wp.get('x')}, {wp.get('y')}) - 거리 {wp.get('distance_m')}m")

        if status.get('is_stuck'):
            lines.append(f"⚠️ STUCK 감지 ({status.get('stuck_duration_s')}초)")

        return "\n".join(lines)

    def _ask_llm(self, additional_context: str = "") -> dict:
        """LLM에게 판단 요청"""
        status_text = self._format_status_for_llm()

        prompt = f"""현재 상태:
{status_text}

최근 행동: {self.action_history[-3:] if self.action_history else "없음"}
{additional_context}

다음 행동을 JSON으로 출력하세요:"""

        start = time.time()
        response = self.model.generate_content(prompt)
        latency = (time.time() - start) * 1000

        text = response.text.strip()

        # JSON 파싱
        try:
            if "```" in text:
                text = text.split("```")[1]
                if text.startswith("json"):
                    text = text[4:]
            result = json.loads(text.strip())
        except:
            result = {"action": "hold", "duration": 1, "error": "JSON 파싱 실패"}

        print(f"  [LLM] {latency:.0f}ms → {result}")
        return result

    def _execute_action(self, action: dict):
        """LLM 결정 실행 - /llm_action으로 JSON 발행"""
        # action 필드가 dict면 그대로, string이면 파싱
        action_data = action.get("action", action)
        if isinstance(action_data, str):
            try:
                action_data = json.loads(action_data)
            except:
                action_data = {"action": action_data}

        action_type = action_data.get("action", "")
        self.action_history.append(action_type)

        # /llm_action 토픽으로 JSON 발행 (action_dispatcher가 처리)
        msg = {'data': json.dumps(action_data)}
        self.action_pub.publish(roslibpy.Message(msg))
        print(f"  [ACTION] Published to /llm_action: {action_type}")

        # 특별 처리
        if action_type == "mission_complete" or action_type == "mission_phase_done":
            print("  [ACTION] ✅ Mission phase complete!")

        elif action_type == "stop":
            print("  [ACTION] ⚠️ STOP")
            # TODO: 회전 구현

    def _publish_waypoint(self, x: float, y: float):
        """웨이포인트 발행"""
        msg = {
            'header': {
                'stamp': {'sec': int(time.time()), 'nanosec': 0},
                'frame_id': 'map'
            },
            'point': {'x': x, 'y': y, 'z': 0.0}
        }
        self.waypoint_pub.publish(roslibpy.Message(msg))
        print(f"  [ACTION] Navigate to ({x}, {y})")

    def _publish_command(self, psi_error: float, thrust: float):
        """직접 명령 발행"""
        msg = {'data': [psi_error, thrust]}
        self.command_pub.publish(roslibpy.Message(msg))

    def run(self, user_command: str = ""):
        """미션 실행 루프"""
        print(f"\n{'='*60}")
        print(f"KABOAT Mission: {self.mission_type}")
        print(f"Model: {self.model_name}")
        print(f"{'='*60}\n")

        self.running = True
        loop_count = 0

        try:
            while self.running and not self.mission_complete:
                loop_count += 1
                print(f"\n[Loop {loop_count}] {datetime.now().strftime('%H:%M:%S')}")

                # 상태 확인
                if not self.boat_status:
                    print("  [WAIT] 상태 수신 대기...")
                    time.sleep(1)
                    continue

                # LLM 판단 (decision_needed가 true일 때만)
                now = time.time()
                if now - self.last_decision_time >= self.decision_interval:
                    self.last_decision_time = now

                    if not self.decision_needed:
                        print(f"  [WAIT] decision_needed=False, action={self.current_action}")
                    else:
                        print(f"  [DECIDE] decision_needed=True, 이전결과={self.last_action_result}")
                        context = f"사용자 명령: {user_command}" if user_command else ""
                        action = self._ask_llm(context)
                        self._execute_action(action)

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n[STOP] 사용자 중단")

        finally:
            self.running = False
            print("\n미션 종료")

    def disconnect(self):
        """연결 해제"""
        self.status_sub.unsubscribe()
        self.ros.terminate()


def main():
    parser = argparse.ArgumentParser(description="KABOAT LLM 미션 실행")
    parser.add_argument("--mission", "-m", default="gate_search",
                       choices=["gate_search", "buoy_orbit", "hopping_tour",
                               "obstacle_course", "docking", "free_navigation"],
                       help="미션 타입 (mission_prompt.py 기준)")
    parser.add_argument("--model", default="gemini-flash-lite-latest",
                       help="Gemini 모델")
    parser.add_argument("--command", "-c", default="",
                       help="사용자 명령")
    parser.add_argument("--interval", "-i", type=float, default=2.0,
                       help="판단 주기 (초)")
    parser.add_argument("--color", default="green",
                       help="부표 색상 (buoy_orbit/docking용)")
    parser.add_argument("--direction", default="cw",
                       choices=["cw", "ccw"],
                       help="선회 방향 (buoy_orbit용)")

    args = parser.parse_args()

    # 미션 파라미터 구성
    mission_params = {
        'color': args.color,
        'direction': args.direction,
    }

    runner = KABOATMissionRunner(args.mission, args.model, mission_params)
    runner.decision_interval = args.interval

    try:
        runner.connect()
        runner.run(args.command)
    finally:
        runner.disconnect()


if __name__ == "__main__":
    main()
