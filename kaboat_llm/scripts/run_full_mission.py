#!/usr/bin/env python3
"""
KABOAT 풀 미션 실행기
전체 미션을 순차적으로 실행: 게이트 → 부표 선회 → 호핑 투어 → 도킹
"""

import os
import sys
import json
import time
from datetime import datetime

import google.generativeai as genai

try:
    import roslibpy
except ImportError:
    print("[ERROR] roslibpy 필요: pip install roslibpy")
    sys.exit(1)

# KABOAT 미션 프롬프트
sys.path.insert(0, '/home/yune/vrx_ws/src/kaboat_autonomous')
try:
    from kaboat_autonomous.mission_prompt import (
        generate_full_prompt,
        format_status_for_llm,
    )
    MISSION_PROMPT_AVAILABLE = True
except ImportError:
    MISSION_PROMPT_AVAILABLE = False
    print("[WARN] mission_prompt.py 로드 실패, 기본 프롬프트 사용")


# 설정
GOOGLE_API_KEY = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
ROSBRIDGE_HOST = os.environ.get("ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.environ.get("ROSBRIDGE_PORT", 9090))

# 풀 미션 순서
FULL_MISSION_SEQUENCE = [
    {
        "type": "gate_search",
        "name": "1단계: 게이트 탐색/통과",
        "params": {},
    },
    {
        "type": "buoy_orbit",
        "name": "2단계: 부표 선회",
        "params": {"color": "red", "direction": "cw"},
    },
    {
        "type": "hopping_tour",
        "name": "3단계: 호핑 투어",
        "params": {},
    },
    {
        "type": "docking",
        "name": "4단계: 도킹",
        "params": {"color": "blue"},
    },
]


class FullMissionRunner:
    """풀 미션 순차 실행기"""

    def __init__(self, model_name: str = "gemini-flash-lite-latest"):
        self.model_name = model_name
        self.current_mission_idx = 0
        self.current_mission = None
        self.model = None

        self.boat_status = None
        self.lidar_summary = {}
        self.decision_needed = False
        self.current_action = None
        self.last_action_result = None
        self.mission_phase_done = False

        self.running = False
        self.decision_interval = 2.0
        self.last_decision_time = 0
        self.action_history = []

        # Gemini
        if not GOOGLE_API_KEY:
            raise ValueError("GOOGLE_API_KEY 환경변수 필요")
        genai.configure(api_key=GOOGLE_API_KEY)

        # ROS
        self.ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
        self.status_sub = roslibpy.Topic(self.ros, '/boat_status', 'std_msgs/String')
        self.sensor_sub = roslibpy.Topic(self.ros, '/sensor_fusion', 'std_msgs/String')
        self.action_pub = roslibpy.Topic(self.ros, '/llm_action', 'std_msgs/String')

    def connect(self):
        print(f"[INFO] rosbridge 연결 중... {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}")
        self.ros.run()
        if not self.ros.is_connected:
            raise ConnectionError("rosbridge 연결 실패")
        print("[OK] rosbridge 연결됨")

        self.status_sub.subscribe(self._status_callback)
        self.sensor_sub.subscribe(self._sensor_callback)

    def _status_callback(self, msg):
        try:
            self.boat_status = json.loads(msg['data'])
            obs = self.boat_status.get('obstacles', {})
            self.lidar_summary = {
                'front_clear': obs.get('front', 999) > 10,
                'closest_obstacle': {'distance': obs.get('closest', 999), 'angle': 0},
            }
        except:
            pass

    def _sensor_callback(self, msg):
        try:
            data = json.loads(msg['data'])
            self.decision_needed = data.get('decision_needed', False)
            self.current_action = data.get('current_action')
            self.last_action_result = data.get('last_action_result')

            # mission_phase_done 감지
            if self.last_action_result == 'mission_phase_done':
                self.mission_phase_done = True
        except:
            pass

    def _setup_mission(self, mission_config: dict):
        """새 미션 설정"""
        mission_type = mission_config['type']
        mission_params = mission_config.get('params', {})

        print(f"\n{'='*60}")
        print(f"  {mission_config['name']}")
        print(f"  타입: {mission_type}, 파라미터: {mission_params}")
        print(f"{'='*60}\n")

        # 시스템 프롬프트 생성
        if MISSION_PROMPT_AVAILABLE:
            system_prompt = generate_full_prompt(mission_type, mission_params)
        else:
            system_prompt = f"당신은 KABOAT 자율주행 선박 AI입니다. 현재 미션: {mission_type}"

        self.model = genai.GenerativeModel(
            model_name=self.model_name,
            system_instruction=system_prompt,
            generation_config={"temperature": 0.1, "max_output_tokens": 512},
        )

        self.current_mission = mission_config
        self.mission_phase_done = False
        self.action_history = []

    def _format_status(self) -> str:
        if not self.boat_status:
            return "상태 정보 없음"

        if MISSION_PROMPT_AVAILABLE and self.lidar_summary:
            return format_status_for_llm(self.boat_status, self.lidar_summary)

        pos = self.boat_status.get('position', {})
        obs = self.boat_status.get('obstacles', {})
        return f"위치: ({pos.get('x',0):.1f}, {pos.get('y',0):.1f}), 헤딩: {pos.get('heading_deg',0):.1f}°, 전방: {obs.get('front',999):.1f}m"

    def _ask_llm(self) -> dict:
        prompt = f"""현재 상태:
{self._format_status()}

최근 행동: {self.action_history[-3:] if self.action_history else "없음"}
이전 결과: {self.last_action_result or "없음"}

다음 행동을 JSON으로 출력하세요:"""

        start = time.time()
        response = self.model.generate_content(prompt)
        latency = (time.time() - start) * 1000

        text = response.text.strip()
        try:
            if "```" in text:
                text = text.split("```")[1]
                if text.startswith("json"):
                    text = text[4:]
            result = json.loads(text.strip())
        except:
            result = {"action": "hold", "duration": 1}

        print(f"  [LLM] {latency:.0f}ms → {result.get('action', result)}")
        return result

    def _execute_action(self, action: dict):
        action_type = action.get("action", "")
        self.action_history.append(action_type)

        msg = {'data': json.dumps(action)}
        self.action_pub.publish(roslibpy.Message(msg))
        print(f"  [ACTION] {action_type}")

        # 미션 완료 신호 감지
        if action_type in ("mission_phase_done", "mission_complete"):
            self.mission_phase_done = True

    def run(self):
        """풀 미션 실행"""
        print("\n" + "=" * 60)
        print("   KABOAT 풀 미션 시작")
        print("   미션 순서: 게이트 → 부표 → 호핑 → 도킹")
        print("=" * 60 + "\n")

        self.running = True
        self.current_mission_idx = 0

        try:
            while self.running and self.current_mission_idx < len(FULL_MISSION_SEQUENCE):
                # 현재 미션 설정
                mission = FULL_MISSION_SEQUENCE[self.current_mission_idx]
                self._setup_mission(mission)

                # 미션 루프
                loop_count = 0
                while self.running and not self.mission_phase_done:
                    loop_count += 1

                    if not self.boat_status:
                        print("  [WAIT] 상태 수신 대기...")
                        time.sleep(1)
                        continue

                    now = time.time()
                    if now - self.last_decision_time >= self.decision_interval:
                        self.last_decision_time = now

                        if self.decision_needed:
                            print(f"  [Loop {loop_count}] decision_needed=True")
                            action = self._ask_llm()
                            self._execute_action(action)
                        else:
                            if loop_count % 10 == 0:
                                print(f"  [Loop {loop_count}] 대기 중... action={self.current_action}")

                    time.sleep(0.5)

                # 미션 완료
                print(f"\n✅ {mission['name']} 완료!")
                self.current_mission_idx += 1
                time.sleep(2)  # 다음 미션 전 잠시 대기

            print("\n" + "=" * 60)
            print("   🎉 풀 미션 완료!")
            print("=" * 60)

        except KeyboardInterrupt:
            print("\n[STOP] 사용자 중단")
        finally:
            self.running = False

    def disconnect(self):
        self.status_sub.unsubscribe()
        self.ros.terminate()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="KABOAT 풀 미션")
    parser.add_argument("--model", default="gemini-flash-lite-latest")
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--start-from", type=int, default=0, help="시작 미션 인덱스 (0=gate, 1=buoy, ...)")
    args = parser.parse_args()

    runner = FullMissionRunner(args.model)
    runner.decision_interval = args.interval
    runner.current_mission_idx = args.start_from

    try:
        runner.connect()
        runner.run()
    finally:
        runner.disconnect()


if __name__ == "__main__":
    main()
