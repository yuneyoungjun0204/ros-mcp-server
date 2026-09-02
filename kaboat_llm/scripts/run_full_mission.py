#!/usr/bin/env python3
"""
KABOAT 풀 미션 실행기 (v2)
- action_dispatcher의 mission_phase.index 모니터링으로 웨이포인트 전환 감지
- Gemini MCP로 주기적 이미지 분석
- 전체 미션 순차 실행: 게이트 → 부표 선회 → 호핑 투어 → 도킹
"""

import os
import sys
import json
import time
import base64
from datetime import datetime
from typing import Optional

try:
    import roslibpy
except ImportError:
    print("[ERROR] roslibpy 필요: pip install roslibpy")
    sys.exit(1)

# Gemini MCP Bridge 임포트
sys.path.insert(0, '/home/yune/ros-mcp-server')
try:
    from kaboat_llm.gemini_mcp_bridge import GeminiMCPBridge
    GEMINI_MCP_AVAILABLE = True
except ImportError:
    GEMINI_MCP_AVAILABLE = False
    print("[WARN] GeminiMCPBridge 로드 실패, 기본 Gemini API 사용")

import google.generativeai as genai

# 설정
GOOGLE_API_KEY = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
ROSBRIDGE_HOST = os.environ.get("ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.environ.get("ROSBRIDGE_PORT", 9090))

# 미션 단계 이름 매핑 (settings.py MISSION_SEQUENCE 기준)
MISSION_PHASE_NAMES = [
    'start',
    'gate_start',
    'gate_end',
    'buoy_orbit',
    'hopping',
    'obstacle_end_dock_start',
]

# 각 미션 단계별 설명
MISSION_DESCRIPTIONS = {
    'start': '시작점 출발',
    'gate_start': '게이트 진입',
    'gate_end': '게이트 통과 완료',
    'buoy_orbit': '부표 선회',
    'hopping': '호핑 투어',
    'obstacle_end_dock_start': '도킹 시작',
}


class FullMissionRunner:
    """풀 미션 실행기 (v2) - action_dispatcher 연동"""

    def __init__(self, use_gemini_mcp: bool = True, image_interval: float = 5.0):
        self.use_gemini_mcp = use_gemini_mcp and GEMINI_MCP_AVAILABLE
        self.image_interval = image_interval  # 이미지 분석 주기 (초)

        # 상태
        self.boat_status = None
        self.action_status = None
        self.current_phase_idx = 0
        self.last_phase_idx = -1
        self.current_image = None
        self.last_image_time = 0

        self.running = False
        self.gemini_bridge: Optional[GeminiMCPBridge] = None

        # Gemini API 설정
        if not GOOGLE_API_KEY:
            raise ValueError("GOOGLE_API_KEY 환경변수 필요")
        genai.configure(api_key=GOOGLE_API_KEY)
        self.vision_model = genai.GenerativeModel('gemini-3.6-flash')

        # ROS 연결
        self.ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)

        # 토픽 구독
        self.boat_status_sub = roslibpy.Topic(self.ros, '/boat_status', 'std_msgs/String')
        self.action_status_sub = roslibpy.Topic(self.ros, '/action_status', 'std_msgs/String')
        self.image_sub = roslibpy.Topic(self.ros, '/wamv/sensors/camera/image_raw/compressed', 'sensor_msgs/CompressedImage')

        # 액션 발행
        self.action_pub = roslibpy.Topic(self.ros, '/llm_action', 'std_msgs/String')

    def connect(self):
        """ROS 및 Gemini MCP 연결"""
        print(f"[INFO] rosbridge 연결 중... {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}", flush=True)
        self.ros.run()
        if not self.ros.is_connected:
            raise ConnectionError("rosbridge 연결 실패")
        print("[OK] rosbridge 연결됨")

        # 토픽 구독
        self.boat_status_sub.subscribe(self._boat_status_callback)
        self.action_status_sub.subscribe(self._action_status_callback)
        self.image_sub.subscribe(self._image_callback)

        # Gemini MCP 연결 (선택)
        if self.use_gemini_mcp:
            try:
                self.gemini_bridge = GeminiMCPBridge()
                self.gemini_bridge.connect(ROSBRIDGE_HOST, ROSBRIDGE_PORT)
                print("[OK] Gemini MCP Bridge 연결됨")
            except Exception as e:
                print(f"[WARN] Gemini MCP 연결 실패: {e}, 기본 Vision API 사용")
                self.gemini_bridge = None

    def _boat_status_callback(self, msg):
        """보트 상태 콜백"""
        try:
            self.boat_status = json.loads(msg['data'])
        except:
            pass

    def _action_status_callback(self, msg):
        """액션 상태 콜백 - 미션 단계 감지"""
        try:
            self.action_status = json.loads(msg['data'])

            # mission_phase에서 현재 인덱스 추출
            mission_phase = self.action_status.get('mission_phase', {})
            self.current_phase_idx = mission_phase.get('index', 0)

        except:
            pass

    def _image_callback(self, msg):
        """이미지 콜백"""
        try:
            self.current_image = msg['data']  # base64 encoded
        except:
            pass

    def _analyze_image(self, prompt: str = None) -> Optional[str]:
        """현재 이미지를 Gemini로 분석"""
        if not self.current_image:
            return None

        if prompt is None:
            phase_name = MISSION_PHASE_NAMES[self.current_phase_idx] if self.current_phase_idx < len(MISSION_PHASE_NAMES) else 'unknown'
            prompt = f"현재 미션: {MISSION_DESCRIPTIONS.get(phase_name, phase_name)}. 전방 카메라 이미지를 분석하고 주요 물체(부표, 게이트, 장애물, 도킹 스테이션)를 식별하세요."

        try:
            # base64 디코딩
            image_data = base64.b64decode(self.current_image)

            start = time.time()
            response = self.vision_model.generate_content([
                prompt,
                {"mime_type": "image/jpeg", "data": image_data}
            ])
            latency = int((time.time() - start) * 1000)

            result = response.text
            print(f"  [Vision] {latency}ms: {result[:100]}...")
            return result

        except Exception as e:
            print(f"  [Vision Error] {e}")
            return None

    def _on_phase_change(self, old_idx: int, new_idx: int):
        """미션 단계 변경 시 호출"""
        old_name = MISSION_PHASE_NAMES[old_idx] if old_idx < len(MISSION_PHASE_NAMES) else 'unknown'
        new_name = MISSION_PHASE_NAMES[new_idx] if new_idx < len(MISSION_PHASE_NAMES) else 'unknown'

        print(f"\n{'='*60}")
        print(f"  ✅ 미션 단계 전환: {old_name} → {new_name}")
        print(f"  {MISSION_DESCRIPTIONS.get(new_name, '')}")
        print(f"{'='*60}\n")

        # 단계 변경 시 이미지 분석
        if self.current_image:
            self._analyze_image(f"새로운 미션 단계 '{new_name}'에 진입했습니다. 전방 상황을 분석하세요.")

    def _send_action(self, action: dict):
        """액션 발행"""
        msg = {'data': json.dumps(action)}
        self.action_pub.publish(roslibpy.Message(msg))
        print(f"  [ACTION] {action.get('action', action)}")

    def run(self):
        """풀 미션 실행"""
        print("\n" + "=" * 60)
        print("   KABOAT 풀 미션 실행기 v2")
        print("   - action_dispatcher 미션 단계 모니터링")
        print("   - Gemini 주기적 이미지 분석")
        print("=" * 60 + "\n")

        self.running = True
        loop_count = 0

        try:
            while self.running:
                loop_count += 1

                # 상태 대기
                if not self.action_status:
                    print("  [WAIT] action_status 수신 대기...")
                    time.sleep(1)
                    continue

                # 미션 단계 변경 감지
                if self.current_phase_idx != self.last_phase_idx:
                    if self.last_phase_idx >= 0:
                        self._on_phase_change(self.last_phase_idx, self.current_phase_idx)
                    self.last_phase_idx = self.current_phase_idx

                # 미션 완료 체크
                mission_phase = self.action_status.get('mission_phase', {})
                total_phases = mission_phase.get('total', 6)
                if self.current_phase_idx >= total_phases:
                    print("\n🎉 풀 미션 완료!")
                    break

                # 주기적 이미지 분석
                now = time.time()
                if now - self.last_image_time >= self.image_interval:
                    self.last_image_time = now
                    if self.current_image:
                        self._analyze_image()

                # 상태 출력 (10 루프마다)
                if loop_count % 20 == 0:
                    pos = self.boat_status.get('position', {}) if self.boat_status else {}
                    phase_name = MISSION_PHASE_NAMES[self.current_phase_idx] if self.current_phase_idx < len(MISSION_PHASE_NAMES) else '?'
                    current_action = self.action_status.get('current_action', 'none')
                    print(f"  [Status] Phase {self.current_phase_idx}/{total_phases} ({phase_name}) | "
                          f"Action: {current_action} | "
                          f"Pos: ({pos.get('x', 0):.1f}, {pos.get('y', 0):.1f})")

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n[STOP] 사용자 중단")
        finally:
            self.running = False

    def disconnect(self):
        """연결 해제"""
        try:
            self.boat_status_sub.unsubscribe()
            self.action_status_sub.unsubscribe()
            self.image_sub.unsubscribe()
        except:
            pass

        if self.gemini_bridge:
            try:
                self.gemini_bridge.disconnect()
            except:
                pass

        self.ros.terminate()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="KABOAT 풀 미션 v2")
    parser.add_argument("--no-mcp", action="store_true", help="Gemini MCP 사용 안 함")
    parser.add_argument("--image-interval", type=float, default=5.0, help="이미지 분석 주기 (초)")
    args = parser.parse_args()

    runner = FullMissionRunner(
        use_gemini_mcp=not args.no_mcp,
        image_interval=args.image_interval
    )

    try:
        runner.connect()
        runner.run()
    finally:
        runner.disconnect()


if __name__ == "__main__":
    main()
