#!/usr/bin/env python3
"""
VRX 시뮬레이션에서 Claude 판단 데이터 수집

1. VRX에서 실시간 센서 데이터 캡처
2. Claude에게 상황 전달 + 판단 요청
3. Claude의 추론 + Tool Call을 학습 데이터로 저장
"""

import json
import time
import os
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Any
import anthropic

# ROS2는 선택적 import (시뮬레이션 연결 시에만)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan, NavSatFix, Imu, Image
    from std_msgs.msg import Float32MultiArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARN] ROS2 not available. Running in offline mode.")


@dataclass
class SensorState:
    """센서 상태 스냅샷"""
    timestamp: str

    # GPS
    gps_lat: float = 0.0
    gps_lon: float = 0.0

    # IMU
    heading: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0

    # LiDAR (요약)
    lidar_front: float = -1.0
    lidar_left: float = -1.0
    lidar_right: float = -1.0
    lidar_back: float = -1.0
    obstacles_detected: int = 0

    # 로컬 좌표 (계산됨)
    local_x: float = 0.0
    local_y: float = 0.0

    # 미션 상태
    mission_type: str = "free_navigation"
    mission_progress: float = 0.0
    current_waypoint: Optional[Dict] = None


@dataclass
class TrainingSample:
    """학습 데이터 샘플"""
    id: str
    timestamp: str

    # 입력
    sensor_state: Dict
    vision_description: str  # 카메라 상황 설명 (수동 또는 VLM)
    user_command: str

    # Claude 출력
    claude_reasoning: str
    claude_tool_call: Dict

    # 메타데이터
    scenario_type: str
    quality_score: Optional[float] = None


class VRXDataCollector:
    """VRX 시뮬레이션 데이터 수집기"""

    def __init__(self):
        self.sensor_state = SensorState(timestamp=self._now())
        self.node = None

        # 기준점 (Sydney Regatta)
        self.ref_lat = -33.7227
        self.ref_lon = 150.6740

    def _now(self) -> str:
        return datetime.now().isoformat()

    def connect_ros(self):
        """ROS2 연결"""
        if not ROS_AVAILABLE:
            print("[ERROR] ROS2 not available")
            return False

        rclpy.init()
        self.node = rclpy.create_node('data_collector')

        # Subscribers
        self.node.create_subscription(
            NavSatFix, '/wamv/sensors/gps/fix',
            self._gps_callback, 10)
        self.node.create_subscription(
            Imu, '/wamv/sensors/imu/data',
            self._imu_callback, 10)
        self.node.create_subscription(
            LaserScan, '/wamv/sensors/lidar/scan',
            self._lidar_callback, 10)

        print("[OK] ROS2 connected")
        return True

    def _gps_callback(self, msg: 'NavSatFix'):
        self.sensor_state.gps_lat = msg.latitude
        self.sensor_state.gps_lon = msg.longitude
        # 로컬 좌표 계산 (간단한 근사)
        self.sensor_state.local_x = (msg.longitude - self.ref_lon) * 111000 * 0.83  # cos(lat)
        self.sensor_state.local_y = (msg.latitude - self.ref_lat) * 111000

    def _imu_callback(self, msg: 'Imu'):
        import math
        q = msg.orientation
        # Quaternion to Euler
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.sensor_state.heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _lidar_callback(self, msg: 'LaserScan'):
        import numpy as np
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0)
        ranges[ranges > 50] = 0  # max range

        n = len(ranges)
        # 방향별 최소 거리 (전방=0, 좌측=90, 후방=180, 우측=270)
        def min_sector(start_deg, end_deg):
            start_idx = int(start_deg * n / 360) % n
            end_idx = int(end_deg * n / 360) % n
            if start_idx < end_idx:
                sector = ranges[start_idx:end_idx]
            else:
                sector = np.concatenate([ranges[start_idx:], ranges[:end_idx]])
            valid = sector[sector > 0]
            return float(np.min(valid)) if len(valid) > 0 else -1.0

        self.sensor_state.lidar_front = min_sector(-30, 30)
        self.sensor_state.lidar_left = min_sector(60, 120)
        self.sensor_state.lidar_back = min_sector(150, 210)
        self.sensor_state.lidar_right = min_sector(240, 300)
        self.sensor_state.obstacles_detected = int(np.sum(ranges > 0))

    def spin_once(self):
        """ROS 메시지 한 번 처리"""
        if self.node:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self.sensor_state.timestamp = self._now()

    def get_state(self) -> Dict:
        """현재 센서 상태 반환"""
        return asdict(self.sensor_state)

    def shutdown(self):
        if self.node:
            self.node.destroy_node()
            rclpy.shutdown()


class ClaudeTeacher:
    """Claude 교사 모델"""

    SYSTEM_PROMPT = """당신은 KABOAT 자율주행 선박의 AI 판단 시스템입니다.

## 역할
주어진 센서 상황과 사용자 명령을 분석하여, 적절한 ROS 2 MCP Tool Call을 생성합니다.

## 사용 가능한 도구
- kaboat_navigate_to: GPS/로컬 좌표로 이동
- kaboat_navigate_between: 두 물체 사이 통과
- kaboat_turn: 방향 전환
- kaboat_stop: 정지
- kaboat_emergency_stop: 긴급 정지
- kaboat_avoid_obstacle: 장애물 회피
- kaboat_orbit_buoy: 부표 선회
- kaboat_hold_position: 위치 유지
- kaboat_dock: 도킹 수행
- kaboat_search_pattern: 탐색 패턴 실행
- kaboat_report_observation: 관측 보고
- kaboat_get_status: 상태 조회
- kaboat_set_speed: 속도 설정
- kaboat_return_home: 귀환

## 출력 형식
반드시 아래 JSON 형식으로만 출력하세요:
```json
{
  "reasoning": "상황 분석 및 판단 근거 (2-3문장)",
  "tool": "tool_name",
  "parameters": {
    "param1": "value1"
  }
}
```

## 안전 원칙
1. 전방 3m 이내 장애물 → 즉시 emergency_stop
2. 센서 이상 시 → 저속 + 안전 모드
3. 불확실한 상황 → 확인 후 행동
"""

    def __init__(self, api_key: Optional[str] = None):
        self.client = anthropic.Anthropic(
            api_key=api_key or os.environ.get("ANTHROPIC_API_KEY")
        )

    def ask(self, sensor_state: Dict, vision_description: str, user_command: str) -> Dict:
        """Claude에게 판단 요청"""

        user_message = f"""## 현재 상황

### 센서 데이터
- 위치: ({sensor_state.get('local_x', 0):.1f}, {sensor_state.get('local_y', 0):.1f})
- 헤딩: {sensor_state.get('heading', 0):.1f}°
- LiDAR:
  - 전방: {sensor_state.get('lidar_front', -1):.1f}m
  - 좌측: {sensor_state.get('lidar_left', -1):.1f}m
  - 우측: {sensor_state.get('lidar_right', -1):.1f}m
  - 후방: {sensor_state.get('lidar_back', -1):.1f}m
  - 장애물 수: {sensor_state.get('obstacles_detected', 0)}개
- GPS: ({sensor_state.get('gps_lat', 0):.6f}, {sensor_state.get('gps_lon', 0):.6f})

### 시각 정보
{vision_description}

### 미션 상태
- 타입: {sensor_state.get('mission_type', 'free_navigation')}
- 진행률: {sensor_state.get('mission_progress', 0)*100:.0f}%

### 사용자 명령
"{user_command}"

위 상황에서 어떤 행동을 취해야 하는지 판단하세요."""

        response = self.client.messages.create(
            model="claude-sonnet-4-20250514",
            max_tokens=1024,
            system=self.SYSTEM_PROMPT,
            messages=[{"role": "user", "content": user_message}]
        )

        # 응답 파싱
        text = response.content[0].text

        # JSON 추출
        try:
            import re
            json_match = re.search(r'```json\s*(.*?)\s*```', text, re.DOTALL)
            if json_match:
                result = json.loads(json_match.group(1))
            else:
                # JSON 블록 없이 바로 JSON인 경우
                result = json.loads(text)
        except json.JSONDecodeError:
            result = {
                "reasoning": text,
                "tool": "kaboat_get_status",
                "parameters": {}
            }

        return result


class DatasetManager:
    """데이터셋 관리"""

    def __init__(self, output_dir: str = "data/raw"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.samples: List[TrainingSample] = []
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")

    def add_sample(self, sample: TrainingSample):
        """샘플 추가"""
        self.samples.append(sample)

        # 즉시 저장 (안전)
        self._save_sample(sample)

    def _save_sample(self, sample: TrainingSample):
        """개별 샘플 저장"""
        filename = self.output_dir / f"{self.session_id}_{sample.id}.json"
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(asdict(sample), f, ensure_ascii=False, indent=2)

    def export_jsonl(self, output_path: str):
        """JSONL 형식으로 내보내기"""
        with open(output_path, 'w', encoding='utf-8') as f:
            for sample in self.samples:
                f.write(json.dumps(asdict(sample), ensure_ascii=False) + '\n')
        print(f"[OK] Exported {len(self.samples)} samples to {output_path}")


def collect_interactive():
    """대화형 데이터 수집 (ROS 연결 없이 수동 입력)"""

    print("=" * 60)
    print("KABOAT 학습 데이터 수집 (대화형 모드)")
    print("=" * 60)

    teacher = ClaudeTeacher()
    dataset = DatasetManager(output_dir="kaboat_llm/data/raw")

    sample_count = 0

    while True:
        print(f"\n--- 샘플 #{sample_count + 1} ---")

        # 센서 상태 입력 (간단 버전)
        print("\n[센서 상태 입력]")
        try:
            local_x = float(input("  local_x (예: -520): ") or "-520")
            local_y = float(input("  local_y (예: 180): ") or "180")
            heading = float(input("  heading (예: 45): ") or "45")
            lidar_front = float(input("  lidar_front (예: 15): ") or "15")
            lidar_left = float(input("  lidar_left (예: 20): ") or "20")
            lidar_right = float(input("  lidar_right (예: 25): ") or "25")
        except ValueError:
            print("[ERROR] 잘못된 입력")
            continue

        sensor_state = {
            "local_x": local_x,
            "local_y": local_y,
            "heading": heading,
            "lidar_front": lidar_front,
            "lidar_left": lidar_left,
            "lidar_right": lidar_right,
            "lidar_back": -1,
            "obstacles_detected": 0,
            "gps_lat": -33.7227,
            "gps_lon": 150.6740,
            "mission_type": "free_navigation",
            "mission_progress": 0.0,
        }

        # 비전 설명
        vision = input("\n[비전 설명] (예: 전방 10m에 빨간 부표): ") or "특이사항 없음"

        # 사용자 명령
        command = input("[사용자 명령] (예: 빨간 부표로 가): ") or "상태 확인"

        # 시나리오 타입
        scenario = input("[시나리오 타입] (nav/obs/buoy/dock/exc/safe): ") or "nav"

        # Claude에게 질문
        print("\n[Claude 판단 중...]")
        try:
            result = teacher.ask(sensor_state, vision, command)

            print(f"\n[Claude 응답]")
            print(f"  reasoning: {result.get('reasoning', 'N/A')}")
            print(f"  tool: {result.get('tool', 'N/A')}")
            print(f"  parameters: {result.get('parameters', {})}")

            # 저장 확인
            save = input("\n저장할까요? (y/n/q): ").lower()

            if save == 'q':
                break
            elif save == 'y':
                sample = TrainingSample(
                    id=f"sample_{sample_count:04d}",
                    timestamp=datetime.now().isoformat(),
                    sensor_state=sensor_state,
                    vision_description=vision,
                    user_command=command,
                    claude_reasoning=result.get('reasoning', ''),
                    claude_tool_call={
                        "tool": result.get('tool', ''),
                        "parameters": result.get('parameters', {})
                    },
                    scenario_type=scenario
                )
                dataset.add_sample(sample)
                sample_count += 1
                print(f"[OK] 샘플 저장됨 (총 {sample_count}개)")

        except Exception as e:
            print(f"[ERROR] Claude API 오류: {e}")
            continue

    # 세션 종료
    if sample_count > 0:
        export_path = f"kaboat_llm/data/processed/train_{dataset.session_id}.jsonl"
        dataset.export_jsonl(export_path)

    print(f"\n수집 완료: {sample_count}개 샘플")


def collect_from_vrx():
    """VRX 시뮬레이션에서 실시간 데이터 수집"""

    if not ROS_AVAILABLE:
        print("[ERROR] ROS2 필요. 대화형 모드 사용: python collect_training_data.py --interactive")
        return

    print("=" * 60)
    print("KABOAT 학습 데이터 수집 (VRX 실시간 모드)")
    print("=" * 60)

    collector = VRXDataCollector()
    if not collector.connect_ros():
        return

    teacher = ClaudeTeacher()
    dataset = DatasetManager(output_dir="kaboat_llm/data/raw")

    sample_count = 0

    try:
        while True:
            # 센서 업데이트
            for _ in range(10):  # 1초간 데이터 수집
                collector.spin_once()
                time.sleep(0.1)

            sensor_state = collector.get_state()

            print(f"\n--- 현재 상태 ---")
            print(f"  위치: ({sensor_state['local_x']:.1f}, {sensor_state['local_y']:.1f})")
            print(f"  헤딩: {sensor_state['heading']:.1f}°")
            print(f"  LiDAR: F={sensor_state['lidar_front']:.1f} L={sensor_state['lidar_left']:.1f} R={sensor_state['lidar_right']:.1f}")

            # 사용자 입력
            action = input("\n[Enter: 스킵 / c: 캡처 / q: 종료]: ").lower()

            if action == 'q':
                break
            elif action == 'c':
                vision = input("[비전 설명]: ") or "특이사항 없음"
                command = input("[사용자 명령]: ") or "상태 확인"
                scenario = input("[시나리오 타입]: ") or "nav"

                print("\n[Claude 판단 중...]")
                result = teacher.ask(sensor_state, vision, command)

                print(f"  reasoning: {result.get('reasoning', 'N/A')}")
                print(f"  tool: {result.get('tool', 'N/A')}")

                if input("저장? (y/n): ").lower() == 'y':
                    sample = TrainingSample(
                        id=f"vrx_{sample_count:04d}",
                        timestamp=datetime.now().isoformat(),
                        sensor_state=sensor_state,
                        vision_description=vision,
                        user_command=command,
                        claude_reasoning=result.get('reasoning', ''),
                        claude_tool_call={
                            "tool": result.get('tool', ''),
                            "parameters": result.get('parameters', {})
                        },
                        scenario_type=scenario
                    )
                    dataset.add_sample(sample)
                    sample_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        collector.shutdown()

    if sample_count > 0:
        export_path = f"kaboat_llm/data/processed/vrx_{dataset.session_id}.jsonl"
        dataset.export_jsonl(export_path)

    print(f"\n수집 완료: {sample_count}개 샘플")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="KABOAT 학습 데이터 수집")
    parser.add_argument("--interactive", "-i", action="store_true",
                       help="대화형 모드 (ROS 없이)")
    parser.add_argument("--vrx", "-v", action="store_true",
                       help="VRX 실시간 모드")

    args = parser.parse_args()

    if args.interactive:
        collect_interactive()
    elif args.vrx:
        collect_from_vrx()
    else:
        # 기본: 대화형
        collect_interactive()
