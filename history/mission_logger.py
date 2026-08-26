#!/usr/bin/env python3
"""
KABOAT 미션 로거
- 프롬프트와 LLM 응답 저장
- 카메라 이미지 및 분석 결과 저장
- (선택) GIF 캡처

사용법:
    from mission_logger import MissionLogger
    logger = MissionLogger()
    logger.log_prompt("센서 상태를 확인해줘")
    logger.log_response("현재 위치: (10, 20), 전방 클리어")
    logger.log_image(image_data, analysis="녹색 부표 감지")
    logger.save()
"""
import os
import json
import base64
from datetime import datetime
from typing import Optional, Dict, Any, List
from pathlib import Path


class MissionLogger:
    """미션 대화 및 이미지 로거"""

    def __init__(self, base_dir: str = "/home/yune/ros-mcp-server/history"):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)

        # 세션 폴더 생성 (타임스탬프)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.session_dir = self.base_dir / f"{timestamp}_mission"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        # 이미지 폴더
        self.images_dir = self.session_dir / "images"
        self.images_dir.mkdir(exist_ok=True)

        # 대화 기록
        self.conversation: List[Dict[str, Any]] = []
        self.image_count = 0

        # 메타데이터
        self.metadata = {
            "session_id": timestamp,
            "start_time": datetime.now().isoformat(),
            "end_time": None,
            "total_prompts": 0,
            "total_images": 0,
            "mission_status": "in_progress"
        }

        print(f"[MissionLogger] 세션 시작: {self.session_dir}")

    def log_prompt(self, prompt: str, sensor_data: Optional[Dict] = None):
        """프롬프트 기록"""
        entry = {
            "type": "prompt",
            "timestamp": datetime.now().isoformat(),
            "content": prompt,
            "sensor_data": sensor_data
        }
        self.conversation.append(entry)
        self.metadata["total_prompts"] += 1
        self._auto_save()

    def log_response(self, response: str, action: Optional[Dict] = None):
        """LLM 응답 기록"""
        entry = {
            "type": "response",
            "timestamp": datetime.now().isoformat(),
            "content": response,
            "action": action
        }
        self.conversation.append(entry)
        self._auto_save()

    def log_action(self, action_type: str, params: Dict, result: Optional[str] = None):
        """액션 실행 기록"""
        entry = {
            "type": "action",
            "timestamp": datetime.now().isoformat(),
            "action_type": action_type,
            "params": params,
            "result": result
        }
        self.conversation.append(entry)
        self._auto_save()

    def log_image(self, image_data: bytes,
                  analysis: Optional[str] = None,
                  detections: Optional[List[Dict]] = None,
                  format: str = "jpeg") -> str:
        """
        이미지 저장 및 분석 결과 기록

        Args:
            image_data: 이미지 바이트 데이터
            analysis: LLM 이미지 분석 결과 텍스트
            detections: 감지된 객체 리스트 [{"label": "green_buoy", "x": 320, "y": 240}, ...]
            format: 이미지 포맷 (jpeg, png)

        Returns:
            저장된 이미지 경로
        """
        self.image_count += 1
        img_filename = f"{self.image_count:03d}_raw.{format}"
        img_path = self.images_dir / img_filename

        # 이미지 저장
        with open(img_path, "wb") as f:
            f.write(image_data)

        # 분석 결과 저장
        analysis_data = {
            "image_file": img_filename,
            "timestamp": datetime.now().isoformat(),
            "analysis_text": analysis,
            "detections": detections or [],
            "image_size_bytes": len(image_data)
        }

        analysis_path = self.images_dir / f"{self.image_count:03d}_analysis.json"
        with open(analysis_path, "w", encoding="utf-8") as f:
            json.dump(analysis_data, f, ensure_ascii=False, indent=2)

        # 대화 기록에 추가
        entry = {
            "type": "image",
            "timestamp": datetime.now().isoformat(),
            "image_path": str(img_path.relative_to(self.session_dir)),
            "analysis": analysis,
            "detections": detections
        }
        self.conversation.append(entry)
        self.metadata["total_images"] += 1
        self._auto_save()

        print(f"[MissionLogger] 이미지 저장: {img_path}")
        return str(img_path)

    def log_image_base64(self, base64_data: str,
                         analysis: Optional[str] = None,
                         detections: Optional[List[Dict]] = None) -> str:
        """Base64 인코딩된 이미지 저장"""
        image_data = base64.b64decode(base64_data)
        return self.log_image(image_data, analysis, detections)

    def log_sensor_state(self, sensor_data: Dict):
        """센서 상태 스냅샷 저장"""
        entry = {
            "type": "sensor_snapshot",
            "timestamp": datetime.now().isoformat(),
            "data": sensor_data
        }
        self.conversation.append(entry)
        self._auto_save()

    def log_waypoint_reached(self, waypoint_name: str, position: Dict):
        """웨이포인트 도달 기록"""
        entry = {
            "type": "waypoint",
            "timestamp": datetime.now().isoformat(),
            "waypoint": waypoint_name,
            "position": position
        }
        self.conversation.append(entry)
        self._auto_save()

    def log_error(self, error_msg: str, context: Optional[Dict] = None):
        """에러 기록"""
        entry = {
            "type": "error",
            "timestamp": datetime.now().isoformat(),
            "error": error_msg,
            "context": context
        }
        self.conversation.append(entry)
        self._auto_save()

    def _auto_save(self):
        """자동 저장 (매 기록마다)"""
        self.save()

    def save(self):
        """대화 기록 저장"""
        # 메타데이터 업데이트
        self.metadata["end_time"] = datetime.now().isoformat()

        # conversation.json 저장
        conv_path = self.session_dir / "conversation.json"
        data = {
            "metadata": self.metadata,
            "conversation": self.conversation
        }
        with open(conv_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def complete_mission(self, status: str = "completed", summary: str = ""):
        """미션 완료 처리"""
        self.metadata["mission_status"] = status
        self.metadata["summary"] = summary
        self.save()
        print(f"[MissionLogger] 미션 {status}: {self.session_dir}")

    def get_session_path(self) -> str:
        """현재 세션 경로 반환"""
        return str(self.session_dir)


# 싱글톤 인스턴스
_logger_instance: Optional[MissionLogger] = None


def get_logger() -> MissionLogger:
    """글로벌 로거 인스턴스 반환"""
    global _logger_instance
    if _logger_instance is None:
        _logger_instance = MissionLogger()
    return _logger_instance


def new_session() -> MissionLogger:
    """새 로거 세션 시작"""
    global _logger_instance
    _logger_instance = MissionLogger()
    return _logger_instance


if __name__ == "__main__":
    # 테스트
    logger = MissionLogger()

    logger.log_prompt("현재 센서 상태를 확인해줘", {"gps": {"x": 10, "y": 20}})
    logger.log_response("현재 위치: (10, 20), 헤딩: 45도, 전방 클리어입니다.")
    logger.log_action("navigate_avoid", {"goal_x": 100, "goal_y": 50}, "started")
    logger.log_waypoint_reached("gate_start", {"x": -1.6, "y": 6.3})
    logger.complete_mission("completed", "게이트 통과 성공")

    print(f"\n테스트 완료. 저장 위치: {logger.get_session_path()}")
