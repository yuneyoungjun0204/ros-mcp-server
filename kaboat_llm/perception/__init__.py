"""
KABOAT Perception 모듈

YOLO 호환 클래스 라벨을 사용하는 객체 감지 시스템
"""
from .color_detector import (
    ColorBuoyDetector,
    BuoyDetection,
    YOLO_LABELS,
    LABEL_TO_ID,
    COLOR_TO_LABEL,
    compute_navigation_error,
)

__all__ = [
    'ColorBuoyDetector',
    'BuoyDetection',
    'YOLO_LABELS',
    'LABEL_TO_ID',
    'COLOR_TO_LABEL',
    'compute_navigation_error',
]
