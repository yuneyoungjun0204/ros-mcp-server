#!/usr/bin/env python3
"""
HSV 기반 색상 감지 모듈
출처: InspirationRobotics/RoboBoat_2025 - GNC/Guidance_Core/Missions/FTP_Cv.py

RoboBoat 대회용 빨간/초록/파란/노란 부표 감지
YOLO 클래스 라벨과 호환되는 출력 형식 사용
"""
import cv2
import numpy as np
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, List, Dict


# YOLO 클래스 라벨 (competition_model 기준)
YOLO_LABELS = [
    "black_buoy", "black_circle", "black_cross", "black_triangle",
    "blue_buoy", "blue_circle", "blue_cross", "blue_racquet_ball", "blue_triangle",
    "dock", "duck_image",
    "green_buoy", "green_cross", "green_pole_buoy", "green_triangle",
    "misc_buoy",
    "red_buoy", "red_circle", "red_cross", "red_pole_buoy", "red_racquet_ball", "red_square",
    "rubber_duck",
    "yellow_buoy", "yellow_racquet_ball"
]

# 색상 → YOLO 라벨 매핑
COLOR_TO_LABEL = {
    'red': 'red_buoy',
    'green': 'green_buoy',
    'blue': 'blue_buoy',
    'yellow': 'yellow_buoy',
    'black': 'black_buoy',
}

# YOLO 라벨 → class_id 매핑
LABEL_TO_ID = {label: idx for idx, label in enumerate(YOLO_LABELS)}


@dataclass
class BuoyDetection:
    """부표 감지 결과 (YOLO 호환 형식)"""
    color: str
    center: Tuple[int, int]
    area: int
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    normalized_x: float  # 0-1 범위
    label: str = ""  # YOLO 호환 라벨 (red_buoy, green_buoy, ...)
    class_id: int = -1  # YOLO class index

    def __post_init__(self):
        if not self.label:
            self.label = COLOR_TO_LABEL.get(self.color, f"{self.color}_buoy")
        if self.class_id < 0:
            self.class_id = LABEL_TO_ID.get(self.label, -1)


def preprocess_frame(frame: np.ndarray) -> np.ndarray:
    """HSV 변환 및 가우시안 블러 적용"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    return hsv


def get_red_mask(hsv: np.ndarray) -> np.ndarray:
    """빨간색 부표 마스크 (HSV wrap-around 처리)"""
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def get_green_mask(hsv: np.ndarray) -> np.ndarray:
    """초록색 부표 마스크"""
    lower_green = np.array([70, 50, 130])
    upper_green = np.array([90, 255, 255])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def get_blue_mask(hsv: np.ndarray) -> np.ndarray:
    """파란색 부표 마스크"""
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def get_yellow_mask(hsv: np.ndarray) -> np.ndarray:
    """노란색 부표 마스크"""
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])

    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def get_black_mask(hsv: np.ndarray) -> np.ndarray:
    """검정색 부표 마스크 (낮은 Value)"""
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 50])

    mask = cv2.inRange(hsv, lower_black, upper_black)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def find_largest_contour(mask: np.ndarray, min_area: int = 500) -> Optional[Tuple[np.ndarray, Tuple[int, int]]]:
    """마스크에서 가장 큰 컨투어와 중심점 찾기"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)

    if area < min_area:
        return None

    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return largest_contour, (cx, cy)

    return None


def find_all_contours(mask: np.ndarray, min_area: int = 500) -> List[Tuple[np.ndarray, Tuple[int, int], int]]:
    """마스크에서 모든 유효 컨투어, 중심점, 면적 찾기"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_area:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                results.append((contour, (cx, cy), area))

    return sorted(results, key=lambda x: x[2], reverse=True)


class ColorBuoyDetector:
    """
    HSV 기반 부표 감지기

    사용법:
        detector = ColorBuoyDetector()
        detections = detector.detect(frame)
        for det in detections:
            print(f"{det.color} buoy at {det.center}")
    """

    COLOR_MASKS = {
        'red': get_red_mask,
        'green': get_green_mask,
        'blue': get_blue_mask,
        'yellow': get_yellow_mask,
        'black': get_black_mask,
    }

    def __init__(self, min_area: int = 500, enabled_colors: List[str] = None):
        """
        Args:
            min_area: 최소 감지 면적 (픽셀)
            enabled_colors: 감지할 색상 목록 (None이면 전체)
        """
        self.min_area = min_area
        self.enabled_colors = enabled_colors or list(self.COLOR_MASKS.keys())

    def detect(self, frame: np.ndarray) -> List[BuoyDetection]:
        """프레임에서 부표 감지"""
        hsv = preprocess_frame(frame)
        detections = []
        frame_width = frame.shape[1]

        for color in self.enabled_colors:
            if color not in self.COLOR_MASKS:
                continue

            mask = self.COLOR_MASKS[color](hsv)
            contours = find_all_contours(mask, self.min_area)

            for contour, center, area in contours:
                x, y, w, h = cv2.boundingRect(contour)
                detections.append(BuoyDetection(
                    color=color,
                    center=center,
                    area=area,
                    bbox=(x, y, w, h),
                    normalized_x=center[0] / frame_width
                ))

        return detections

    def detect_and_draw(self, frame: np.ndarray) -> Tuple[np.ndarray, List[BuoyDetection]]:
        """프레임에서 부표 감지 및 시각화"""
        detections = self.detect(frame)
        result_frame = frame.copy()

        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'black': (128, 128, 128),
        }

        for det in detections:
            color = color_map.get(det.color, (255, 255, 255))
            x, y, w, h = det.bbox
            cv2.rectangle(result_frame, (x, y), (x + w, y + h), color, 2)
            cv2.circle(result_frame, det.center, 5, color, -1)
            label = f"{det.color} ({det.normalized_x:.2f})"
            cv2.putText(result_frame, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return result_frame, detections


def compute_navigation_error(red_x: Optional[float], green_x: Optional[float]) -> Tuple[float, str]:
    """
    빨간/초록 부표 위치로 항행 오차 계산

    Args:
        red_x: 빨간 부표의 normalized x (0-1), None이면 미감지
        green_x: 초록 부표의 normalized x (0-1), None이면 미감지

    Returns:
        (error, description): 오차값 (-1 ~ 1, 음수=좌회전, 양수=우회전)과 설명
    """
    if red_x is not None and green_x is not None:
        midpoint = (red_x + green_x) / 2
        error = (midpoint - 0.5) * 2  # -1 to 1
        return error, "both_visible"

    elif red_x is None and green_x is not None:
        if green_x > 0.75:
            return 0.0, "green_far_right_go_straight"
        elif green_x > 0.5:
            return -0.2, "green_right_slight_left"
        elif green_x > 0.25:
            return -0.4, "green_center_turn_left"
        else:
            return -0.6, "green_left_hard_left"

    elif green_x is None and red_x is not None:
        if red_x < 0.25:
            return 0.0, "red_far_left_go_straight"
        elif red_x < 0.5:
            return 0.2, "red_left_slight_right"
        elif red_x < 0.75:
            return 0.4, "red_center_turn_right"
        else:
            return 0.6, "red_right_hard_right"

    return 0.0, "no_buoys_detected"


if __name__ == "__main__":
    detector = ColorBuoyDetector(enabled_colors=['red', 'green'])

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다")
        exit(1)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result, detections = detector.detect_and_draw(frame)

        red_x = None
        green_x = None
        for det in detections:
            if det.color == 'red':
                red_x = det.normalized_x
            elif det.color == 'green':
                green_x = det.normalized_x

        error, desc = compute_navigation_error(red_x, green_x)
        cv2.putText(result, f"Error: {error:.2f} ({desc})", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("Buoy Detection", result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
