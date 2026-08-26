#!/usr/bin/env python3
"""
미션 GIF 캡처 스크립트 (선택적)
- Gazebo 창 또는 시각화 창 캡처
- 프레임 모아서 GIF 생성

사용법:
    python3 capture_gif.py --duration 30 --output mission.gif

의존성:
    pip install pillow mss
"""
import argparse
import time
import os
from pathlib import Path
from datetime import datetime

try:
    from PIL import Image
    import mss
    CAPTURE_AVAILABLE = True
except ImportError:
    CAPTURE_AVAILABLE = False
    print("캡처 모듈 없음. 설치: pip install pillow mss")


def capture_screen_region(monitor_region=None):
    """화면 영역 캡처"""
    if not CAPTURE_AVAILABLE:
        return None

    with mss.mss() as sct:
        if monitor_region:
            region = monitor_region
        else:
            # 전체 화면 (첫 번째 모니터)
            region = sct.monitors[1]

        screenshot = sct.grab(region)
        img = Image.frombytes('RGB', screenshot.size, screenshot.bgra, 'raw', 'BGRX')
        return img


def capture_gif(output_path: str, duration: float = 30.0, fps: float = 2.0,
                region: dict = None):
    """
    GIF 캡처

    Args:
        output_path: 저장 경로
        duration: 캡처 시간 (초)
        fps: 초당 프레임 수
        region: 캡처 영역 {"left": x, "top": y, "width": w, "height": h}
    """
    if not CAPTURE_AVAILABLE:
        print("캡처 불가: pillow, mss 설치 필요")
        return False

    frames = []
    interval = 1.0 / fps
    start_time = time.time()

    print(f"GIF 캡처 시작: {duration}초, {fps} FPS")
    print(f"저장 위치: {output_path}")

    while time.time() - start_time < duration:
        frame = capture_screen_region(region)
        if frame:
            # 크기 조절 (GIF 용량 줄이기)
            frame = frame.resize((frame.width // 2, frame.height // 2), Image.Resampling.LANCZOS)
            frames.append(frame)

        elapsed = time.time() - start_time
        print(f"\r캡처 중: {elapsed:.1f}/{duration}초, {len(frames)} 프레임", end="")
        time.sleep(interval)

    print(f"\n캡처 완료. {len(frames)} 프레임 저장 중...")

    if frames:
        frames[0].save(
            output_path,
            save_all=True,
            append_images=frames[1:],
            duration=int(1000 / fps),
            loop=0,
            optimize=True
        )
        print(f"GIF 저장 완료: {output_path}")
        return True

    return False


def main():
    parser = argparse.ArgumentParser(description='미션 GIF 캡처')
    parser.add_argument('--duration', type=float, default=30.0, help='캡처 시간 (초)')
    parser.add_argument('--fps', type=float, default=2.0, help='초당 프레임')
    parser.add_argument('--output', type=str, default=None, help='출력 파일')
    parser.add_argument('--left', type=int, default=None, help='캡처 영역 X')
    parser.add_argument('--top', type=int, default=None, help='캡처 영역 Y')
    parser.add_argument('--width', type=int, default=None, help='캡처 영역 폭')
    parser.add_argument('--height', type=int, default=None, help='캡처 영역 높이')

    args = parser.parse_args()

    # 출력 경로
    if args.output:
        output_path = args.output
    else:
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_path = f"/home/yune/ros-mcp-server/history/{timestamp}_mission.gif"

    # 캡처 영역
    region = None
    if all([args.left, args.top, args.width, args.height]):
        region = {
            "left": args.left,
            "top": args.top,
            "width": args.width,
            "height": args.height
        }

    capture_gif(output_path, args.duration, args.fps, region)


if __name__ == "__main__":
    main()
