# RoboBoat_2025 레포지토리 분석

## 개요

**레포**: https://github.com/InspirationRobotics/RoboBoat_2025
**팀**: InspirationRobotics
**대회**: RoboBoat 2025

---

## 가져온 것들

### 1. 클래스 정의 (competition_model) ✅
- 25개 클래스 (부표, 도형, 덕, 도킹 등)
- `models/classes.json`에 저장

### 2. HSV 색상 감지 코드 ✅
- `FTP_Cv.py` → `color_detector.py`
- 빨간/초록 부표 감지 로직
- 항행 오차 계산 함수

### 3. ONNX 모델 (sign_model) ⚠️
- `sign-simplified.onnx` (43MB)
- **문제 발견**: 클래스 메타데이터 손실 (class0-class998 placeholder)
- 원본 문서상 4개 클래스였으나 실제 모델에서 추출 불가
- 사용 권장하지 않음 - HSV 감지 또는 자체 학습 필요

---

## 적용 가능한 코드

| 모듈 | 파일 | 설명 | 적용 가능성 |
|------|------|------|------------|
| HSV 감지 | FTP_Cv.py | 색상 기반 부표 감지 | ✅ 완료 |
| 웨이포인트 | waypointNav.py | GPS 웨이포인트 네비게이션 | ⚠️ 유사 기능 있음 |
| 모터 제어 | motor_core.py | surge/veer/rotate 함수 | ⚠️ 유사 기능 있음 |
| GIS 함수 | gis_funcs.py | 거리/방위각 계산 | ⚠️ 유사 기능 있음 |

---

## 적용 불가능한 것들

| 항목 | 이유 |
|------|------|
| .blob 모델 | OAK-D 카메라 전용 (OpenVINO) |
| OAK-D API | 우리 시뮬레이터에 해당 카메라 없음 |
| T200 모터 드라이버 | 하드웨어 전용 |
| MiniMaestro 서보 | 하드웨어 전용 |

---

## 권장 사항

### 즉시 사용
1. **HSV 색상 감지** - 시뮬레이터에서 바로 작동
2. **클래스 정의** - 향후 YOLO 학습 시 참고

### 향후 개발
1. **YOLO 커스텀 학습** - VRX 시뮬레이터 이미지로 학습
   - classes.json의 25개 클래스 활용
   - 300-500장 이미지 필요
   - YOLOv8n 권장 (실시간 성능)

2. **웨이포인트 네비게이션** - 기존 코드와 비교하여 개선점 참고
   - P 제어 기반 조향
   - 거리 기반 속도 조절

---

## 레포 구조

```
RoboBoat_2025/
├── API/
│   ├── Camera/          # OAK-D 카메라 API
│   ├── GPS/             # GPS 인터페이스
│   ├── Motors/          # T200 모터 드라이버
│   └── Servos/          # 서보 제어
├── GNC/
│   ├── Control_Core/    # 모터 제어 로직
│   ├── Guidance_Core/   # 웨이포인트, 미션
│   └── info_core.py     # 센서 통합
├── Perception/
│   ├── Models/          # YOLO 모델들
│   └── Perception_Core/ # 인식 처리
└── Test_Scripts/        # 테스트 코드
```

---

## 결론

1. **HSV 색상 감지가 가장 실용적** - 추가 학습 없이 즉시 사용
2. **ONNX 모델은 제한적** - 입력 크기 고정, 클래스 제한
3. **competition_model(.blob)은 사용 불가** - OAK-D 전용
4. **원본 .pt 파일 없음** - 커스텀 학습 필요
