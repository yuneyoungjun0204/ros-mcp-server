# KABOAT Perception 모듈

RoboBoat 대회용 인식 시스템. **YOLO 클래스 라벨과 완전 호환**.

## 출처

- 클래스 정의: [InspirationRobotics/RoboBoat_2025](https://github.com/InspirationRobotics/RoboBoat_2025)
- HSV 색상 감지: FTP_Cv.py 기반

## YOLO 클래스 라벨 호환

HSV 감지와 YOLO 감지 모두 동일한 라벨 형식을 사용합니다:

| 색상 | 라벨 | class_id |
|------|------|----------|
| red | `red_buoy` | 16 |
| green | `green_buoy` | 11 |
| blue | `blue_buoy` | 4 |
| yellow | `yellow_buoy` | 23 |
| black | `black_buoy` | 0 |

```python
from kaboat_llm.perception import YOLO_LABELS, LABEL_TO_ID

# 25개 클래스 전체 목록
print(YOLO_LABELS)

# 라벨 → class_id 변환
print(LABEL_TO_ID['red_buoy'])  # 16
```

## 사용 가능한 모델

### 1. HSV 기반 색상 감지 (권장)
```python
from kaboat_llm.perception import ColorBuoyDetector

detector = ColorBuoyDetector(enabled_colors=['red', 'green'])
detections = detector.detect(frame)

for det in detections:
    print(f'{det.label} (class_id={det.class_id}) at {det.center}')
    # 출력: red_buoy (class_id=16) at (150, 200)
```

- YOLO 없이 즉시 사용 가능
- 빨간/초록/파란/노란/검정 부표 감지
- CPU만으로 실시간 처리 (30+ FPS)
- **YOLO 호환 라벨 및 class_id 출력**

### 2. ONNX 모델 (sign-simplified.onnx) - ⚠️ 사용 주의

**현재 상태: ultralytics 래퍼로는 사용 불가**

원본 RoboBoat_2025 레포의 ONNX 모델은 클래스 메타데이터가 내장되어 있지 않습니다:
- 원본 클래스: Black Boat, Cross, Orange Boat, Triangle (4개)
- ultralytics 로드 시: `class0`, `class1`, ... (placeholder 이름으로 표시)

**모델 구조는 정상** (출력 shape: `[1, 9, H, W]` = 4 bbox + 1 obj + 4 class scores)

**사용 방법:**
1. **권장**: HSV 색상 감지 사용 (즉시 작동, 학습 불필요)
2. **대안**: raw onnxruntime + 수동 라벨 매핑
3. **대안**: VRX 시뮬레이터에서 자체 YOLO 모델 학습

```python
# 방법 1: HSV 감지 (권장)
from kaboat_llm.perception.color_detector import ColorBuoyDetector
detector = ColorBuoyDetector(enabled_colors=['red', 'green'])

# 방법 2: raw onnxruntime (고급 사용자)
import onnxruntime as ort
LABELS = ['Black Boat', 'Cross', 'Orange Boat', 'Triangle']
session = ort.InferenceSession('sign-simplified.onnx')
# ... 후처리에서 LABELS[class_id] 사용
```

### 3. 클래스 정의 (classes.json)

RoboBoat 대회용 25개 클래스:
- 부표: black_buoy, blue_buoy, green_buoy, red_buoy, yellow_buoy, misc_buoy
- 폴 부표: green_pole_buoy, red_pole_buoy
- 도형: circle, cross, triangle, square (색상별)
- 기타: dock, rubber_duck, racquet_ball

## VRX 시뮬레이터 사용

VRX 시뮬레이터에서는 HSV 색상 감지를 권장합니다:
1. 부표 색상이 명확함
2. 추가 학습 데이터 없이 즉시 작동
3. 실시간 성능 보장

### YOLO 커스텀 학습이 필요한 경우

1. VRX 시뮬레이터에서 이미지 수집
2. Roboflow로 라벨링
3. YOLOv8 fine-tuning

```bash
# 학습 예시
yolo train model=yolov8n.pt data=vrx_buoys.yaml epochs=50
```

## 파일 구조

```
perception/
├── README.md
├── __init__.py
├── color_detector.py    # HSV 기반 감지
├── yolo_detector.py     # YOLO ROS 노드 (TODO)
└── models/
    ├── classes.json     # 25개 클래스 정의
    └── sign-simplified.onnx  # ONNX 모델 (640x352)
```
