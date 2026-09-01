# KABOAT LLM 지식 증류 및 경량화 계획

KABOAT 자율주행 시스템을 위한 온보드 LLM 구축 계획서입니다.
Claude를 교사 모델로 활용하여 데이터셋을 생성하고, 경량 VLM/SLM을 파인튜닝하여 실시간 추론이 가능한 온보드 모델을 구축합니다.

---

## 1. 목표 및 요구사항

### 1.1 최종 목표

| 항목 | 요구사항 |
|------|---------|
| **추론 지연** | < 200ms (10Hz 제어 루프 호환) |
| **출력 형식** | ROS 2 MCP JSON Tool Call 전용 |
| **입력 모달리티** | 텍스트 (센서 요약) + 이미지 (카메라) |
| **배포 환경** | Jetson Orin / Intel NUC (16GB RAM) |
| **네트워크** | 오프라인 동작 필수 |

### 1.2 현재 상태 vs 목표 상태

```
현재: 센서 → kaboat_autonomous (규칙 기반) → 스러스터
         └── 고정된 알고리즘, 상황 적응 제한

목표: 센서 → 경량 VLM → ROS 2 MCP JSON → kaboat_tools → 스러스터
         └── 상황별 판단, 자연어 명령 이해, 유연한 미션 수행
```

---

## 2. 아키텍처 개요

```
┌─────────────────────────────────────────────────────────────────┐
│                    Phase 1: 데이터셋 생성                        │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │ 시나리오 템플릿│ →  │ Claude API  │  → │ 검증된 데이터셋  │  │
│  │   (100개)     │    │ (Self-Instruct)│  │  (10,000+ 샘플)  │  │
│  └──────────────┘    └──────────────┘    └──────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    Phase 2: QLoRA 파인튜닝                       │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │ 베이스 모델   │ +  │ 데이터셋     │  → │ 파인튜닝된 모델  │  │
│  │ (Qwen2.5-VL) │    │ (Phase 1)    │    │  (LoRA Adapters) │  │
│  └──────────────┘    └──────────────┘    └──────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    Phase 3: 배포 및 통합                         │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────┐  │
│  │ 양자화 (AWQ) │ →  │ vLLM/llama.cpp│ → │ ros-mcp-server   │  │
│  │  INT4/INT8   │    │  로컬 서버    │    │    통합          │  │
│  └──────────────┘    └──────────────┘    └──────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Phase 1: 데이터셋 생성 (Claude 교사)

### 3.1 데이터 스키마

#### 입력 형식 (Input)

```json
{
  "context": {
    "position": {"x": -520.5, "y": 180.3, "heading": 45.2},
    "sensors": {
      "lidar": {
        "front": 15.2,
        "left": 8.5,
        "right": 22.1,
        "obstacles_detected": 3
      },
      "gps": {"lat": -33.7227, "lon": 150.6740},
      "imu": {"roll": 0.5, "pitch": 1.2, "yaw": 45.2}
    },
    "vision": {
      "description": "전방 10m에 빨간 부표, 좌측 15m에 녹색 부표",
      "detected_objects": [
        {"type": "buoy", "color": "red", "distance": 10, "bearing": 0},
        {"type": "buoy", "color": "green", "distance": 15, "bearing": -30}
      ]
    },
    "mission": {
      "type": "gate_navigation",
      "current_waypoint": {"x": -530, "y": 200},
      "progress": 0.4
    }
  },
  "user_command": "빨간 부표와 녹색 부표 사이로 통과해"
}
```

#### 출력 형식 (Output - ROS 2 MCP JSON Tool Call)

```json
{
  "tool": "kaboat_navigate_between",
  "parameters": {
    "target_left": {"type": "buoy", "color": "red"},
    "target_right": {"type": "buoy", "color": "green"},
    "speed": "normal",
    "avoid_obstacles": true
  },
  "reasoning": "빨간 부표(우측)와 녹색 부표(좌측) 사이 게이트 통과"
}
```

### 3.2 시나리오 카테고리 (100개 템플릿)

| 카테고리 | 시나리오 수 | 예시 |
|---------|-----------|------|
| **네비게이션** | 25 | 웨이포인트 이동, GPS 좌표 도달, 경로 추종 |
| **장애물 회피** | 20 | 전방 장애물 회피, 좁은 통로 통과, 동적 장애물 |
| **부표 미션** | 20 | 게이트 통과, 부표 선회, 색상 인식 순서 보고 |
| **도킹** | 15 | 도킹 접근, 정밀 위치 조정, 도킹 완료 판정 |
| **예외 상황** | 10 | 센서 이상, 위치 불확실, 미션 중단 |
| **안전** | 10 | 긴급 정지, 안전 거리 유지, 귀환 |

### 3.3 데이터 증강 전략

```python
# Self-Instruct 방식: Claude로 변형 생성
augmentation_strategies = {
    "paraphrase": "동일 의미, 다른 표현으로 명령 재작성",
    "parameter_variation": "거리/각도/속도 파라미터 변형",
    "context_shift": "위치/센서값/미션 상태 변형",
    "error_injection": "센서 노이즈, 부분 정보 누락",
    "language_variation": "한국어/영어 혼용, 구어체/문어체",
}
```

### 3.4 데이터셋 생성 파이프라인

```
1. 시드 시나리오 작성 (수동, 100개)
   └── templates/scenarios/*.yaml

2. Claude API로 변형 생성 (자동, 시나리오당 100개)
   └── scripts/generate_dataset.py
   └── 총 10,000개 raw 샘플

3. 검증 및 필터링
   ├── JSON 스키마 검증
   ├── ROS 2 메시지 파싱 테스트
   └── 중복/유사도 필터링 (SimHash)
   └── 최종 8,000개 학습용, 1,000개 검증용, 1,000개 테스트용
```

### 3.5 품질 보증

| 검증 항목 | 방법 |
|----------|------|
| **JSON 유효성** | jsonschema 라이브러리로 스키마 검증 |
| **Tool 존재 확인** | ros-mcp-server 등록된 도구와 매칭 |
| **파라미터 범위** | 물리적 타당성 (속도 < MAX_THRUST 등) |
| **일관성** | 동일 입력 → 유사 출력 (Claude 재생성 비교) |

---

## 4. Phase 2: QLoRA 파인튜닝

### 4.1 베이스 모델 후보

| 모델 | 파라미터 | Vision | 한국어 | 추천도 |
|------|---------|--------|-------|-------|
| **Qwen2.5-VL-7B** | 7B | O | O | ★★★★★ |
| Phi-3.5-Vision | 4.2B | O | △ | ★★★★☆ |
| LLaVA-1.6-Mistral-7B | 7B | O | △ | ★★★☆☆ |
| InternVL2-8B | 8B | O | O | ★★★★☆ |

**선정: Qwen2.5-VL-7B**
- 이유: Vision 지원, 한국어 성능 우수, Apache 2.0 라이선스

### 4.2 QLoRA 설정

```python
# peft 라이브러리 설정
lora_config = LoraConfig(
    r=64,                    # LoRA rank
    lora_alpha=128,          # 스케일링 팩터
    target_modules=[
        "q_proj", "k_proj", "v_proj", "o_proj",  # Attention
        "gate_proj", "up_proj", "down_proj"       # MLP
    ],
    lora_dropout=0.05,
    bias="none",
    task_type="CAUSAL_LM"
)

# 양자화 설정 (4-bit)
bnb_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=torch.bfloat16,
    bnb_4bit_use_double_quant=True
)
```

### 4.3 학습 설정

```python
training_args = TrainingArguments(
    output_dir="./kaboat-vlm-lora",
    num_train_epochs=3,
    per_device_train_batch_size=4,
    gradient_accumulation_steps=4,
    learning_rate=2e-4,
    warmup_ratio=0.03,
    lr_scheduler_type="cosine",
    logging_steps=10,
    save_strategy="epoch",
    evaluation_strategy="epoch",
    bf16=True,
    gradient_checkpointing=True,
    max_grad_norm=0.3,
)
```

### 4.4 Constrained Output 학습

모델이 **오직 유효한 JSON Tool Call만** 출력하도록 제약:

```python
# 1. 특수 토큰 추가
special_tokens = {
    "bos_token": "<|tool_call|>",
    "eos_token": "<|end_tool|>",
    "pad_token": "<|pad|>"
}

# 2. 학습 데이터 포맷
template = """<|context|>
{input_context}
<|command|>
{user_command}
<|tool_call|>
{json_output}
<|end_tool|>"""

# 3. Constrained Decoding (추론 시)
from outlines import models, generate
model = models.transformers("./kaboat-vlm-lora")
generator = generate.json(model, KaboatToolCall)  # Pydantic 스키마
```

### 4.5 학습 리소스 요구사항

| 단계 | GPU 메모리 | 시간 (RTX 4090) |
|------|-----------|----------------|
| 모델 로드 (4-bit) | ~8GB | - |
| 학습 (batch=4, grad_accum=4) | ~20GB | 4-6시간 |
| 추론 테스트 | ~6GB | - |

---

## 5. Phase 3: 배포 및 통합

### 5.1 모델 양자화

```bash
# AWQ 양자화 (INT4, 더 빠른 추론)
python -m awq.entry --model_path ./kaboat-vlm-lora \
    --w_bit 4 --q_group_size 128 \
    --output_path ./kaboat-vlm-awq
```

### 5.2 추론 서버 설정

#### Option A: vLLM (권장, 높은 처리량)

```python
from vllm import LLM, SamplingParams

llm = LLM(
    model="./kaboat-vlm-awq",
    quantization="awq",
    dtype="half",
    max_model_len=2048,
    gpu_memory_utilization=0.8
)

sampling_params = SamplingParams(
    temperature=0.1,      # 낮은 temperature로 결정적 출력
    max_tokens=256,
    stop=["<|end_tool|>"]
)
```

#### Option B: llama.cpp (Jetson 최적화)

```bash
# GGUF 변환
python convert.py ./kaboat-vlm-lora --outtype f16 --outfile kaboat-vlm.gguf

# 양자화
./quantize kaboat-vlm.gguf kaboat-vlm-q4_k_m.gguf q4_k_m

# 서버 실행
./server -m kaboat-vlm-q4_k_m.gguf -c 2048 --port 8080
```

### 5.3 ros-mcp-server 통합

```
ros-mcp-server/
├── ros_mcp/
│   ├── llm/
│   │   ├── __init__.py
│   │   ├── local_inference.py    # 로컬 LLM 추론
│   │   ├── tool_parser.py        # JSON Tool Call 파싱
│   │   └── safety_validator.py   # 안전 검증
│   └── tools/
│       └── kaboat_tools.py       # KABOAT MCP 도구
└── robot_specifications/
    └── wamv_kaboat.yaml
```

#### local_inference.py

```python
from typing import Optional
import httpx
from pydantic import BaseModel

class KaboatToolCall(BaseModel):
    tool: str
    parameters: dict
    reasoning: Optional[str] = None

class LocalLLMClient:
    def __init__(self, endpoint: str = "http://localhost:8080"):
        self.endpoint = endpoint
        self.client = httpx.AsyncClient(timeout=5.0)
    
    async def generate_tool_call(
        self,
        context: dict,
        user_command: str
    ) -> KaboatToolCall:
        prompt = self._format_prompt(context, user_command)
        
        response = await self.client.post(
            f"{self.endpoint}/completion",
            json={
                "prompt": prompt,
                "temperature": 0.1,
                "max_tokens": 256,
                "stop": ["<|end_tool|>"]
            }
        )
        
        result = response.json()["content"]
        return KaboatToolCall.model_validate_json(result)
```

### 5.4 실시간 파이프라인

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Sensors   │ →  │  Context    │ →  │  Local LLM  │ →  │  MCP Tool   │
│  (10Hz)     │    │  Builder    │    │  (5Hz)      │    │  Executor   │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
      │                  │                  │                  │
      │                  │                  │                  ▼
      │                  │                  │           ┌─────────────┐
      └──────────────────┴──────────────────┴─────────→│ Safety      │
                    Emergency Override                  │ Watchdog    │
                                                       └─────────────┘
```

---

## 6. 디렉토리 구조

```
kaboat_llm/
├── README.md
├── pyproject.toml
│
├── data/
│   ├── templates/
│   │   └── scenarios/           # 시드 시나리오 (100개)
│   │       ├── navigation.yaml
│   │       ├── obstacle_avoidance.yaml
│   │       ├── buoy_missions.yaml
│   │       ├── docking.yaml
│   │       ├── exceptions.yaml
│   │       └── safety.yaml
│   ├── raw/                     # Claude 생성 raw 데이터
│   ├── processed/               # 검증된 학습 데이터
│   │   ├── train.jsonl
│   │   ├── val.jsonl
│   │   └── test.jsonl
│   └── schemas/
│       └── tool_call.json       # JSON 스키마
│
├── scripts/
│   ├── generate_dataset.py      # Claude API 데이터 생성
│   ├── validate_dataset.py      # 데이터 검증
│   ├── train_qlora.py           # QLoRA 학습
│   ├── convert_gguf.py          # GGUF 변환
│   └── evaluate.py              # 평가
│
├── src/
│   └── kaboat_llm/
│       ├── __init__.py
│       ├── context_builder.py   # 센서 → 컨텍스트 변환
│       ├── inference.py         # 로컬 추론
│       ├── tool_executor.py     # MCP 도구 실행
│       └── safety.py            # 안전 검증
│
├── models/
│   ├── base/                    # 베이스 모델
│   ├── lora/                    # LoRA 어댑터
│   └── quantized/               # 양자화 모델
│
└── tests/
    ├── test_dataset.py
    ├── test_inference.py
    └── test_integration.py
```

---

## 7. 일정 및 마일스톤

| 주차 | Phase | 작업 내용 | 산출물 |
|------|-------|----------|--------|
| **W1** | 1 | 시나리오 템플릿 작성 (100개) | `data/templates/` |
| **W2** | 1 | Claude 데이터 생성 스크립트 | `scripts/generate_dataset.py` |
| **W3** | 1 | 데이터 검증 및 필터링 | `data/processed/*.jsonl` |
| **W4** | 2 | QLoRA 학습 환경 구축 | `scripts/train_qlora.py` |
| **W5** | 2 | 파인튜닝 및 평가 | `models/lora/` |
| **W6** | 3 | 양자화 및 추론 서버 | `models/quantized/` |
| **W7** | 3 | ros-mcp-server 통합 | `src/kaboat_llm/` |
| **W8** | 3 | VRX 시뮬레이션 테스트 | 데모 영상 |

---

## 8. 평가 지표

### 8.1 기능 평가

| 지표 | 목표 | 측정 방법 |
|------|------|----------|
| **Tool 정확도** | > 95% | 정답 Tool 선택 비율 |
| **파라미터 정확도** | > 90% | 파라미터 값 일치율 |
| **JSON 유효율** | 100% | 파싱 가능한 출력 비율 |
| **지연시간** | < 200ms | P95 추론 시간 |

### 8.2 시뮬레이션 평가

| 미션 | 성공 기준 | 평가 횟수 |
|------|----------|----------|
| 게이트 통과 | 충돌 없이 통과 | 20회 |
| 부표 선회 | 반경 8m 내 궤도 유지 | 20회 |
| 도킹 | 오차 < 1m | 20회 |
| 장애물 회피 | 충돌 0회 | 50회 |

---

## 9. 위험 요소 및 대응

| 위험 | 영향 | 대응 |
|------|------|------|
| Claude API 비용 초과 | 데이터 생성 지연 | 배치 요청, 캐싱, 예산 상한 설정 |
| 모델 성능 미달 | 추가 학습 필요 | 데이터 증강, r 값 증가, 에폭 추가 |
| Jetson 메모리 부족 | 배포 불가 | 더 작은 모델 (3B) 또는 추가 양자화 |
| 실시간성 미달 | 제어 지연 | 규칙 기반 fallback, 비동기 추론 |

---

## 10. 다음 단계

1. **즉시**: 시나리오 템플릿 YAML 작성 시작
2. **이번 주**: Claude API 데이터 생성 스크립트 구현
3. **검토 필요**: 베이스 모델 최종 선정 (Qwen2.5-VL vs InternVL2)

---

## 부록 A: MCP Tool 목록 (ros-mcp-server 연동 대상)

| Tool 이름 | 설명 | 파라미터 |
|-----------|------|---------|
| `kaboat_get_status` | 보트 상태 조회 | - |
| `kaboat_navigate_to` | GPS/로컬 좌표 이동 | `target`, `speed` |
| `kaboat_navigate_between` | 두 물체 사이 통과 | `target_left`, `target_right` |
| `kaboat_orbit_buoy` | 부표 선회 | `color`, `radius`, `direction` |
| `kaboat_hold_position` | 위치 유지 | `duration` |
| `kaboat_emergency_stop` | 긴급 정지 | - |
| `kaboat_dock` | 도킹 수행 | `bay_id` |
| `kaboat_search_pattern` | 탐색 패턴 실행 | `pattern`, `range` |
| `kaboat_report_observation` | 관측 보고 | `type`, `details` |

---

## 부록 B: 실패 케이스 학습 데이터 예시

```json
{
  "context": {
    "sensors": {
      "lidar": {"front": 2.1, "obstacles_detected": 1},
      "vision": {"description": "전방 장애물 매우 가까움"}
    }
  },
  "user_command": "앞으로 가",
  "output": {
    "tool": "kaboat_emergency_stop",
    "parameters": {},
    "reasoning": "전방 2.1m에 장애물 감지. 안전 거리(5m) 미만으로 긴급 정지 후 회피 기동 필요"
  }
}
```

```json
{
  "context": {
    "sensors": {
      "vision": {"description": "안개로 시야 제한", "visibility": "low"}
    },
    "mission": {"type": "buoy_detection"}
  },
  "user_command": "부표 찾아",
  "output": {
    "tool": "kaboat_search_pattern",
    "parameters": {"pattern": "expanding_square", "range": 50},
    "reasoning": "시야 제한으로 직접 탐지 불가. 확장 사각형 패턴으로 탐색 시작"
  }
}
```
