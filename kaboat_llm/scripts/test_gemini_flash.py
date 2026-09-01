#!/usr/bin/env python3
"""
Gemini Flash 응답 속도 테스트
KABOAT 시나리오로 JSON Tool Call 생성 시간 측정
"""

import os
import time
import json
import google.generativeai as genai

# API 키 설정
GOOGLE_API_KEY = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
if not GOOGLE_API_KEY:
    print("[ERROR] GOOGLE_API_KEY 또는 GEMINI_API_KEY 환경변수 필요")
    print("  export GOOGLE_API_KEY='your-api-key'")
    exit(1)

genai.configure(api_key=GOOGLE_API_KEY)

# 시스템 프롬프트
SYSTEM_PROMPT = """당신은 KABOAT 자율주행 선박의 AI 판단 시스템입니다.

주어진 센서 상황과 사용자 명령을 분석하여, 적절한 ROS 2 MCP Tool Call을 JSON으로 출력하세요.

사용 가능한 도구:
- kaboat_navigate_to: 좌표 이동
- kaboat_avoid_obstacle: 장애물 회피
- kaboat_orbit_buoy: 부표 선회
- kaboat_emergency_stop: 긴급 정지
- kaboat_hold_position: 위치 유지

출력 형식 (JSON만, 다른 텍스트 없이):
{"tool": "tool_name", "parameters": {...}}
"""

# 테스트 시나리오들
TEST_SCENARIOS = [
    {
        "name": "단순 이동",
        "context": "위치: (0, 0), 헤딩: 45°, LiDAR: 전방 30m 장애물 없음",
        "command": "웨이포인트로 가"
    },
    {
        "name": "장애물 회피",
        "context": "위치: (-520, 180), 헤딩: 0°, LiDAR: 전방 5m 장애물, 좌측 20m, 우측 8m",
        "command": "앞으로 가"
    },
    {
        "name": "긴급 정지",
        "context": "위치: (10, 10), 헤딩: 90°, LiDAR: 전방 2m 장애물",
        "command": "계속 가"
    },
    {
        "name": "부표 선회",
        "context": "위치: (-500, 200), 헤딩: 45°, Vision: 빨간 부표 정면 10m",
        "command": "빨간 부표 선회해"
    },
    {
        "name": "게이트 통과",
        "context": "위치: (-530, 190), 헤딩: 0°, Vision: 빨간 부표 우측 15m, 녹색 부표 좌측 15m",
        "command": "게이트 통과해"
    },
]


def test_gemini_flash(scenario: dict) -> dict:
    """단일 시나리오 테스트"""

    model = genai.GenerativeModel(
        model_name="gemini-flash-lite-latest",  # 최신 Flash Lite (더 빠름, 더 저렴)
        system_instruction=SYSTEM_PROMPT,
        generation_config={
            "temperature": 0.1,
            "max_output_tokens": 256,
        }
    )

    prompt = f"""현재 상황:
{scenario['context']}

사용자 명령: "{scenario['command']}"

JSON Tool Call을 출력하세요:"""

    # 시간 측정
    start_time = time.time()
    response = model.generate_content(prompt)
    end_time = time.time()

    latency = end_time - start_time
    output = response.text.strip()

    # JSON 파싱 시도
    try:
        # 코드 블록 제거
        if "```" in output:
            output = output.split("```")[1]
            if output.startswith("json"):
                output = output[4:]
        parsed = json.loads(output.strip())
        valid_json = True
    except:
        parsed = None
        valid_json = False

    return {
        "scenario": scenario["name"],
        "latency_ms": int(latency * 1000),
        "valid_json": valid_json,
        "output": output[:200],  # 처음 200자만
        "parsed": parsed
    }


def main():
    print("=" * 60)
    print("Gemini Flash 응답 속도 테스트")
    print("=" * 60)

    results = []

    for i, scenario in enumerate(TEST_SCENARIOS):
        print(f"\n[{i+1}/{len(TEST_SCENARIOS)}] {scenario['name']}...")

        try:
            result = test_gemini_flash(scenario)
            results.append(result)

            status = "✅" if result["valid_json"] else "❌"
            print(f"  {status} {result['latency_ms']}ms")
            if result["parsed"]:
                print(f"  → {result['parsed'].get('tool', 'N/A')}")
            else:
                print(f"  → {result['output'][:50]}...")

        except Exception as e:
            print(f"  ❌ 오류: {e}")
            results.append({
                "scenario": scenario["name"],
                "latency_ms": -1,
                "valid_json": False,
                "error": str(e)
            })

    # 결과 요약
    print("\n" + "=" * 60)
    print("결과 요약")
    print("=" * 60)

    valid_results = [r for r in results if r["latency_ms"] > 0]
    if valid_results:
        latencies = [r["latency_ms"] for r in valid_results]
        valid_jsons = [r for r in valid_results if r["valid_json"]]

        print(f"  테스트 수: {len(TEST_SCENARIOS)}")
        print(f"  성공: {len(valid_results)}")
        print(f"  JSON 유효: {len(valid_jsons)}/{len(valid_results)}")
        print(f"  평균 응답시간: {sum(latencies)//len(latencies)}ms")
        print(f"  최소: {min(latencies)}ms")
        print(f"  최대: {max(latencies)}ms")

    print("\n상세 결과:")
    for r in results:
        status = "✅" if r.get("valid_json") else "❌"
        print(f"  {status} {r['scenario']}: {r.get('latency_ms', 'N/A')}ms")


if __name__ == "__main__":
    main()
