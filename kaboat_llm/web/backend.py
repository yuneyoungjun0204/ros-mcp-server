#!/usr/bin/env python3
"""
KABOAT Web Control Backend
웹에서 시뮬레이터, rosbridge, 노드 등을 실행/중지
"""

import os
import signal
import subprocess
import asyncio
from pathlib import Path
from typing import Dict, Optional

# .env 파일에서 환경변수 로드
env_file = Path(__file__).parent / ".env"
if env_file.exists():
    for line in env_file.read_text().strip().split("\n"):
        if "=" in line and not line.startswith("#"):
            key, val = line.split("=", 1)
            os.environ.setdefault(key.strip(), val.strip())

from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

app = FastAPI(title="KABOAT Control API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# 프로세스 관리
processes: Dict[str, subprocess.Popen] = {}

# linuxbrew 제외한 PATH (ROS humble Python 충돌 방지)
CLEAN_PATH = "/usr/bin:/bin:/usr/local/bin:/usr/sbin:/sbin"
ROS_ENV = "source /opt/ros/humble/setup.bash && source /home/yune/vrx_ws/install/setup.bash"

# 실행 명령 정의
LAUNCH_COMMANDS = {
    "simulator": {
        "name": "KABOAT Simulator",
        "cmd": "ros2 launch kaboat_pkg kaboat_sim.launch.py",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "rosbridge": {
        "name": "ROSBridge",
        "cmd": "ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "kaboat_core": {
        "name": "KABOAT Core",
        "cmd": "ros2 launch kaboat_autonomous kaboat_core.launch.py",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "llm_interface": {
        "name": "LLM Interface",
        "cmd": "ros2 run kaboat_autonomous llm_interface",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "mission_runner": {
        "name": "Mission Runner",
        "cmd": "ros2 run kaboat_autonomous mission_runner",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "action_dispatcher": {
        "name": "Action Dispatcher",
        "cmd": "ros2 run kaboat_autonomous action_dispatcher",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "autonomous_viz": {
        "name": "Autonomous + Viz",
        "cmd": "ros2 launch kaboat_autonomous autonomous.launch.py",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "cluster_visualizer": {
        "name": "Cluster Visualizer",
        "cmd": "ros2 run kaboat_autonomous cluster_visualizer",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "camera_bridge": {
        "name": "Camera Bridge",
        "cmd": "ros2 run ros_gz_image image_bridge /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image --ros-args -r /world/kaboat_course/model/wamv/link/wamv/base_link/sensor/front_left_camera_sensor/image:=/wamv/sensors/camera/image_raw",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "image_republish": {
        "name": "Image Republish",
        "cmd": "ros2 run image_transport republish raw --ros-args -r in:=/wamv/sensors/camera/image_raw -r out/compressed:=/wamv/sensors/camera/image_raw/compressed",
        "cwd": None,
        "env_setup": ROS_ENV,
    },
    "yolo_detector": {
        "name": "YOLO Detector",
        "cmd": "python3 /home/yune/ros-mcp-server/kaboat_llm/perception/yolo_detector.py --ros-args -p use_yolo:=true -p hybrid_mode:=true -p yolo_model:=/home/yune/ros-mcp-server/kaboat_llm/models/roboboat_buoy.pt -p confidence_threshold:=0.05",
        "cwd": "/home/yune/ros-mcp-server/kaboat_llm/perception",
        "env_setup": ROS_ENV,
    },
    "gemini_mission": {
        "name": "Gemini Mission",
        "cmd": "python3 /home/yune/ros-mcp-server/kaboat_llm/scripts/run_mission_with_llm.py --mission gate_search",
        "cwd": "/home/yune/ros-mcp-server/kaboat_llm/scripts",
        "env_setup": ROS_ENV,
    },
    "full_mission": {
        "name": "Full Mission",
        "cmd": "python3 /home/yune/ros-mcp-server/kaboat_llm/scripts/run_full_mission.py",
        "cwd": "/home/yune/ros-mcp-server/kaboat_llm/scripts",
        "env_setup": ROS_ENV,
    },
    "gemini_mcp": {
        "name": "Gemini MCP",
        "cmd": "python3 -c \"from kaboat_llm.gemini_mcp_bridge import GeminiMCPBridge; b=GeminiMCPBridge(); b.connect('127.0.0.1',9090); import time; time.sleep(999999)\"",
        "cwd": "/home/yune/ros-mcp-server",
        "env_setup": ROS_ENV,
    },
}

# 풀 미션 순서
FULL_MISSION_SEQUENCE = [
    {"type": "gate_search", "name": "게이트 탐색/통과"},
    {"type": "buoy_orbit", "name": "부표 선회", "params": {"color": "red", "direction": "cw"}},
    {"type": "hopping_tour", "name": "호핑 투어"},
    {"type": "docking", "name": "도킹"},
]

# 미션 웨이포인트 (로컬 좌표 - settings.py의 GPS를 변환한 값)
# REF: lat=-33.722759, lon=150.674028 → 로컬 원점
MISSION_WAYPOINTS = {
    'start': {'x': 0, 'y': 0, 'name': '시작점', 'color': '#00ff88'},
    'gate_start': {'x': -4, 'y': 13, 'name': '게이트 시작', 'color': '#ff6b6b'},
    'gate_end': {'x': -3, 'y': 95, 'name': '게이트 끝', 'color': '#ff6b6b'},
    'buoy_orbit': {'x': 0, 'y': 123, 'name': '부표 선회', 'color': '#ffd93d'},
    'hopping': {'x': 43, 'y': 112, 'name': '호핑 투어', 'color': '#6bcb77'},
    'dock_start': {'x': 47, 'y': 22, 'name': '도킹 시작', 'color': '#4d96ff'},
}

MISSION_SEQUENCE_ORDER = ['start', 'gate_start', 'gate_end', 'buoy_orbit', 'hopping', 'dock_start']


class LaunchRequest(BaseModel):
    mission_type: Optional[str] = "gate_search"
    extra_args: Optional[str] = ""


WEB_DIR = Path(__file__).parent

@app.get("/")
async def root():
    return FileResponse(WEB_DIR / "index.html")


class ImageAnalysisRequest(BaseModel):
    image: str  # base64 encoded image
    prompt: Optional[str] = "이 이미지를 분석해주세요."


@app.post("/api/analyze_image")
async def analyze_image(request: ImageAnalysisRequest):
    """Gemini Vision으로 이미지 분석"""
    import time
    import base64

    api_key = os.environ.get("GOOGLE_API_KEY") or os.environ.get("GEMINI_API_KEY")
    if not api_key:
        return {"error": "GOOGLE_API_KEY 환경변수 필요"}

    try:
        import google.generativeai as genai
        genai.configure(api_key=api_key)

        model = genai.GenerativeModel('gemini-3.5-flash-lite')

        # Base64 이미지를 bytes로 변환
        image_data = base64.b64decode(request.image)

        start_time = time.time()
        response = model.generate_content([
            request.prompt,
            {"mime_type": "image/jpeg", "data": image_data}
        ])
        latency_ms = int((time.time() - start_time) * 1000)

        return {
            "response": response.text,
            "latency_ms": latency_ms
        }
    except Exception as e:
        return {"error": str(e)}


class BuoyDetectionRequest(BaseModel):
    image: str  # base64 encoded image
    colors: Optional[list] = ['red', 'green', 'blue', 'yellow', 'black']


@app.post("/api/detect_buoys")
async def detect_buoys(request: BuoyDetectionRequest):
    """HSV 기반 부표 감지"""
    import time
    import base64
    import cv2
    import numpy as np
    import sys
    from pathlib import Path

    # perception 모듈 import
    perception_path = str(Path(__file__).parent.parent / "perception")
    if perception_path not in sys.path:
        sys.path.insert(0, perception_path)

    try:
        from color_detector import ColorBuoyDetector

        start_time = time.time()

        # Base64 → OpenCV 이미지
        image_data = base64.b64decode(request.image)
        nparr = np.frombuffer(image_data, np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if frame is None:
            return {"error": "이미지 디코딩 실패"}

        # 부표 감지
        detector = ColorBuoyDetector(
            min_area=300,
            enabled_colors=request.colors
        )
        detections = detector.detect(frame)

        latency_ms = int((time.time() - start_time) * 1000)

        # 결과 포맷팅
        results = []
        for det in detections:
            results.append({
                "color": det.color,
                "center": list(det.center),
                "area": det.area,
                "bbox": list(det.bbox),
                "normalized_x": round(det.normalized_x, 3)
            })

        # 시각화된 이미지 생성
        vis_frame, _ = detector.detect_and_draw(frame)
        _, buffer = cv2.imencode('.jpg', vis_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        vis_base64 = base64.b64encode(buffer).decode('utf-8')

        return {
            "detections": results,
            "count": len(results),
            "latency_ms": latency_ms,
            "visualized_image": vis_base64
        }

    except Exception as e:
        import traceback
        return {"error": str(e), "trace": traceback.format_exc()}


@app.post("/api/autonomous/start")
async def autonomous_start():
    """자율 모드 시작 - ROS 토픽으로 발행"""
    try:
        clean_env = os.environ.copy()
        clean_env["PATH"] = CLEAN_PATH

        result = subprocess.run(
            ["/bin/bash", "-c",
             f"{ROS_ENV} && "
             "ros2 topic pub --once /autonomous_control std_msgs/String "
             "'{\"data\": \"{\\\"action\\\": \\\"start\\\"}\"}'"
            ],
            env=clean_env,
            capture_output=True,
            text=True,
            timeout=10
        )
        return {"status": "started", "output": result.stdout or result.stderr}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/api/autonomous/stop")
async def autonomous_stop():
    """자율 모드 중지"""
    try:
        clean_env = os.environ.copy()
        clean_env["PATH"] = CLEAN_PATH

        result = subprocess.run(
            ["/bin/bash", "-c",
             f"{ROS_ENV} && "
             "ros2 topic pub --once /autonomous_control std_msgs/String "
             "'{\"data\": \"{\\\"action\\\": \\\"stop\\\"}\"}'"
            ],
            env=clean_env,
            capture_output=True,
            text=True,
            timeout=10
        )
        return {"status": "stopped", "output": result.stdout or result.stderr}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.get("/api/autonomous/status")
async def autonomous_status():
    """자율 모드 상태 조회 - /mission_status 토픽에서 가져옴"""
    try:
        clean_env = os.environ.copy()
        clean_env["PATH"] = CLEAN_PATH

        result = subprocess.run(
            ["/bin/bash", "-c",
             f"{ROS_ENV} && "
             "timeout 2 ros2 topic echo /mission_status --once 2>/dev/null || echo '{}'"
            ],
            env=clean_env,
            capture_output=True,
            text=True,
            timeout=5
        )
        # YAML 형식 파싱 시도
        output = result.stdout.strip()
        if 'data:' in output:
            import re
            match = re.search(r"data:\s*'(.+)'", output, re.DOTALL)
            if match:
                return {"status": "ok", "data": json.loads(match.group(1).replace("\\", ""))}
        return {"status": "ok", "data": {"autonomous_mode": False, "mission_status": "unknown"}}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/api/release_boat")
async def release_boat():
    """WAM-V를 플랫폼에서 해제 (시작 시 고정 문제 해결)"""
    try:
        # linuxbrew 제외한 환경
        clean_env = os.environ.copy()
        clean_env["PATH"] = CLEAN_PATH

        result = subprocess.run(
            ["/bin/bash", "-c",
             "source /opt/ros/humble/setup.bash && "
             "gz topic -t /vrx/release -m gz.msgs.Empty -p ''"],
            env=clean_env,
            capture_output=True,
            text=True,
            timeout=10
        )
        return {"status": "released", "output": result.stdout or result.stderr}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/api/unpause_sim")
async def unpause_sim():
    """시뮬레이션 unpause"""
    try:
        clean_env = os.environ.copy()
        clean_env["PATH"] = CLEAN_PATH

        result = subprocess.run(
            ["/bin/bash", "-c",
             "source /opt/ros/humble/setup.bash && "
             "gz service -s /world/kaboat_course/control "
             "--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
             "--req 'pause: false' --timeout 5000"],
            env=clean_env,
            capture_output=True,
            text=True,
            timeout=10
        )
        return {"status": "unpaused", "output": result.stdout or result.stderr}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.get("/api/waypoints")
async def get_waypoints():
    """미션 웨이포인트 반환"""
    return {
        "waypoints": MISSION_WAYPOINTS,
        "sequence": MISSION_SEQUENCE_ORDER,
    }


@app.get("/api/status")
async def get_status():
    """모든 프로세스 상태"""
    status = {}
    for name, config in LAUNCH_COMMANDS.items():
        proc = processes.get(name)
        if proc and proc.poll() is None:
            status[name] = {"running": True, "pid": proc.pid, "display_name": config["name"]}
        else:
            status[name] = {"running": False, "pid": None, "display_name": config["name"]}
    return status


# 속도 배율 설정
speed_multiplier = 1.0

class SpeedRequest(BaseModel):
    multiplier: float

@app.get("/api/speed")
async def get_speed():
    """현재 속도 배율 조회"""
    return {"multiplier": speed_multiplier}

@app.post("/api/speed")
async def set_speed(request: SpeedRequest):
    """속도 배율 설정 (0.25 ~ 5.0)"""
    global speed_multiplier
    if request.multiplier < 0.25 or request.multiplier > 5.0:
        raise HTTPException(400, "Speed multiplier must be between 0.25 and 5.0")
    speed_multiplier = request.multiplier

    # settings.py 파일도 업데이트
    settings_path = Path("/home/yune/vrx_ws/src/kaboat_autonomous/config/settings.py")
    if settings_path.exists():
        content = settings_path.read_text()
        import re
        content = re.sub(r'SPEED_MULTIPLIER = [\d.]+', f'SPEED_MULTIPLIER = {speed_multiplier}', content)
        settings_path.write_text(content)

    return {"status": "ok", "multiplier": speed_multiplier}


# === Mission Settings API ===

class SettingsRequest(BaseModel):
    value: float

@app.post("/api/settings/goal_range")
async def set_goal_range(request: SettingsRequest):
    """도착 판정 거리 설정 (1-25m) - action_dispatcher로 명령 전송"""
    value = max(1.0, min(25.0, request.value))
    cmd = json.dumps({"action": "set_goal_range", "value": value})
    await publish_to_topic("/llm_action", "std_msgs/String", {"data": cmd})
    return {"status": "ok", "goal_range": value}

@app.post("/api/settings/move_speed")
async def set_move_speed(request: SettingsRequest):
    """이동 속도 설정 (1-10) - action_dispatcher로 명령 전송"""
    value = max(1.0, min(10.0, request.value))
    cmd = json.dumps({"action": "set_speed", "value": value})
    await publish_to_topic("/llm_action", "std_msgs/String", {"data": cmd})
    return {"status": "ok", "move_speed": value}


@app.post("/api/launch/{name}")
async def launch_process(name: str, request: LaunchRequest = None):
    """프로세스 실행"""
    if name not in LAUNCH_COMMANDS:
        raise HTTPException(404, f"Unknown process: {name}")

    if name in processes and processes[name].poll() is None:
        return {"status": "already_running", "pid": processes[name].pid}

    config = LAUNCH_COMMANDS[name]
    cmd = config["cmd"]

    # 미션 타입 적용
    if request and name == "gemini_mission" and request.mission_type:
        cmd = cmd.replace("gate_search", request.mission_type)
    if request and request.extra_args:
        cmd += f" {request.extra_args}"

    # bash에서 환경 설정 후 실행 (linuxbrew 제외)
    full_cmd = f"{config['env_setup']} && {cmd}"

    # linuxbrew 제외한 환경
    clean_env = os.environ.copy()
    clean_env["PATH"] = CLEAN_PATH

    try:
        proc = subprocess.Popen(
            full_cmd,
            shell=True,
            executable="/bin/bash",
            cwd=config["cwd"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=clean_env,
            preexec_fn=os.setsid,  # 프로세스 그룹 생성
        )
        processes[name] = proc

        # 시뮬레이터 시작 시 5초 후 자동 릴리즈
        if name == "simulator":
            asyncio.create_task(auto_release_boat())

        return {"status": "started", "pid": proc.pid, "command": cmd}
    except Exception as e:
        raise HTTPException(500, str(e))


async def auto_release_boat():
    """시뮬레이터 시작 후 5초 뒤 자동으로 보트 릴리즈"""
    await asyncio.sleep(5)
    clean_env = os.environ.copy()
    clean_env["PATH"] = CLEAN_PATH
    try:
        subprocess.run(
            ["/bin/bash", "-c",
             "source /opt/ros/humble/setup.bash && "
             "gz topic -t /vrx/release -m gz.msgs.Empty -p ''"],
            env=clean_env,
            capture_output=True,
            timeout=10
        )
        print("[AUTO] Boat released from platform")
    except Exception as e:
        print(f"[AUTO] Release failed: {e}")


@app.post("/api/stop/{name}")
async def stop_process(name: str):
    """프로세스 중지"""
    if name not in processes:
        return {"status": "not_found"}

    proc = processes[name]
    if proc.poll() is not None:
        return {"status": "already_stopped"}

    try:
        # 프로세스 그룹 전체 종료
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5)
        return {"status": "stopped"}
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        return {"status": "killed"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


@app.post("/api/stop_all")
async def stop_all():
    """모든 프로세스 중지"""
    results = {}
    for name in list(processes.keys()):
        results[name] = await stop_process(name)
    return results


@app.get("/api/logs/{name}")
async def get_logs(name: str, lines: int = 50):
    """프로세스 로그 (최근 N줄)"""
    if name not in processes:
        return {"logs": [], "error": "not_found"}

    proc = processes[name]
    if proc.stdout:
        try:
            # non-blocking read
            import select
            logs = []
            while select.select([proc.stdout], [], [], 0)[0]:
                line = proc.stdout.readline()
                if line:
                    logs.append(line.decode('utf-8', errors='ignore').strip())
                else:
                    break
            return {"logs": logs[-lines:]}
        except:
            return {"logs": [], "error": "read_error"}
    return {"logs": []}


# ===== Gemini MCP Bridge API =====
gemini_bridge = None
gemini_history = []


class GeminiChatRequest(BaseModel):
    message: str
    include_image: Optional[bool] = False


@app.post("/api/gemini_mcp/chat")
async def gemini_mcp_chat(request: GeminiChatRequest):
    """Gemini MCP Bridge로 대화 - 사고 과정 및 이미지 포함"""
    global gemini_bridge, gemini_history
    import sys
    import time
    import base64

    sys.path.insert(0, "/home/yune/ros-mcp-server")

    try:
        from kaboat_llm.gemini_mcp_bridge import GeminiMCPBridge

        if gemini_bridge is None:
            gemini_bridge = GeminiMCPBridge()
            if not gemini_bridge.connect("127.0.0.1", 9090):
                return {"error": "rosbridge 연결 실패"}

        start_time = time.time()
        function_calls = []
        image_base64 = None

        # 이미지 포함 요청 시 카메라 이미지 캡처
        if request.include_image:
            try:
                gemini_bridge.rosbridge.send({
                    "op": "subscribe",
                    "topic": "/wamv/sensors/camera/image_raw/compressed",
                    "type": "sensor_msgs/CompressedImage",
                    "id": "camera_capture"
                })
                img_msg = gemini_bridge.rosbridge.receive(timeout=2.0)
                if img_msg and "msg" in img_msg:
                    image_base64 = img_msg["msg"].get("data", "")
                # 구독 해제
                gemini_bridge.rosbridge.send({
                    "op": "unsubscribe",
                    "topic": "/wamv/sensors/camera/image_raw/compressed",
                    "id": "camera_capture"
                })
            except Exception as img_err:
                print(f"[GeminiMCP] Image capture failed: {img_err}")

        response = gemini_bridge.chat(
            request.message,
            callback=lambda name, args, result: function_calls.append({
                "function": name,
                "args": args,
                "result": str(result)[:200]
            })
        )

        latency_ms = int((time.time() - start_time) * 1000)

        entry = {
            "timestamp": time.strftime("%H:%M:%S"),
            "message": request.message,
            "function_calls": function_calls,
            "response": response,
            "latency_ms": latency_ms,
            "image": image_base64
        }
        gemini_history.insert(0, entry)
        gemini_history = gemini_history[:10]

        return entry

    except Exception as e:
        import traceback
        return {"error": str(e), "trace": traceback.format_exc()}


@app.get("/api/gemini_mcp/status")
async def gemini_mcp_status():
    """Gemini MCP Bridge 상태"""
    global gemini_bridge
    connected = gemini_bridge is not None and gemini_bridge.rosbridge.ws is not None
    return {
        "connected": connected,
        "history_count": len(gemini_history),
        "recent": gemini_history[:3] if gemini_history else []
    }


@app.post("/api/gemini_mcp/disconnect")
async def gemini_mcp_disconnect():
    """Gemini MCP Bridge 연결 해제"""
    global gemini_bridge
    if gemini_bridge:
        gemini_bridge.disconnect()
        gemini_bridge = None
    return {"status": "disconnected"}


# 정적 파일 서빙
app.mount("/static", StaticFiles(directory=str(WEB_DIR)), name="static")


if __name__ == "__main__":
    print("=" * 50)
    print("KABOAT Web Control Backend")
    print("http://localhost:8000")
    print("=" * 50)
    uvicorn.run(app, host="0.0.0.0", port=8000)
