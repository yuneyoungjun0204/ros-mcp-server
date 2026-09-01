#!/usr/bin/env python3
"""
YOLO/HSV 기반 객체 감지 ROS2 노드

카메라 이미지에서 부표 및 객체를 감지하여 ROS 토픽으로 발행

사용법 (직접 실행):
    python3 yolo_detector.py

사용법 (ROS 환경에서):
    from kaboat_llm.perception.yolo_detector import ObjectDetectorNode
    node = ObjectDetectorNode()
    rclpy.spin(node)

파라미터:
    use_yolo: YOLO 사용 여부 (기본값: False, HSV 색상 감지 사용)
    camera_topic: 카메라 토픽 (기본값: /wamv/sensors/cameras/front_left_camera_sensor/image)
    publish_rate: 감지 발행 주기 Hz (기본값: 10.0)

토픽:
    구독: camera_topic (sensor_msgs/Image)
    발행: /detected_objects (std_msgs/String, JSON)
          /detection_image (sensor_msgs/Image, 시각화)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
from typing import List, Dict, Any

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("[WARN] ultralytics not installed, YOLO disabled")

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False
    print("[WARN] onnxruntime not installed, ONNX disabled")

try:
    from .color_detector import ColorBuoyDetector, BuoyDetection, YOLO_LABELS, LABEL_TO_ID
except ImportError:
    import sys
    from pathlib import Path
    # Add repo root to sys.path for direct script execution
    _repo_root = str(Path(__file__).resolve().parents[2])
    if _repo_root not in sys.path:
        sys.path.insert(0, _repo_root)
    from kaboat_llm.perception.color_detector import ColorBuoyDetector, BuoyDetection, YOLO_LABELS, LABEL_TO_ID


class ObjectDetectorNode(Node):
    """
    객체 감지 ROS2 노드

    HSV 색상 감지 또는 YOLO 모델 사용 가능
    """

    def __init__(self):
        super().__init__('object_detector')

        # 파라미터 선언
        self.declare_parameter('use_yolo', False)
        self.declare_parameter('hybrid_mode', True)  # YOLO + HSV 병합 모드
        self.declare_parameter('yolo_model', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_topic', '/wamv/sensors/camera/image_raw')
        self.declare_parameter('enabled_colors', ['red', 'green', 'blue', 'yellow', 'black'])
        self.declare_parameter('min_area', 500)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # 파라미터 로드
        self.use_yolo = self.get_parameter('use_yolo').value
        self.hybrid_mode = self.get_parameter('hybrid_mode').value
        self.yolo_model_path = self.get_parameter('yolo_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        camera_topic = self.get_parameter('camera_topic').value
        enabled_colors = self.get_parameter('enabled_colors').value
        min_area = self.get_parameter('min_area').value

        # CV Bridge
        self.bridge = CvBridge()

        # 감지기 초기화
        self.yolo_model = None
        self.onnx_session = None
        self.onnx_labels = []
        self.onnx_input_size = (640, 352)
        self.color_detector = ColorBuoyDetector(
            min_area=min_area,
            enabled_colors=enabled_colors
        )

        # 모델 경로 우선순위: roboboat_buoy.pt > yolov8n.pt > ONNX
        default_roboboat = str(Path(__file__).parent.parent / 'models' / 'roboboat_buoy.pt')
        default_onnx = str(Path(__file__).parent / 'models' / 'sign-simplified.onnx')
        default_pt = str(Path(__file__).parent.parent / 'models' / 'yolov8n.pt')

        if not self.yolo_model_path:
            if Path(default_roboboat).exists() and YOLO_AVAILABLE:
                self.yolo_model_path = default_roboboat
            elif Path(default_onnx).exists() and ONNX_AVAILABLE:
                self.yolo_model_path = default_onnx
            elif Path(default_pt).exists() and YOLO_AVAILABLE:
                self.yolo_model_path = default_pt

        if self.use_yolo and self.yolo_model_path:
            if self.yolo_model_path.endswith('.onnx') and ONNX_AVAILABLE:
                try:
                    self.onnx_session = ort.InferenceSession(
                        self.yolo_model_path,
                        providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
                    )
                    self._load_onnx_config()
                    self.get_logger().info(f'ONNX model loaded: {self.yolo_model_path}')
                    self.get_logger().info(f'Classes: {len(self.onnx_labels)} - {self.onnx_labels[:5]}...')
                except Exception as e:
                    self.get_logger().error(f'Failed to load ONNX: {e}')
                    self.use_yolo = False
            elif YOLO_AVAILABLE:
                try:
                    self.yolo_model = YOLO(self.yolo_model_path, task='detect')
                    model_classes = list(self.yolo_model.names.values())
                    self.get_logger().info(f'YOLO model loaded: {self.yolo_model_path}')
                    self.get_logger().info(f'Classes ({len(model_classes)}): {model_classes}')
                except Exception as e:
                    self.get_logger().error(f'Failed to load YOLO: {e}')
                    self.use_yolo = False
            else:
                self.get_logger().error('No YOLO/ONNX runtime available')
                self.use_yolo = False

        if not self.use_yolo:
            self.get_logger().info('Using HSV color detection')

        # Publishers
        self.detection_pub = self.create_publisher(String, '/detected_objects', 10)
        self.image_pub = self.create_publisher(Image, '/detection_image', 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # 상태
        self.last_frame = None
        self.last_detections = []
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.last_detection_time = 0.0

        # Rate limiting
        publish_rate = self.get_parameter('publish_rate').value
        self.detection_interval = 1.0 / publish_rate if publish_rate > 0 else 0.0

        self.get_logger().info(f'Object Detector started, subscribing to: {camera_topic}')
        self.get_logger().info(f'Detection rate limit: {publish_rate} Hz')

    def _load_onnx_config(self):
        """ONNX 모델 설정 로드 (classes.json)"""
        from pathlib import Path
        config_path = Path(self.yolo_model_path).parent / 'classes.json'
        if config_path.exists():
            with open(config_path) as f:
                config = json.load(f)
                self.onnx_labels = config.get('labels', [])
                nn_config = config.get('nn_config', {})
                input_size = nn_config.get('input_size', '640x352')
                w, h = map(int, input_size.split('x'))
                self.onnx_input_size = (w, h)
                self.confidence_threshold = nn_config.get('confidence_threshold', self.confidence_threshold)

        # 출력 형식 확인 (OAK-D 특수 형식 감지)
        outputs = self.onnx_session.get_outputs()
        if outputs and 'yolov6r2' in outputs[0].name:
            out_shape = outputs[0].shape
            if len(out_shape) == 4 and out_shape[1] == 9:
                self.onnx_format = 'oakd_yolov6'
                self.get_logger().warn('OAK-D YOLOv6 format detected - using anchor-based decoding')
            else:
                self.onnx_format = 'standard'
        else:
            self.onnx_format = 'standard'

    def image_callback(self, msg: Image):
        """카메라 이미지 콜백 (rate-limited)"""
        now = time.time()

        # Rate limiting: skip if too soon since last detection
        if self.detection_interval > 0 and (now - self.last_detection_time) < self.detection_interval:
            return

        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_frame = cv_image
            self.last_detection_time = now

            # 감지 수행
            detections = []

            # YOLO 감지
            if self.use_yolo and self.onnx_session:
                detections = self._detect_onnx(cv_image)
            elif self.use_yolo and self.yolo_model:
                detections = self._detect_yolo(cv_image)

            # 하이브리드 모드: HSV 결과 병합 (YOLO 결과가 없거나 hybrid_mode 활성화 시)
            if self.hybrid_mode or not detections:
                hsv_detections = self._detect_color(cv_image)
                if self.use_yolo and detections:
                    # YOLO + HSV 병합 (중복 제거)
                    detections = self._merge_detections(detections, hsv_detections)
                else:
                    detections = hsv_detections

            self.last_detections = detections

            # JSON 발행
            detection_msg = String()
            detection_msg.data = json.dumps({
                'timestamp': time.time(),
                'frame_id': msg.header.frame_id,
                'detections': detections,
                'fps': round(self.fps, 1)
            })
            self.detection_pub.publish(detection_msg)

            # 시각화 이미지 발행
            vis_image = self._visualize(cv_image, detections)
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.image_pub.publish(vis_msg)

            # FPS 계산
            self.frame_count += 1
            now = time.time()
            if now - self.last_fps_time >= 1.0:
                self.fps = self.frame_count / (now - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = now

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')

    def _detect_yolo(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """YOLO로 객체 감지 (모델의 네이티브 라벨 사용)"""
        results = self.yolo_model(image, conf=self.confidence_threshold, verbose=False)
        detections = []

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])

                # 모델의 네이티브 라벨 사용 (roboboat_buoy.pt 등 지원)
                cls_name = result.names.get(cls_id, f'class_{cls_id}')

                center_x = (x1 + x2) / 2 / image.shape[1]  # normalized
                center_y = (y1 + y2) / 2 / image.shape[0]

                detections.append({
                    'label': cls_name,
                    'class_id': cls_id,
                    'confidence': round(conf, 3),
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'center_normalized': [round(center_x, 3), round(center_y, 3)],
                    'source': 'yolo'
                })

        return detections

    def _detect_onnx(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """ONNX 모델로 객체 감지 (YOLOv6 형식)"""
        orig_h, orig_w = image.shape[:2]
        input_w, input_h = self.onnx_input_size

        # 전처리: 리사이즈 및 정규화
        resized = cv2.resize(image, (input_w, input_h))
        blob = resized.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)  # HWC -> CHW
        blob = np.expand_dims(blob, axis=0)  # NCHW

        # 추론
        input_name = self.onnx_session.get_inputs()[0].name
        outputs = self.onnx_session.run(None, {input_name: blob})

        detections = []

        # OAK-D YOLOv6 형식 처리
        if getattr(self, 'onnx_format', 'standard') == 'oakd_yolov6':
            strides = [8, 16, 32]
            anchors_per_scale = [
                [[10, 13], [16, 30], [33, 23]],
                [[30, 61], [62, 45], [59, 119]],
                [[116, 90], [156, 198], [373, 326]]
            ]
            for idx, output in enumerate(outputs):
                dets = self._parse_oakd_yolov6(output, orig_w, orig_h, input_w, input_h,
                                               strides[idx], anchors_per_scale[idx])
                detections.extend(dets)
        else:
            # 표준 YOLO 출력 파싱
            for output in outputs:
                detections.extend(self._parse_yolov6_output(output, orig_w, orig_h, input_w, input_h))

        # NMS 적용
        detections = self._nms(detections, iou_threshold=0.5)
        return detections

    def _parse_oakd_yolov6(self, output: np.ndarray, orig_w: int, orig_h: int,
                          input_w: int, input_h: int, stride: int,
                          anchors: List) -> List[Dict[str, Any]]:
        """OAK-D YOLOv6 출력 파싱 (9채널 = 3앵커 * 3값)"""
        detections = []

        # output shape: [1, 9, H, W]
        output = output[0]  # [9, H, W]
        _, grid_h, grid_w = output.shape

        # 9채널을 3앵커로 분리: [3, 3, H, W]
        output = output.reshape(3, 3, grid_h, grid_w)

        scale_x = orig_w / input_w
        scale_y = orig_h / input_h

        for anchor_idx in range(3):
            anchor_w, anchor_h = anchors[anchor_idx]

            for gy in range(grid_h):
                for gx in range(grid_w):
                    # [tx, ty, conf] 또는 [tx, ty, tw/th combined]
                    values = output[anchor_idx, :, gy, gx]

                    # 시그모이드 적용 (objectness)
                    conf = 1 / (1 + np.exp(-values[2]))

                    if conf < self.confidence_threshold:
                        continue

                    # 좌표 계산
                    tx, ty = values[0], values[1]
                    cx = (gx + 1 / (1 + np.exp(-tx))) * stride
                    cy = (gy + 1 / (1 + np.exp(-ty))) * stride

                    # 앵커 기반 크기
                    w = anchor_w * 2
                    h = anchor_h * 2

                    # 원본 좌표로 변환
                    x1 = int((cx - w/2) * scale_x)
                    y1 = int((cy - h/2) * scale_y)
                    x2 = int((cx + w/2) * scale_x)
                    y2 = int((cy + h/2) * scale_y)

                    # 경계 체크
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(orig_w, x2), min(orig_h, y2)

                    if x2 <= x1 or y2 <= y1:
                        continue

                    # 클래스 없이 일반 객체로 반환
                    detections.append({
                        'label': 'object',
                        'class_id': -1,
                        'confidence': round(float(conf), 3),
                        'bbox': [x1, y1, x2, y2],
                        'center_normalized': [round((x1+x2)/2/orig_w, 3), round((y1+y2)/2/orig_h, 3)],
                        'source': 'onnx_oakd'
                    })

        return detections

    def _parse_yolov6_output(self, output: np.ndarray, orig_w: int, orig_h: int,
                             input_w: int, input_h: int) -> List[Dict[str, Any]]:
        """YOLOv6 출력 파싱"""
        detections = []

        # 출력 형식: [batch, num_boxes, 5+num_classes] 또는 [batch, 5+num_classes, num_boxes]
        if len(output.shape) == 3:
            output = output[0]  # 배치 제거
            if output.shape[0] < output.shape[1]:
                output = output.T  # [5+classes, boxes] -> [boxes, 5+classes]

        num_classes = len(self.onnx_labels) if self.onnx_labels else (output.shape[-1] - 5)

        for row in output:
            if len(row) < 5 + num_classes:
                continue

            # [cx, cy, w, h, obj_conf, class_scores...]
            cx, cy, w, h = row[:4]
            obj_conf = row[4] if len(row) > 5 else 1.0
            class_scores = row[5:5+num_classes] if len(row) > 5 else row[4:4+num_classes]

            # 최고 클래스 선택
            cls_id = int(np.argmax(class_scores))
            cls_conf = float(class_scores[cls_id])
            conf = obj_conf * cls_conf if obj_conf < 1.0 else cls_conf

            if conf < self.confidence_threshold:
                continue

            # 좌표 변환 (입력 크기 -> 원본 크기)
            scale_x = orig_w / input_w
            scale_y = orig_h / input_h

            x1 = int((cx - w/2) * scale_x)
            y1 = int((cy - h/2) * scale_y)
            x2 = int((cx + w/2) * scale_x)
            y2 = int((cy + h/2) * scale_y)

            # 경계 체크
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(orig_w, x2), min(orig_h, y2)

            if x2 <= x1 or y2 <= y1:
                continue

            label = self.onnx_labels[cls_id] if cls_id < len(self.onnx_labels) else f'class_{cls_id}'
            center_x = (x1 + x2) / 2 / orig_w
            center_y = (y1 + y2) / 2 / orig_h

            detections.append({
                'label': label,
                'class_id': cls_id,
                'confidence': round(conf, 3),
                'bbox': [x1, y1, x2, y2],
                'center_normalized': [round(center_x, 3), round(center_y, 3)],
                'source': 'onnx'
            })

        return detections

    def _merge_detections(self, yolo_dets: List[Dict], hsv_dets: List[Dict],
                           iou_threshold: float = 0.3) -> List[Dict]:
        """YOLO와 HSV 감지 결과 병합 (YOLO 우선, 중복 HSV 제거)"""
        merged = list(yolo_dets)  # YOLO 결과 우선

        for hsv_det in hsv_dets:
            is_duplicate = False
            for yolo_det in yolo_dets:
                if self._iou(hsv_det['bbox'], yolo_det['bbox']) > iou_threshold:
                    is_duplicate = True
                    break
            if not is_duplicate:
                merged.append(hsv_det)

        return merged

    def _nms(self, detections: List[Dict], iou_threshold: float = 0.5) -> List[Dict]:
        """Non-Maximum Suppression"""
        if not detections:
            return []

        # 신뢰도로 정렬
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)
        keep = []

        while detections:
            best = detections.pop(0)
            keep.append(best)

            detections = [
                det for det in detections
                if self._iou(best['bbox'], det['bbox']) < iou_threshold
            ]

        return keep

    def _iou(self, box1: List[int], box2: List[int]) -> float:
        """Intersection over Union"""
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter

        return inter / union if union > 0 else 0

    def _detect_color(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """HSV 색상으로 객체 감지 (YOLO 호환 출력)"""
        buoy_detections = self.color_detector.detect(image)
        detections = []

        for det in buoy_detections:
            x, y, w, h = det.bbox
            detections.append({
                'label': det.label,  # YOLO 호환 라벨 (red_buoy, green_buoy, ...)
                'class_id': det.class_id,  # YOLO class index
                'confidence': 0.9,  # HSV는 신뢰도 고정
                'bbox': [x, y, x + w, y + h],
                'center_normalized': [round(det.normalized_x, 3),
                                     round(det.center[1] / image.shape[0], 3)],
                'area': det.area,
                'source': 'hsv'
            })

        return detections

    def _visualize(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """감지 결과 시각화"""
        vis = image.copy()

        color_map = {
            'red': (0, 0, 255),
            'green': (0, 255, 0),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'black': (128, 128, 128),
        }

        for det in detections:
            label = det['label']
            bbox = det['bbox']
            conf = det['confidence']

            # 색상 결정
            color = (0, 255, 0)  # 기본 초록
            for c_name, c_val in color_map.items():
                if c_name in label.lower():
                    color = c_val
                    break

            # 박스 그리기
            cv2.rectangle(vis, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)

            # 라벨 그리기
            text = f'{label} {conf:.2f}'
            cv2.putText(vis, text, (bbox[0], bbox[1] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # FPS 표시
        cv2.putText(vis, f'FPS: {self.fps:.1f}', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis, f'Mode: {"YOLO" if self.use_yolo else "HSV"}', (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(vis, f'Objects: {len(detections)}', (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return vis


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
