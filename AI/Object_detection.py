import cv2
import time
import json
import base64
import numpy as np
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ultralytics import YOLO
import threading
import multiprocessing as mp
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from pose_api import start_pose_api_server, update_pose
from datetime import datetime
import subprocess
import os

# COCO 17-kpt 인덱스 (Ultralytics 기준)
NOSE, LEYE, REYE, LEAR, REAR, LSH, RSH, LEL, REL, LWR, RWR, LHIP, RHIP, LKNE, RKNE, LANK, RANK = range(17)

def _mid(p1, p2):
    return (p1 + p2) / 2.0

def lie_score_from_keypoints(kpts_xy: np.ndarray, bbox, img_h: int):
    """
    kpts_xy: (17,2) ndarray with [x,y] in pixels
    bbox: [x1,y1,x2,y2] in pixels
    img_h: image height (for normalization)
    returns: (lie_score: float 0~1, detail: dict)
    """
    sh_c = _mid(kpts_xy[LSH], kpts_xy[RSH])
    hip_c = _mid(kpts_xy[LHIP], kpts_xy[RHIP])
    v = hip_c - sh_c
    v_norm = np.linalg.norm(v) + 1e-6
    v_unit = v / v_norm
    vertical = np.array([0.0, 1.0])
    verticality = abs(float(np.dot(v_unit, vertical)))
    horizontal_score = 1.0 - verticality

    x1, y1, x2, y2 = bbox
    w, h = max(1.0, x2 - x1), max(1.0, y2 - y1)
    aspect = w / h
    aspect_score = float(np.tanh(aspect - 1.2))

    ys = {
        "ankle": np.mean([kpts_xy[LANK,1], kpts_xy[RANK,1]]),
        "knee": np.mean([kpts_xy[LKNE,1], kpts_xy[RKNE,1]]),
        "hip": hip_c[1],
        "shoulder": sh_c[1],
        "nose": kpts_xy[NOSE,1],
    }
    order = ["ankle","knee","hip","shoulder","nose"]
    inversions = 0
    for i in range(len(order)-1):
        if not (ys[order[i]] > ys[order[i+1]]):
            inversions += 1
    order_score = inversions / 4.0

    key_idxs = [NOSE, LSH, RSH, LHIP, RHIP, LKNE, RKNE, LANK, RANK]
    yvals = kpts_xy[key_idxs,1]
    yspread = (float(np.max(yvals)) - float(np.min(yvals))) / (img_h + 1e-6)
    spread_score = 1.0 - float(np.clip(yspread / 0.5, 0, 1))

    lie_score = 0.45*horizontal_score + 0.25*aspect_score + 0.20*order_score + 0.10*spread_score
    return float(lie_score), {
        "horizontal": float(horizontal_score),
        "aspect": float(aspect_score),
        "order": float(order_score),
        "spread": float(spread_score),
        "aspect_raw": float(aspect)
    }

# 알림을 보낼 특정 클래스 이름 정의
ALERT_CLASSES = [
    "MATERIAL_COLLAPSE",
    "SMOKING_VIOLATION",
    "NOT_WEAR_WORKER",
    "PERSON_FALL"
]

SITUATION_MAPPING = {
    "MATERIAL_COLLAPSE": "COLLAPSE",
    "SMOKING_VIOLATION": "SMOKE",
    "NOT_WEAR_WORKER": "EQUIPMENT",
    "PERSON_FALL": "FALL"
}

ALERT_DURATION_THRESHOLD = 0.5
RESET_ALERT_AFTER = 60
KP_CONF_THRES = 0.35
MIN_VISIBLE_KPTS = 8
EDGE_MARGIN = 8
REQUIRE_LOWER_BODY_FOR_FALL = True
LIE_THRESHOLD = 0.55
MIN_HORIZONTAL_SCORE = 0.35
MIN_BOX_AREA = 80*80

# MQTT 설정
PORT = 1883
BROKER = "192.168.100.141"
AMR_SERIAL = "AMR001"
TOPIC = f"alert"

# paho.mqtt.client의 DeprecationWarning을 피하기 위해 client_id를 사용합니다.
# 하지만 이 부분은 main 프로세스에서 사용되지 않으므로 제거합니다.
# 대신, process_and_stream_frames 함수 안에서 local_client를 초기화합니다.
# client = mqtt.Client()

robot_pose = mp.Manager().dict({'x': None, 'y': None})

class PoseSubscriber(Node):
    def __init__(self, shared_pose):
        super().__init__('pose_subscriber')
        self.shared_pose = shared_pose

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.history = HistoryPolicy.KEEP_LAST
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            qos_profile)

    def listener_callback(self, msg):
        self.shared_pose['x'] = msg.pose.pose.position.x
        self.shared_pose['y'] = msg.pose.pose.position.y
        update_pose(self.shared_pose['x'], self.shared_pose['y'])
        self.get_logger().info(f"Received pose: x={self.shared_pose['x']}, y={self.shared_pose['y']}")

def start_ros2_node(shared_pose):
    rclpy.init()
    node = PoseSubscriber(shared_pose)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def send_alert_to_backend(client_instance, situation, image_base64="", x=None, y=None, detail=""):
    """
    AI에서 Backend로 알림 전송 (클라이언트 인스턴스를 인자로 받음)
    """
    global robot_pose
    if x is None or y is None:
        x_val = robot_pose['x'] if robot_pose['x'] is not None else 0.0
        y_val = robot_pose['y'] if robot_pose['y'] is not None else 0.0
    else:
        x_val = x
        y_val = y
    current_time_str = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

    message = {
        "serial" : "AMR001",
        "case": situation,
        "image": image_base64,
        "x": x_val,
        "y": y_val,
        "timeStamp": current_time_str
    }

    try:
        # QoS 1로 전송하고 브로커의 PUBACK 응답을 기다립니다.
        result = client_instance.publish(TOPIC, json.dumps(message), qos=1, retain=False)
        result.wait_for_publish()
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print(f"[AI -> Backend] 상황: {situation}, 상세: {detail}, 로봇 위치: x={x_val}, y={y_val}")
            print("✅ MQTT 브로커로부터 메시지 수신 확인됨.")
            return True
        else:
            print(f"[ERROR] MQTT 메시지 전송 실패 (리턴 코드: {result.rc})")
            return False

    except Exception as e:
        print(f"[ERROR] 알림 전송 실패: {e}")
        return False

def encode_frame_to_base64(frame):
    """이미지 프레임을 Base64로 인코딩"""
    try:
        _, buffer = cv2.imencode('.jpg', frame)
        return base64.b64encode(buffer).decode("utf-8")
    except Exception as e:
        print(f"[ERROR] 이미지 인코딩 실패: {e}")
        return ""

def get_situation_detail(situation):
    """상황별 상세 정보 반환"""
    detail_mapping = {
        "COLLAPSE": "적재 물류가 붕괴되었습니다. 즉시 확인이 필요합니다.",
        "SMOKE": "작업장 내에서 흡연이 감지되었습니다. 안전 규정을 확인하세요.",
        "EQUIPMENT": "안전장비를 착용하지 않은 작업자가 감지되었습니다.",
        "FALL": "쓰러져 있는 작업자가 감지되었습니다. 응급 조치가 필요합니다."
    }
    return detail_mapping.get(situation, "알 수 없는 상황이 발생했습니다.")

def capture_frames(queue):
    """카메라에서 프레임을 캡처하여 큐에 넣는 프로세스"""
    cap = cv2.VideoCapture("/dev/video0")
    if not cap.isOpened():
        print("[ERROR] 카메라를 열 수 없습니다.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, 30)

    current_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    current_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    current_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"✅ 카메라 캡처 프로세스 시작...")
    print(f"   - 설정된 해상도: {current_width}x{current_height}")
    print(f"   - 설정된 프레임: {current_fps}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] 프레임을 읽을 수 없습니다.")
            break
        
        if not queue.full():
            queue.put(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    print("❌ 카메라 캡처 프로세스 종료")

def process_and_stream_frames(queue):
    """큐에서 프레임을 가져와 AI 추론 및 통신, 그리고 RTSP 스트리밍을 수행하는 프로세스"""
    # 이 프로세스에서 MQTT 클라이언트를 생성하고 연결합니다.
    local_client = mqtt.Client()
    try:
        local_client.connect(BROKER, PORT, 60)
        local_client.loop_start()
        print("✅ MQTT 연결 성공")
    except Exception as e:
        print(f"[ERROR] MQTT 연결 실패: {e}")
        return

    try:
        model = YOLO("best.pt").to("cuda")
        pose_model = YOLO("yolo11n-pose.pt").to("cuda")
        print("✅ YOLO 모델 로드 완료")
    except Exception as e:
        print(f"[ERROR] YOLO 모델 로드 실패: {e}")
        local_client.loop_stop()
        return
    
    frame_width, frame_height = 1280, 720
    ffmpeg_cmd = [
        "ffmpeg",
        "-y",
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "-s", f"{frame_width}x{frame_height}",
        "-r", "30",
        "-i", "-",
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-rtsp_transport", "tcp",
        "-an",
        "-f", "rtsp",
        "rtsp://localhost:8554/mystream"
    ]
    
    print("🎥 FFmpeg 스트리밍 프로세스 시작 중...")
    stream_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    print(f"✅ AI 알림 시스템 시작 - AMR: {AMR_SERIAL}")
    print(f"✅ MQTT 토픽: {TOPIC}")
    print("✅ 감지 대상:", ALERT_CLASSES)

    local_detection_states = {
        cls_name: {'start_time': None, 'alert_sent': False, 'alert_sent_time': None}
        for cls_name in ALERT_CLASSES
    }

    try:
        while True:
            try:
                frame = queue.get(timeout=1)
            except mp.TimeoutError:
                continue
                
            current_time = time.time()
            detected_classes_in_current_frame = set()
            img_h, img_w = frame.shape[:2]

            try:
                results = model(frame, stream=True)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = model.names[class_id]

                        if class_name in ALERT_CLASSES and confidence > 0.7:
                            detected_classes_in_current_frame.add(class_name)
                            color = (0, 0, 255)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            label = f"{class_name} {confidence:.2f}"
                            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            except Exception as e:
                print(f"[ERROR] YOLO 감지 오류: {e}")

            try:
                pose_res = pose_model(frame, verbose=False)[0]
                if pose_res.keypoints is not None and pose_res.boxes is not None:
                    kpts_xy_all = pose_res.keypoints.xy
                    boxes_all = pose_res.boxes.xyxy
                    conf_all = None
                    if hasattr(pose_res.keypoints, "conf") and pose_res.keypoints.conf is not None:
                        conf_all = pose_res.keypoints.conf
                    else:
                        if hasattr(pose_res.keypoints, "data"):
                            data = pose_res.keypoints.data
                            if data is not None and data.shape[-1] == 3:
                                conf_all = data[..., 2]

                    for i, (kpts, b) in enumerate(zip(kpts_xy_all, boxes_all)):
                        kpts_xy = kpts.detach().cpu().numpy()
                        bbox = b.detach().cpu().numpy()
                        x1, y1, x2, y2 = bbox.astype(int)
                        area = max(1, (x2-x1)*(y2-y1))
                        if area < MIN_BOX_AREA:
                            continue

                        if conf_all is not None:
                            kconf = conf_all[i].detach().cpu().numpy()
                            vis = kconf >= KP_CONF_THRES
                        else:
                            vis = np.ones(17, dtype=bool)

                        NOSE, LSH, RSH, LHIP, RHIP, LKNE, RKNE, LANK, RANK = 0, 5, 6, 11, 12, 13, 14, 15, 16
                        visible_cnt = int(vis.sum())
                        has_shoulders = bool(vis[LSH] and vis[RSH])
                        has_hips = bool(vis[LHIP] and vis[RHIP])
                        has_lower = bool(vis[LKNE] or vis[RKNE] or vis[LANK] or vis[RANK])
                        crop_y = (y1 <= EDGE_MARGIN) or (y2 >= frame.shape[0]-EDGE_MARGIN)

                        if visible_cnt < MIN_VISIBLE_KPTS or not has_shoulders:
                            continue
                        if REQUIRE_LOWER_BODY_FOR_FALL and not (has_lower or has_hips):
                            continue
                        if crop_y and not has_lower:
                            continue

                        lie_score, detail = lie_score_from_keypoints(kpts_xy, bbox, frame.shape[0])
                        if lie_score >= LIE_THRESHOLD and detail.get("horizontal", 0.0) >= MIN_HORIZONTAL_SCORE:
                            detected_classes_in_current_frame.add("PERSON_FALL")
                            color = (0, 165, 255)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame, f"PERSON_FALL {lie_score:.2f}", (x1, max(0, y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            except Exception as e:
                print(f"[ERROR] Pose 계산 오류: {e}")

            current_detected_classes = detected_classes_in_current_frame
            for cls_name in ALERT_CLASSES:
                is_detected_in_frame = cls_name in current_detected_classes
                
                if is_detected_in_frame:
                    if local_detection_states[cls_name]['start_time'] is None:
                        local_detection_states[cls_name]['start_time'] = current_time

                    duration = current_time - local_detection_states[cls_name]['start_time']
                    
                    if duration >= ALERT_DURATION_THRESHOLD and not local_detection_states[cls_name]['alert_sent']:
                        situation = SITUATION_MAPPING.get(cls_name, "UNKNOWN")
                        detail = get_situation_detail(situation)
                        image_base64 = encode_frame_to_base64(frame)

                        if send_alert_to_backend(local_client, situation, image_base64, None, None, detail):
                            local_detection_states[cls_name]['alert_sent'] = True
                            local_detection_states[cls_name]['alert_sent_time'] = current_time
                            print(f"✅ 알림 전송 완료: {situation}")
                else:
                    if local_detection_states[cls_name]['alert_sent'] == False:
                        local_detection_states[cls_name]['start_time'] = None

                if local_detection_states[cls_name]['alert_sent'] and (current_time - local_detection_states[cls_name]['alert_sent_time'] >= RESET_ALERT_AFTER):
                    local_detection_states[cls_name]['alert_sent'] = False
                    local_detection_states[cls_name]['alert_sent_time'] = None
                    local_detection_states[cls_name]['start_time'] = None
                    print(f"[RESET] {cls_name} 알림 상태 초기화")

            cv2.imshow("AI Alert System", frame)
            
            try:
                if stream_proc.stdin and stream_proc.poll() is None:
                    stream_proc.stdin.write(frame.tobytes())
            except BrokenPipeError:
                print("[ERROR] FFmpeg 파이프 연결이 끊어졌습니다. 스트리밍이 중단됩니다.")
                stream_proc.stdin.close()
                stream_proc.wait()
                stream_proc = None
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("사용자에 의해 중단됨.")
    finally:
        cv2.destroyAllWindows()
        local_client.loop_stop()
        local_client.disconnect()
        if stream_proc and stream_proc.stderr:
            stderr_output = stream_proc.stderr.read().decode('utf-8')
            if stderr_output:
                print("\n--- FFmpeg 에러 출력 시작 ---")
                print(stderr_output)
                print("--- FFmpeg 에러 출력 끝 ---")

        if stream_proc and stream_proc.stdin:
            stream_proc.stdin.close()
        if stream_proc:
            stream_proc.wait()
        print("❌ AI 및 스트리밍 프로세스 종료")


def main():
    try:
        rtsp_proc = subprocess.Popen(["/home/ssafy/mediamtx"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("✅ RTSP 서버(mediamtx) 실행 중...")
        time.sleep(2)
    except Exception as e:
        print(f"❌ RTSP 서버 실행 실패: {e}")
        return

    queue = mp.Queue(maxsize=1)
    manager = mp.Manager()
    shared_pose = manager.dict({'x': None, 'y': None})

    print("✅ ROS2 노드 스레드 시작...")
    ros_thread = threading.Thread(target=start_ros2_node, args=(shared_pose,), daemon=True)
    ros_thread.start()
    
    print("✅ API 서버 스레드 시작...")
    api_thread = threading.Thread(target=start_pose_api_server, daemon=True)
    api_thread.start()

    print("✅ 카메라 캡처 프로세스 생성 중...")
    capture_process = mp.Process(target=capture_frames, args=(queue,))
    print("✅ AI/스트리밍 프로세스 생성 중...")
    processing_process = mp.Process(target=process_and_stream_frames, args=(queue,))

    capture_process.start()
    processing_process.start()

    try:
        capture_process.join()
        processing_process.join()
    except KeyboardInterrupt:
        print("메인 프로세스에 의해 중단됨.")
    finally:
        if 'rtsp_proc' in locals():
            rtsp_proc.terminate()
        print("🎬 시스템 종료")

if __name__ == "__main__":
    main()