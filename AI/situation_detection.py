import cv2
import time
import json
import base64
import numpy as np
import paho.mqtt.client as mqtt
from ultralytics import YOLO
import threading
from pose_api import start_pose_api_server, update_pose
from datetime import datetime
import subprocess
import os

def start_rtsp_server():
    if not os.path.exists("/home/ssafy/mediamtx"):
        raise FileNotFoundError("❌ mediamtx 실행 파일이 없습니다.")
    print("✅ RTSP 서버(mediamtx) 실행 중...")
    return subprocess.Popen(["/home/ssafy/mediamtx"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def start_ffmpeg_stream():
    ffmpeg_cmd = [
        "ffmpeg",
        "-f", "v4l2",
        "-input_format", "yuyv422",
        "-framerate", "30",
        "-video_size", "1280x720",
        "-i", "/dev/video0",
        "-pix_fmt", "yuv420p",
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-profile:v", "baseline",
        "-level:v", "3.1",
        "-f", "rtsp",
        "rtsp://localhost:8554/mystream"
    ]
    print("🎥 FFmpeg 스트리밍 시작 중...")
    return subprocess.Popen(ffmpeg_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

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
    # 1) 상체 방향 수평성 (어깨↔엉덩이 중심 벡터)
    sh_c = _mid(kpts_xy[LSH], kpts_xy[RSH])
    hip_c = _mid(kpts_xy[LHIP], kpts_xy[RHIP])
    v = hip_c - sh_c
    v_norm = np.linalg.norm(v) + 1e-6
    v_unit = v / v_norm
    vertical = np.array([0.0, 1.0])
    verticality = abs(float(np.dot(v_unit, vertical)))  # 1=수직, 0=수평
    horizontal_score = 1.0 - verticality

    # 2) 박스 가로/세로 비
    x1, y1, x2, y2 = bbox
    w, h = max(1.0, x2 - x1), max(1.0, y2 - y1)
    aspect = w / h
    aspect_score = float(np.tanh(aspect - 1.2))  # 1.2부터 가점

    # 3) 관절의 y-순서가 깨졌는지
    ys = {
        "ankle": np.mean([kpts_xy[LANK,1], kpts_xy[RANK,1]]),
        "knee":  np.mean([kpts_xy[LKNE,1], kpts_xy[RKNE,1]]),
        "hip":   hip_c[1],
        "shoulder": sh_c[1],
        "nose": kpts_xy[NOSE,1],
    }
    order = ["ankle","knee","hip","shoulder","nose"]
    inversions = 0
    for i in range(len(order)-1):
        if not (ys[order[i]] > ys[order[i+1]]):  # 서 있으면 ankle>knee>... 성립
            inversions += 1
    order_score = inversions / 4.0

    # 4) y-분산 (눕기면 여러 관절 y가 비슷 → 분산↓)
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
    "MATERIAL_COLLAPSE",     # 적재 물류 붕괴
    "SMOKING_VIOLATION",     # 작업장 내 흡연
    "NOT_WEAR_WORKER",       # 안전장비 미착용
    "PERSON_FALL"
]

detection_states = {
    cls_name: {'start_time': None, 'alert_sent': False, 'alert_sent_time': None}
    for cls_name in ALERT_CLASSES
}

ALERT_DURATION_THRESHOLD = 0.5  
RESET_ALERT_AFTER = 60  
# --- lie/fall 판정 보강용 상수 ---
KP_CONF_THRES = 0.35       # 키포인트 최소 신뢰도
MIN_VISIBLE_KPTS = 8        # 전체 최소 가시 키포인트 수
EDGE_MARGIN = 8             # 프레임 경계 여유(px)
REQUIRE_LOWER_BODY_FOR_FALL = True  # 하체(무릎/발목) 일부라도 보여야 'FALL' 허용
LIE_THRESHOLD = 0.55     # 이 이상이면 '눕기'로 간주 (환경별로 0.5~0.65 사이 튜닝 권장)
MIN_HORIZONTAL_SCORE = 0.35 # 수평성 최소 요건
MIN_BOX_AREA = 80*80     # 너무 작은 인물 박스는 무시(원거리 노이즈 방지)

# MQTT 설정
BROKER = "192.168.100.141"
PORT = 1883
AMR_SERIAL = "AMR001"   
TOPIC = f"alert"

# client = mqtt.Client()
# client.connect(BROKER, PORT, 60)

SITUATION_MAPPING = {
    "MATERIAL_COLLAPSE": "COLLAPSE",
    "SMOKING_VIOLATION": "SMOKE", 
    "NOT_WEAR_WORKER": "EQUIPMENT",
    "PERSON_FALL": "FALL"    # ⬅️ 추가
}

# robot_pose를 전역 변수로 선언하고 초기화
robot_pose = {'x': None, 'y': None}

# class PoseSubscriber(Node):
#     def __init__(self):
#         super().__init__('pose_subscriber')
        
        # /amcl_pose 토픽 발행자의 QoS 프로필과 일치하도록 설정
        # qos_profile = QoSProfile(depth=10)
        # qos_profile.reliability = ReliabilityPolicy.RELIABLE
        # qos_profile.history = HistoryPolicy.KEEP_LAST
        # qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # self.subscription = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/amcl_pose',  # Localization에서 퍼블리시하는 토픽
        #     self.listener_callback,
        #     qos_profile) # QoS 프로필 적용

#     def listener_callback(self, msg):
#         global robot_pose
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         robot_pose['x'] = x
#         robot_pose['y'] = y
#         update_pose(x, y)  # <--- API 서버에 현재 좌표 업데이트
#         self.get_logger().info(f"Received pose: x={x}, y={y}")


# def start_ros2_node():
#     rclpy.init()
#     node = PoseSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# 스레드로 실행
# ros_thread = threading.Thread(target=start_ros2_node, daemon=True)
# ros_thread.start()

# API 서버 실행 (로컬 + 외부에서 좌표 접근 가능)
# start_pose_api_server()

def send_alert_to_backend(situation, image_base64="", x=None, y=None, detail=""):
    """
    AI에서 Backend로 알림 전송
    
    Args:
        situation (str): 상황 ("COLLAPSE", "SMOKE", "EQUIPMENT")
        image_base64 (str): Base64로 인코딩된 이미지
        x (str): X 좌표 또는 구역명
        y (str): Y 좌표 또는 구역명
        detail (str): 상세 정보
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

    # try:
    #     client.publish(TOPIC, json.dumps(message))
    #     print(f"[AI -> Backend] 상황: {situation}, 상세: {detail}, 로봇 위치: x={x_val}, y={y_val}")
    #     return True
    # except Exception as e:
    #     print(f"[ERROR] 알림 전송 실패: {e}")
    #     return False

# 이미지 프레임을 Base64로 변환하는 함수
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

def main():
    # ▶️ RTSP 서버 및 스트리밍 시작
    # try:
    #     rtsp_proc = start_rtsp_server()
    #     time.sleep(2)  # 서버 시작 대기

    #     ffmpeg_proc = start_ffmpeg_stream()
    #     time.sleep(2)  # 스트리밍 시작 대기
    # except Exception as e:
    #     print(f"RTSP 초기화 실패: {e}")
    #     return
    # 웹캠 초기화
    cap = cv2.VideoCapture(0)
    # cap = cv2.VideoCapture("/dev/video0")

    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return

    # YOLOv8 모델 로드
    try:
        model = YOLO("epoch40.pt").to("cuda")
        pose_model = YOLO("yolo11n-pose.pt").to("cuda")
        print("YOLO 모델 로드 완료")
    except Exception as e:
        print(f"YOLO 모델 로드 실패: {e}")
        return

    print(f"AI 알림 시스템 시작 - AMR: {AMR_SERIAL}")
    print(f"MQTT 토픽: {TOPIC}")
    print("감지 대상:", ALERT_CLASSES)

    try:
        while True:
            # ros_thread가 백그라운드에서 실행되므로, main 루프는 이미지 처리에 집중
            ret, frame = cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다.")
                break
            
            current_time = time.time()
            detected_classes_in_current_frame = set()
            img_h, img_w = frame.shape[:2]
            
            # YOLO 모델로 객체 감지 수행
            try:
                results = model(frame, stream=True)

                # 감지 결과 처리 및 화면에 그리기
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        # 바운딩 박스 좌표
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        
                        # 신뢰도
                        confidence = float(box.conf[0])
                        
                        # 클래스 ID와 이름
                        class_id = int(box.cls[0])
                        class_name = model.names[class_id]

                        # 알림 대상 클래스인 경우만 처리
                        if class_name in ALERT_CLASSES and confidence > 0.7:
                            detected_classes_in_current_frame.add(class_name)
                            
                            # 바운딩 박스 그리기 (빨간색으로 표시)
                            color = (0, 0, 255)  # 빨간색 (BGR)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            
                            # 레이블 표시
                            label = f"{class_name} {confidence:.2f}"
                            cv2.putText(frame, label, (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            except Exception as e:
                print(f"YOLO 감지 오류: {e}")

            try:
                pose_res = pose_model(frame, verbose=False)[0]
                if pose_res.keypoints is not None and pose_res.boxes is not None:
                    kpts_xy_all = pose_res.keypoints.xy
                    boxes_all = pose_res.boxes.xyxy

                    # conf(가시성) 안전 추출
                    conf_all = None
                    if hasattr(pose_res.keypoints, "conf") and pose_res.keypoints.conf is not None:
                        conf_all = pose_res.keypoints.conf  # (n,17)
                    else:
                        # fallback: data가 (n,17,3) 형태면 마지막 채널이 conf
                        if hasattr(pose_res.keypoints, "data"):
                            data = pose_res.keypoints.data  # (n,17,2 or 3)
                            if data is not None and data.shape[-1] == 3:
                                conf_all = data[..., 2]

                    for i, (kpts, b) in enumerate(zip(kpts_xy_all, boxes_all)):
                        kpts_xy = kpts.detach().cpu().numpy()        # (17,2)
                        bbox = b.detach().cpu().numpy()              # [x1,y1,x2,y2]
                        x1, y1, x2, y2 = bbox.astype(int)
                        area = max(1, (x2-x1)*(y2-y1))
                        if area < MIN_BOX_AREA:
                            continue

                        # 가시성 마스크 계산
                        if conf_all is not None:
                            kconf = conf_all[i].detach().cpu().numpy()  # (17,)
                            vis = kconf >= KP_CONF_THRES
                        else:
                            # conf 정보가 없으면 느슨하게 모두 보인 것으로 간주
                            vis = np.ones(17, dtype=bool)

                        # 관절 인덱스
                        NOSE, LSH, RSH, LHIP, RHIP, LKNE, RKNE, LANK, RANK = 0, 5, 6, 11, 12, 13, 14, 15, 16

                        visible_cnt = int(vis.sum())
                        has_shoulders = bool(vis[LSH] and vis[RSH])
                        has_hips = bool(vis[LHIP] and vis[RHIP])
                        has_lower = bool(vis[LKNE] or vis[RKNE] or vis[LANK] or vis[RANK])

                        # 프레임 경계 크롭 여부
                        crop_y = (y1 <= EDGE_MARGIN) or (y2 >= frame.shape[0]-EDGE_MARGIN)

                        # ===== 가드레일: 부분 크롭/가시성 체크 =====
                        # 1) 최소 가시 키포인트 수
                        if visible_cnt < MIN_VISIBLE_KPTS:
                            continue
                        # 2) 어깨는 반드시 둘 다 필요 (상체 방향 추정의 신뢰성)
                        if not has_shoulders:
                            continue
                        # 3) 하체 일부라도 보이거나, 최소한 엉덩이+어깨 조합은 있어야 함
                        if REQUIRE_LOWER_BODY_FOR_FALL:
                            if not (has_lower or has_hips):
                                continue
                        else:
                            if not (has_hips or has_lower):
                                continue
                        # 4) 프레임 상/하단에 걸리면서 하체가 안 보이면 보류(오탐 방지)
                        if crop_y and not has_lower:
                            continue

                        # 거르기 통과했으면 점수 계산
                        lie_score, detail = lie_score_from_keypoints(kpts_xy, bbox, frame.shape[0])

                        # 수평성 최소 요건 추가(부분크롭 시 aspect만으로 눕기로 가는 것 방지)
                        if lie_score >= LIE_THRESHOLD and detail.get("horizontal", 0.0) >= MIN_HORIZONTAL_SCORE:
                            detected_classes_in_current_frame.add("PERSON_FALL")

                            # 시각화
                            color = (0, 165, 255)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame, f"PERSON_FALL {lie_score:.2f}",
                                        (x1, max(0, y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            except Exception as e:
                print(f"Pose 계산 오류: {e}")
            # 각 알림 대상 클래스에 대한 탐지 상태 업데이트 및 알림 전송
            for cls_name in ALERT_CLASSES:
                if cls_name in detected_classes_in_current_frame:
                    # 클래스가 현재 탐지됨
                    if detection_states[cls_name]['start_time'] is None:
                        # 새로 탐지된 경우, 시작 시간 기록
                        detection_states[cls_name]['start_time'] = current_time
                        detection_states[cls_name]['alert_sent'] = False
                    
                    # 탐지 지속 시간 계산
                    duration = current_time - detection_states[cls_name]['start_time']

                    # 0.5초 이상 지속되고 아직 알림을 보내지 않았다면
                    if duration >= ALERT_DURATION_THRESHOLD and not detection_states[cls_name]['alert_sent']:
                        # 상황 매핑
                        situation = SITUATION_MAPPING.get(cls_name, "UNKNOWN")
                        
                        # 상세 정보 생성
                        detail = get_situation_detail(situation)
                        
                        # 이미지 인코딩
                        image_base64 = encode_frame_to_base64(frame)
                        
                        # 알림 전송
                        if send_alert_to_backend(situation, image_base64, None, None, detail):
                            detection_states[cls_name]['alert_sent'] = True
                            detection_states[cls_name]['alert_sent_time'] = current_time
                            print(f"✅ 알림 전송 완료: {situation}")
                else:
                    # 클래스가 현재 탐지되지 않음 (상태 초기화)
                    detection_states[cls_name]['start_time'] = None
                
                # 1분 후 초기화
                if detection_states[cls_name]['alert_sent']:
                    if current_time - detection_states[cls_name]['alert_sent_time'] >= RESET_ALERT_AFTER:
                        detection_states[cls_name]['alert_sent'] = False
                        detection_states[cls_name]['alert_sent_time'] = None
                        print(f"[RESET] {cls_name} 알림 상태 초기화")
            
            # 결과 프레임 출력
            cv2.imshow("AI Alert System", frame)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("사용자에 의해 중단됨.")
    
    finally:
        # 자원 해제
        cap.release()
        cv2.destroyAllWindows()
        # client.disconnect()
        # if 'rtsp_proc' in locals():
        #     rtsp_proc.terminate()
        # if 'ffmpeg_proc' in locals():
        #     ffmpeg_proc.terminate()
        print("🎬 시스템 종료")

if __name__ == "__main__":
    main()
