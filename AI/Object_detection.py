import cv2
import time
import json
import base64
import paho.mqtt.client as mqtt
from ultralytics import YOLO

# 알림을 보낼 특정 클래스 이름 정의
ALERT_CLASSES = [
    "MATERIAL_COLLAPSE",    # 적재 물류 붕괴
    "SMOKING_VIOLATION",    # 작업장 내 흡연
    "NOT_WEAR_WORKER"       # 안전장비 미착용
]

detection_states = {
    cls_name: {'start_time': None, 'alert_sent': False, 'alert_sent_time': None}
    for cls_name in ALERT_CLASSES
}

ALERT_DURATION_THRESHOLD = 0.5  
RESET_ALERT_AFTER = 60  

# MQTT 설정
BROKER = "192.168.100.141"
PORT = 1883
AMR_SERIAL = "AMR001"  
TOPIC = f"alert/{AMR_SERIAL}"

client = mqtt.Client()
client.connect(BROKER, PORT, 60)

SITUATION_MAPPING = {
    "MATERIAL_COLLAPSE": "COLLAPSE",
    "SMOKING_VIOLATION": "SMOKE", 
    "NOT_WEAR_WORKER": "EQUIPMENT"
}

def send_alert_to_backend(situation, image_base64="", x="", y="", detail=""):
    """
    AI에서 Backend로 알림 전송
    
    Args:
        situation (str): 상황 ("COLLAPSE", "SMOKE", "EQUIPMENT")
        image_base64 (str): Base64로 인코딩된 이미지
        x (str): X 좌표 또는 구역명
        y (str): Y 좌표 또는 구역명
        detail (str): 상세 정보
    """
    message = {
        "situation": situation,
        "image": image_base64,
        "x": x,
        "y": y,
        "detail": detail
    }

    try:
        client.publish(TOPIC, json.dumps(message))
        print(f"[AI -> Backend] 상황: {situation}, 상세: {detail}")
        return True
    except Exception as e:
        print(f"[ERROR] 알림 전송 실패: {e}")
        return False

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
        "EQUIPMENT": "안전장비를 착용하지 않은 작업자가 감지되었습니다."
    }
    return detail_mapping.get(situation, "알 수 없는 상황이 발생했습니다.")

def main():
    # 웹캠 초기화
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return

    # YOLOv8 모델 로드
    try:
        model = YOLO("best.pt").to("cuda")
        print("YOLO 모델 로드 완료")
    except Exception as e:
        print(f"YOLO 모델 로드 실패: {e}")
        return

    print(f"AI 알림 시스템 시작 - AMR: {AMR_SERIAL}")
    print(f"MQTT 토픽: {TOPIC}")
    print("감지 대상:", ALERT_CLASSES)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
        
        current_time = time.time()
        detected_classes_in_current_frame = set()

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
                    if class_name in ALERT_CLASSES:
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
                    if send_alert_to_backend(situation, image_base64, "", "", detail):
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

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()
    client.disconnect()
    print("AI 알림 시스템 종료")

if __name__ == "__main__":
    main() 