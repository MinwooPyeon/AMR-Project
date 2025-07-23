import cv2
from ultralytics import YOLO

def main():
    # 0은 기본 웹캠을 의미합니다. 다른 카메라를 사용하려면 숫자를 변경하세요.
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return

    # YOLOv8 모델 로드 (다운로드되어 있지 않다면 자동으로 다운로드됩니다)
    model = YOLO("yolov8n.pt").to("cuda") # 'yolov8n.pt' 대신 원하는 모델 파일 사용

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # YOLO 모델로 객체 감지 수행
        # stream=True는 비디오 스트림 처리에 최적화된 모드입니다.
        results = model(frame, stream=True)

        # 감지 결과 처리 및 화면에 그리기
        for r in results:
            boxes = r.boxes  # 감지된 객체의 바운딩 박스 정보
            for box in boxes:
                # 바운딩 박스 좌표 얻기 (xyxy 형식: x1, y1, x2, y2)
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # 신뢰도 (confidence) 얻기
                confidence = float(box.conf[0])

                # 클래스 ID와 이름 얻기
                class_id = int(box.cls[0])
                class_name = model.names[class_id] # YOLO 모델의 클래스 이름

                # 바운딩 박스 그리기
                color = (0, 255, 0) # 초록색 (BGR)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # 레이블 (클래스 이름 및 신뢰도) 표시
                label = f"{class_name} {confidence:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 결과 프레임 출력
        cv2.imshow("Webcam with YOLO Object Detection", frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()