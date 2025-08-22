import sys
sys.path.append('/usr/lib/python3.10/dist-packages')

import cv2

capture = cv2.VideoCapture(0)  # 0 → 기본 USB 카메라

# 해상도 설정
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 카메라 열기 실패 시 메시지 출력
if not capture.isOpened():
    print("Failed to open camera.")
    exit()

while True:
    ret, frame = capture.read()
    if not ret:
        print("Failed to read frame from camera.")
        break

    cv2.imshow("VideoFrame", frame)

    # 'q' 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 처리
capture.release()
cv2.destroyAllWindows()
