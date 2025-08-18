# ROS2 통신 모듈

AMR과 AI 간의 ROS2 통신을 담당하는 모듈

## 파일 구조

- `ai_position_subscriber.py`: AI 위치 데이터 구독자
- `requirements.txt`: Python 의존성
- `__init__.py`: 패키지 초기화

## 주요 기능

- AI 위치 데이터 수신
- AI 명령 데이터 처리
- ROS2 토픽 구독
- 실시간 데이터 동기화

## 사용법

```python
from ros2_module import AIPositionSubscriber

# AI 위치 구독자 생성
subscriber = AIPositionSubscriber()

# 위치 콜백 설정
def position_callback(x, y):
    print(f"AI 위치: ({x}, {y})")

subscriber.set_position_callback(position_callback)

# AI 데이터 콜백 설정
def ai_data_callback(data):
    print(f"AI 데이터: {data}")

subscriber.set_ai_data_callback(ai_data_callback)
```

## 구독 토픽

- `/ai/position` (Pose2D): AI 위치 데이터
- `/ai/position_point` (Point): AI 위치 포인트
- `/ai/position_array` (Float64MultiArray): AI 위치 배열
- `/ai/position_json` (String): AI 위치 JSON
- `/position` (String): AI 명령 데이터

## 의존성

- rclpy>=0.1.0
- geometry_msgs>=0.1.0
- std_msgs>=0.1.0
- numpy>=1.21.0
