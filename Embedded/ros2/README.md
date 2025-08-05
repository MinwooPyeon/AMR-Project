# ROS2 통신 모듈

AMR과 AI 간의 ROS2 통신을 담당하는 모듈들입니다.

## 파일 구조

- `ai_position_subscriber.py`: AI 위치 데이터 구독자

## 주요 기능

- AI 위치 데이터 수신
- AI 명령 데이터 처리
- ROS2 토픽 구독
- 실시간 데이터 동기화

## 사용법

```python
from ros2 import AIPositionSubscriber

# AI 위치 구독자 생성
subscriber = AIPositionSubscriber()

# 위치 콜백 설정
def position_callback(x, y):
    print(f"AI 위치: ({x}, {y})")

subscriber.set_position_callback(position_callback)
``` 