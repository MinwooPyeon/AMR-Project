# 센서 동기화 모듈

AMR의 다양한 센서 데이터 수집 및 동기화를 담당하는 모듈들입니다.

## 파일 구조

- `sensor_data_sync.py`: 센서 데이터 동기화
- `requirements.txt`: Python 의존성
- `__init__.py`: 패키지 초기화

## 주요 기능

- 다중 센서 데이터 수집 (LIDAR, CAMERA, IMU, MOTOR_STATUS)
- 실시간 데이터 동기화
- 백엔드 전송용 데이터 패킷 생성
- 센서 데이터 이력 관리

## 사용법

```python
from sensors import SensorDataSync, SensorType

# 센서 동기화 생성
sensor_sync = SensorDataSync("AMR001")

# 센서 데이터 업데이트
sensor_sync.update_sensor_data(SensorType.MOTOR_STATUS, {
    "left_speed": 25.0,
    "right_speed": 25.0
})

# 동기화 시작
sensor_sync.start_sync()

# 데이터 패킷 생성
data_packet = sensor_sync.create_data_packet()
```

## 센서 타입

- `SensorType.LIDAR`: 라이다 센서 데이터
- `SensorType.CAMERA`: 카메라 이미지 데이터
- `SensorType.IMU`: 관성 측정 장치 데이터
- `SensorType.MOTOR_STATUS`: 모터 상태 데이터

## 의존성

- numpy>=1.21.0
- opencv-python>=4.5.0
- Pillow>=8.3.0
