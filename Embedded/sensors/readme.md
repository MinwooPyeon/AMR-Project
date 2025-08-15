# 센서 동기화 모듈

AMR의 다양한 센서 데이터 수집 및 동기화를 담당하는 모듈들입니다.

## 파일 구조

- `sensor_data_sync.py`: 센서 데이터 동기화

## 주요 기능

- 다중 센서 데이터 수집 (LIDAR, CAMERA, IMU, MOTOR_STATUS)
- 실시간 데이터 동기화
- 백엔드 전송용 데이터 패킷 생성
- 센서 데이터 이력 관리

## 사용법

```python
from sensor_sync import SensorDataSync, SensorType

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