# MQTT 통신 모듈

AMR과 백엔드 간의 MQTT 통신을 담당하는 모듈들입니다.

## 파일 구조

- `backend_mqtt_subscriber.py`: 백엔드 MQTT 구독자
- `sensor_data_transmitter.py`: 센서 데이터 전송기

## 주요 기능

- MQTT 브로커 연결 및 관리
- 센서 데이터 전송
- 백엔드 명령 수신
- 연결 상태 모니터링

## 사용법

```python
from mqtt import BackendMQTTSubscriber, SensorDataTransmitter

# 백엔드 구독자 생성
subscriber = BackendMQTTSubscriber("192.168.100.141", 1883)

# 센서 데이터 전송기 생성
transmitter = SensorDataTransmitter("AMR001", "192.168.100.141", 1883)
``` 