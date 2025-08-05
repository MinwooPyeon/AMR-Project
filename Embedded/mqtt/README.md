# MQTT 통신 모듈

AMR과 백엔드, AI 간의 MQTT 통신을 담당하는 모듈들입니다.

## 파일 구조

- `backend_mqtt_subscriber.py`: 백엔드 MQTT 구독자 (192.168.100.141:1883)
- `sensor_data_transmitter.py`: 센서 데이터 전송기 (192.168.100.141:1883)
- `ai_mqtt_client.py`: AI MQTT 클라이언트 (localhost:1883)
- `mqtt_manager.py`: 통합 MQTT 매니저

## 통신 구조

### 백엔드 통신 (192.168.100.141:1883) - 송신만
**토픽**: `status`

**Embedded → Backend**:
```json
{
  "serial": "AMR001",
  "state": "RUNNING",
  "x": "AI에서 받은 x 좌표",
  "y": "AI에서 받은 y 좌표",
  "speed": "25"
}
```

### AI 통신 (localhost:1883) - 수신만
**토픽**: `position`

**AI → Embedded**:
```json
{
  "MOVING_FORWARD": "",
  "ROTATE_LEFT": "",
  "ROTATE_RIGHT": "",
  "MOVING_BACKWARD": "",
  "STOP": "",
  "img": "",
  "situation": "",
  "x": "",
  "y": ""
}
```

## 주요 기능

- MQTT 브로커 연결 및 관리
- 센서 데이터 전송 (백엔드) - 송신만
- AI 명령 수신 (localhost) - 수신만
- 연결 상태 모니터링
- 통합 통신 관리

## 사용법

### 개별 모듈 사용
```python
from mqtt import SensorDataTransmitter, AIMQTTClient

# 백엔드 통신 (송신만)
transmitter = SensorDataTransmitter("AMR001", "192.168.100.141", 1883)

# AI 통신 (수신만)
ai_client = AIMQTTClient("AMR001", "localhost", 1883)
```

### 통합 매니저 사용
```python
from mqtt.mqtt_manager import MQTTManager

# 통합 MQTT 매니저
manager = MQTTManager("AMR001")
manager.connect_all()

# 백엔드로 데이터 전송 (AI 좌표 사용, speed는 25로 고정)
manager.send_to_backend({"state": "RUNNING"})

# AI 명령 수신 (수신만)
active_command = manager.get_active_ai_command()
situation = manager.get_ai_situation()
``` 