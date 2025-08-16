# 통신 모듈

AMR(Autonomous Mobile Robot) 프로젝트 통신 기능

## 개요

- **CommunicationManager**: MQTT, WebSocket, HTTP 통신을 통합 관리
- **ProtocolHandler**: 메시지 인코딩/디코딩 및 프로토콜 처리
- **CommunicationConfig**: 통신 설정 및 토픽 관리

## 설치

### 의존성

```bash
pip install -r requirements.txt
```

## 사용법

### 통신 매니저

```python
from communication import CommunicationManager

manager = CommunicationManager()

# MQTT 설정
manager.setup_mqtt("localhost", 1883)
manager.subscribe_mqtt("amr/sensor/data", callback_function)
manager.publish_mqtt("amr/command", "move_forward")

# WebSocket 서버 설정
await manager.setup_websocket_server("localhost", 8765)
manager.register_websocket_callback("/sensor", sensor_callback)

# HTTP 클라이언트 설정
manager.setup_http_client("http://localhost:8000")
response = manager.http_post("/api/status", {"status": "running"})
```

### 프로토콜 핸들러

```python
from communication import ProtocolHandler

handler = ProtocolHandler()

# JSON 메시지 생성
sensor_data = handler.create_sensor_data("temp_001", 25.5, "celsius")
command_msg = handler.create_command("move", {"direction": "forward"})

# 메시지 인코딩/디코딩
encoded = handler.encode_json(sensor_data)
decoded = handler.decode_json(encoded)

# 메시지 검증
is_valid = handler.validate_message(decoded, ["type", "payload"])
```

### 설정 관리

```python
from communication import CommunicationConfig

# MQTT 설정 가져오기
mqtt_config = CommunicationConfig.get_mqtt_config()

# 토픽 가져오기
topic = CommunicationConfig.get_topic("sensor_data")

# 메시지 타입 가져오기
msg_type = CommunicationConfig.get_message_type("SENSOR_DATA")
```

## 기능

### 지원 프로토콜

- **MQTT**: 메시지 큐 기반 통신
- **WebSocket**: 실시간 양방향 통신
- **HTTP**: RESTful API 통신

### 메시지 타입

- **sensor_data**: 센서 데이터 전송
- **motor_control**: 모터 제어 명령
- **status**: 상태 정보
- **command**: 일반 명령
- **alert**: 알림 메시지
- **config**: 설정 정보
- **heartbeat**: 연결 상태 확인
- **ack**: 확인 응답
- **error**: 오류 메시지

### 통신 기능

- **메시지 발행/구독**: MQTT 토픽 기반 통신
- **실시간 통신**: WebSocket을 통한 실시간 데이터 전송
- **REST API**: HTTP를 통한 RESTful 통신
- **메시지 브로드캐스트**: 여러 프로토콜로 동시 전송
- **연결 상태 모니터링**: 각 프로토콜의 연결 상태 확인

## 설정

### 환경 변수

```env
# MQTT 설정
MQTT_BROKER=localhost
MQTT_PORT=1883
MQTT_KEEPALIVE=60
MQTT_USERNAME=user
MQTT_PASSWORD=pass
MQTT_CLIENT_ID=amr_client

# WebSocket 설정
WEBSOCKET_HOST=localhost
WEBSOCKET_PORT=8765
WEBSOCKET_MAX_CONNECTIONS=100

# HTTP 설정
HTTP_BASE_URL=http://localhost:8000
HTTP_TIMEOUT=30
HTTP_RETRIES=3
```

### 기본 토픽

- `amr/sensor/data` - 센서 데이터
- `amr/motor/control` - 모터 제어
- `amr/status` - 상태 정보
- `amr/command` - 명령
- `amr/alert` - 알림
- `amr/config` - 설정
- `amr/heartbeat` - 연결 상태

## 테스트

```bash
# 통신 매니저 테스트
python -m communication.communication_manager

# 프로토콜 핸들러 테스트
python -m communication.protocol_handler
```
