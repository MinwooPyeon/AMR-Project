# MQTT 통신 모듈

AMR 시스템의 MQTT 통신을 관리하는 모듈입니다.

## 📡 데이터 흐름

```
AI 시스템 (localhost:1883) → Embedded 시스템 → Backend (192.168.100.141:1883)
```

## 📁 파일 구조

### 핵심 파일들
- `ai_mqtt_client.py`: AI MQTT 클라이언트 (localhost:1883 수신)
- `sensor_data_transmitter.py`: 센서 데이터 전송기 (192.168.100.141:1883 송신)
- `mqtt_manager.py`: 통합 MQTT 매니저

## 🔗 연결 구조

### 1. AI → Embedded (localhost:1883)
- **역할**: AI에서 보내는 명령 및 데이터 수신
- **토픽**: `ai_data`, `position`, `alert/*`
- **JSON 형태**:
```json
{
    "serial": "string",
    "x": "float",
    "y": "float",
    "img": "Base64",
    "case": "string",
    "timeStamp": "string"
}
```

### 2. Embedded → Backend (192.168.100.141:1883)
- **역할**: Embedded에서 Backend로 데이터 전송
- **토픽**: `robot_data`
- **JSON 형태**:
```json
{
    "serial": "string",
    "state": "string",
    "x": "float",
    "y": "float",
    "speed": "float",
    "angle": "float"
}
```

## 🚀 사용 방법

### 기본 사용법
```python
from mqtt.mqtt_manager import MQTTManager

# MQTT 매니저 초기화
mqtt_manager = MQTTManager("AMR001")

# 모든 연결 설정
if mqtt_manager.connect_all():
    print("MQTT 연결 성공")
    
    # Backend로 데이터 전송
    data = {
        "serial": "AMR001",
        "state": "RUNNING",
        "x": "10.5",
        "y": "20.3",
        "speed": "5.0",
        "angle": "45.0"
    }
    mqtt_manager.send_to_backend(data)
```

### AI 데이터 수신
```python
def ai_command_callback(command):
    print(f"AI 명령 수신: {command}")

mqtt_manager.set_ai_command_callback(ai_command_callback)
```

## 📊 모니터링

### 연결 상태 확인
```python
status = mqtt_manager.get_connection_status()
print(f"Backend 연결: {status['backend']}")
print(f"AI 연결: {status['ai']}")
```

### 통계 조회
```python
stats = mqtt_manager.get_stats()
print(f"Backend 전송: {stats['backend_sent_count']}")
print(f"AI 수신: {stats['ai_received_count']}")
```

## ⚠️ 주의사항

1. **단방향 통신**: AI → Embedded → Backend 순서로만 데이터 흐름
2. **연결 관리**: 자동 재연결 기능 포함
3. **에러 처리**: 연결 실패 시 적절한 에러 처리
4. **데이터 형식**: 모든 숫자 값은 문자열로 전송

## 🧪 테스트

```bash
# 단방향 통신 테스트
python utils/test_bidirectional_communication.py
``` 