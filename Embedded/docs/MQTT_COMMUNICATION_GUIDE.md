# MQTT 통신 가이드

## 개요

이 프로젝트는 임베디드 시스템과 AI 시스템 간의 MQTT 통신을 구현합니다.

## 통신 구조

### 1. 임베디드 → AI 통신 (192.168.100.141:1883)

**브로커**: `192.168.100.141:1883`  
**토픽**: `robot_data`  
**송신 주체**: 임베디드 시스템  
**수신 주체**: AI 시스템

#### 데이터 구조

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

#### 예시

```json
{
  "serial": "AMR001",
  "state": "moving",
  "x": 10.5,
  "y": 20.3,
  "speed": 1.5,
  "angle": 45.0
}
```

### 2. AI 수신 데이터 (localhost:1883)

**브로커**: `localhost:1883`  
**토픽**: `ai_data`  
**송신 주체**: AI 시스템  
**수신 주체**: AI 시스템 (자체 처리)

#### 데이터 구조

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

#### 예시

```json
{
  "serial": "AMR001",
  "x": 15.2,
  "y": 8.7,
  "img": "iVBORw0KGgoAAAANSUhEUgAA...",
  "case": "normal",
  "timeStamp": "2024-01-15 14:30:25"
}
```

## 구현 파일

### 임베디드 시스템 (C++)

1. **헤더 파일**: `include/amr/mqtt/robot_data_publisher.h`
2. **구현 파일**: `src/mqtt/robot_data_publisher.cpp`
3. **사용 예시**: `main.cpp`

```cpp
auto robot_data_publisher = std::make_shared<RobotDataPublisher>(
    "amr_robot_publisher",
    "192.168.100.141",
    "robot_data",
    1883,
    60
);

robot_data_publisher->publishRobotData(
    "AMR001",
    "moving",
    10.5,
    20.3,
    1.5,
    45.0
);
```

### AI 시스템 (Python)

1. **MQTT 클라이언트**: `mqtt/ai_mqtt_client.py`
2. **통신 테스트**: `ai/ai_communication_test.py`
3. **테스트 스크립트**: `mqtt_test.py`

```python
ai_client = AIMQTTClient("AMR001", "localhost", 1883)

def embedded_data_callback(data):
    print(f"임베디드 데이터 수신: {data}")

def ai_data_callback(data):
    print(f"AI 데이터 수신: {data}")

ai_client.set_embedded_data_callback(embedded_data_callback)
ai_client.set_ai_data_callback(ai_data_callback)

ai_client.subscribe_to_embedded_data()
ai_client.subscribe_to_ai_data()
```

## 테스트 방법

### 1. MQTT 브로커 실행

```bash
sudo apt-get install mosquitto mosquitto-clients

sudo systemctl start mosquitto
sudo systemctl enable mosquitto
```

### 2. Python 테스트

```bash
pip install paho-mqtt

python mqtt_test.py

cd ai
python run_test.py
```

### 3. C++ 빌드 및 실행

```bash
mkdir build && cd build
cmake ..
make

./amr_main
```

## 데이터 흐름

```
임베디드 시스템 (C++)
    ↓ MQTT (192.168.100.141:1883)
    robot_data 토픽
    ↓
AI 시스템 (Python)
    ↓ MQTT (localhost:1883)
    ai_data 토픽
    ↓
AI 처리 결과
```

## 주의사항

1. **네트워크 연결**: 임베디드 시스템과 AI 시스템이 같은 네트워크에 있어야 합니다.
2. **MQTT 브로커**: 두 시스템 모두 MQTT 브로커에 접근할 수 있어야 합니다.
3. **데이터 형식**: JSON 데이터는 UTF-8 인코딩으로 전송됩니다.
4. **에러 처리**: 연결 실패나 데이터 파싱 오류에 대한 적절한 처리가 필요합니다.

## 문제 해결

### 연결 실패

- MQTT 브로커가 실행 중인지 확인
- 네트워크 연결 상태 확인
- 방화벽 설정 확인

### 데이터 수신 안됨

- 토픽 구독 상태 확인
- 데이터 형식 확인
- 로그 메시지 확인

### 성능 이슈

- QoS 레벨 조정
- 메시지 크기 최적화
- 연결 풀링 고려
