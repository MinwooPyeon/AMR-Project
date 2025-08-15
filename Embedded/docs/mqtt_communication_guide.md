# MQTT 통신 가이드

## 개요

이 문서는 AMR 프로젝트에서 MQTT 통신을 설정하고 사용하는 방법을 설명합니다.

## MQTT 기본 개념

### MQTT란?

MQTT(Message Queuing Telemetry Transport)는 IoT 장치 간의 경량 메시지 전송을 위한 프로토콜입니다.

### 주요 특징

- **경량**: 최소한의 대역폭 사용
- **실시간**: 낮은 지연 시간
- **신뢰성**: QoS 레벨을 통한 메시지 전송 보장
- **확장성**: 많은 장치 동시 연결 지원

## MQTT 브로커 설정

### 1. Mosquitto 설치

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install mosquitto mosquitto-clients

# 서비스 시작
sudo systemctl start mosquitto
sudo systemctl enable mosquitto

# 상태 확인
sudo systemctl status mosquitto
```

### 2. 설정 파일 편집

```bash
# 설정 파일 편집
sudo nano /etc/mosquitto/mosquitto.conf
```

```conf
# 기본 설정
port 1883
allow_anonymous true

# 로그 설정
log_type all
log_dest file /var/log/mosquitto/mosquitto.log

# 지속성 설정
persistence true
persistence_location /var/lib/mosquitto/

# 보안 설정 (선택사항)
# password_file /etc/mosquitto/passwd
```

### 3. 서비스 재시작

```bash
sudo systemctl restart mosquitto
```

## MQTT 클라이언트 구현

### Python MQTT 클라이언트

```python
import paho.mqtt.client as mqtt
import json
import time

class MQTTClient:
    def __init__(self, broker="localhost", port=1883, client_id=None):
        self.broker = broker
        self.port = port
        self.client_id = client_id or f"amr_client_{int(time.time())}"

        # MQTT 클라이언트 생성
        self.client = mqtt.Client(client_id=self.client_id)

        # 콜백 함수 설정
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        # 연결 상태
        self.connected = False

    def on_connect(self, client, userdata, flags, rc):
        """연결 콜백"""
        if rc == 0:
            print(f"MQTT 브로커에 연결됨: {self.broker}")
            self.connected = True
        else:
            print(f"연결 실패, 코드: {rc}")

    def on_message(self, client, userdata, msg):
        """메시지 수신 콜백"""
        try:
            payload = json.loads(msg.payload.decode())
            print(f"메시지 수신: {msg.topic} - {payload}")
        except json.JSONDecodeError:
            print(f"메시지 수신: {msg.topic} - {msg.payload.decode()}")

    def on_disconnect(self, client, userdata, rc):
        """연결 해제 콜백"""
        print("MQTT 브로커와 연결 해제됨")
        self.connected = False

    def connect(self):
        """브로커에 연결"""
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            return True
        except Exception as e:
            print(f"연결 오류: {e}")
            return False

    def disconnect(self):
        """연결 해제"""
        self.client.loop_stop()
        self.client.disconnect()

    def publish(self, topic, message, qos=0):
        """메시지 발행"""
        if not self.connected:
            print("브로커에 연결되지 않음")
            return False

        try:
            if isinstance(message, dict):
                message = json.dumps(message)
            result = self.client.publish(topic, message, qos=qos)
            return result.rc == mqtt.MQTT_ERR_SUCCESS
        except Exception as e:
            print(f"발행 오류: {e}")
            return False

    def subscribe(self, topic, qos=0):
        """토픽 구독"""
        if not self.connected:
            print("브로커에 연결되지 않음")
            return False

        try:
            result = self.client.subscribe(topic, qos=qos)
            return result[0] == mqtt.MQTT_ERR_SUCCESS
        except Exception as e:
            print(f"구독 오류: {e}")
            return False
```

## 토픽 구조

### 토픽 네이밍 규칙

```
amr/{robot_id}/{category}/{action}
```

### 예시 토픽

```
# 로봇 상태
amr/AMR001/status/position
amr/AMR001/status/battery
amr/AMR001/status/motor

# 명령
amr/AMR001/command/move
amr/AMR001/command/stop
amr/AMR001/command/emergency

# 센서 데이터
amr/AMR001/sensor/imu
amr/AMR001/sensor/camera
amr/AMR001/sensor/lidar

# 알림
amr/AMR001/alert/warning
amr/AMR001/alert/error
amr/AMR001/alert/info
```

## 메시지 형식

### JSON 메시지 구조

```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "robot_id": "AMR001",
  "type": "status",
  "data": {
    "position": {
      "x": 10.5,
      "y": 20.3,
      "angle": 45.0
    },
    "battery": 85.5,
    "status": "running"
  }
}
```

### 명령 메시지

```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "command": "move_forward",
  "parameters": {
    "speed": 50,
    "distance": 100
  }
}
```

### 알림 메시지

```json
{
  "timestamp": "2024-01-01T12:00:00Z",
  "level": "warning",
  "message": "배터리 레벨이 낮습니다",
  "data": {
    "battery_level": 15.0
  }
}
```

## QoS 레벨

### QoS 0: 최대 한 번 전송

- 메시지 손실 가능
- 가장 빠른 전송
- 네트워크 상태가 좋을 때 사용

### QoS 1: 최소 한 번 전송

- 메시지 중복 가능
- 적당한 신뢰성
- 일반적인 용도에 적합

### QoS 2: 정확히 한 번 전송

- 메시지 중복 없음
- 가장 높은 신뢰성
- 중요한 데이터에 사용

## 보안 설정

### 1. 사용자 인증

```bash
# 비밀번호 파일 생성
sudo mosquitto_passwd -c /etc/mosquitto/passwd username

# 설정 파일 수정
sudo nano /etc/mosquitto/mosquitto.conf
```

```conf
allow_anonymous false
password_file /etc/mosquitto/passwd
```

### 2. SSL/TLS 설정

```bash
# 인증서 생성
sudo openssl req -new -x509 -days 365 -extensions v3_ca -keyout ca.key -out ca.crt

# 설정 파일 수정
sudo nano /etc/mosquitto/mosquitto.conf
```

```conf
listener 8883
cafile /etc/mosquitto/certs/ca.crt
certfile /etc/mosquitto/certs/server.crt
keyfile /etc/mosquitto/certs/server.key
```

## 모니터링 및 디버깅

### 1. MQTT 브로커 로그 확인

```bash
# 실시간 로그 확인
sudo tail -f /var/log/mosquitto/mosquitto.log

# 연결 상태 확인
sudo mosquitto_sub -h localhost -t "amr/+/status/#" -v
```

### 2. 클라이언트 연결 테스트

```bash
# 구독 테스트
mosquitto_sub -h localhost -t "amr/+/status/#" -v

# 발행 테스트
mosquitto_pub -h localhost -t "amr/AMR001/status/test" -m '{"test": "message"}'
```

### 3. Python 디버깅 스크립트

```python
#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import json
import time

def on_connect(client, userdata, flags, rc):
    print(f"연결됨: {rc}")
    client.subscribe("amr/+/status/#")

def on_message(client, userdata, msg):
    print(f"토픽: {msg.topic}")
    print(f"메시지: {msg.payload.decode()}")

def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect("localhost", 1883, 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
```

## 성능 최적화

### 1. 연결 풀링

```python
import threading
from queue import Queue

class MQTTConnectionPool:
    def __init__(self, broker, port, pool_size=5):
        self.broker = broker
        self.port = port
        self.pool_size = pool_size
        self.connections = Queue()
        self.lock = threading.Lock()

        # 연결 풀 초기화
        for i in range(pool_size):
            client = self._create_client()
            self.connections.put(client)

    def _create_client(self):
        client = mqtt.Client()
        client.connect(self.broker, self.port)
        return client

    def get_connection(self):
        return self.connections.get()

    def return_connection(self, client):
        self.connections.put(client)
```

### 2. 메시지 배치 처리

```python
import asyncio
from collections import deque

class MessageBatcher:
    def __init__(self, batch_size=10, flush_interval=1.0):
        self.batch_size = batch_size
        self.flush_interval = flush_interval
        self.messages = deque()
        self.last_flush = time.time()

    def add_message(self, topic, message):
        self.messages.append((topic, message))

        if (len(self.messages) >= self.batch_size or
            time.time() - self.last_flush >= self.flush_interval):
            self.flush()

    def flush(self):
        if not self.messages:
            return

        # 배치 메시지 처리
        batch = list(self.messages)
        self.messages.clear()
        self.last_flush = time.time()

        # 실제 발행 로직
        for topic, message in batch:
            # MQTT 발행
            pass
```

## 설정 파일

### MQTT 설정 (config/mqtt_config.yaml)

```yaml
mqtt:
  broker: "localhost"
  port: 1883
  keepalive: 60
  client_id: "amr_client"

  # 보안 설정
  username: ""
  password: ""
  use_ssl: false

  # QoS 설정
  qos:
    status: 1
    command: 2
    sensor: 0
    alert: 2

  # 토픽 설정
  topics:
    status: "amr/{robot_id}/status"
    command: "amr/{robot_id}/command"
    sensor: "amr/{robot_id}/sensor"
    alert: "amr/{robot_id}/alert"

  # 재연결 설정
  reconnect:
    enabled: true
    max_attempts: 5
    delay: 1.0
```

## 문제 해결

### 일반적인 문제

#### 1. 연결 실패

```bash
# 브로커 상태 확인
sudo systemctl status mosquitto

# 포트 확인
netstat -tlnp | grep 1883

# 방화벽 확인
sudo ufw status
```

#### 2. 메시지 손실

- QoS 레벨 확인
- 네트워크 상태 확인
- 브로커 성능 확인

#### 3. 지연 시간

- 네트워크 대역폭 확인
- 메시지 크기 최적화
- 배치 처리 사용

### 디버깅 도구

```bash
# MQTT 브로커 통계
mosquitto_sub -h localhost -t '$SYS/#' -v

# 연결 수 확인
mosquitto_sub -h localhost -t '$SYS/broker/clients/connected' -v

# 메시지 처리량 확인
mosquitto_sub -h localhost -t '$SYS/broker/messages/received' -v
```

## 결론

MQTT는 IoT 장치 간의 효율적인 통신을 위한 강력한 프로토콜입니다. 적절한 설정과 모니터링을 통해 안정적인 통신 시스템을 구축할 수 있습니다.
