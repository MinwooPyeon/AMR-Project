# AI 모듈

AMR(Autonomous Mobile Robot) 시스템의 AI 관련 기능을 제공하는 Python 패키지

## 개요

AI 시스템과 임베디드 시스템 간의 통신을 담당

- **AI Alert Publisher**: AI에서 감지한 이상 상황을 ROS2와 MQTT를 통해 발행
- **AI Data Simulator**: AI 데이터 시뮬레이션 및 테스트
- **AI File Client**: 파일 기반 AI 데이터 수신 및 모니터링
- **AI Position Subscriber**: AI 위치 및 명령 데이터 구독

## 설치

### 의존성 설치

```bash
pip install -r requirements.txt
```

### 환경 설정

`.env` 파일을 생성하여 환경 변수를 설정할 수 있습니다:

```env
# MQTT 설정
MQTT_BROKER=localhost
MQTT_PORT=1883
MQTT_KEEPALIVE=60

# AI 데이터 파일 경로
AI_DATA_FILE_PATH=/tmp/ai_data.json
AI_IMAGES_PATH=ai_images

# 기본 로봇 ID
DEFAULT_ROBOT_ID=AMR001

# 로그 레벨
LOG_LEVEL=INFO
```

## 사용법

### 1. AI Alert Publisher

AI에서 감지한 이상 상황 발행

```python
from ai import AIAlertPublisher, AlertSituation

# Alert Publisher 초기화
publisher = AIAlertPublisher("AMR001")

# 이상 상황 발행
publisher.publish_collapse_alert(
    image="base64_encoded_image",
    x=10.5,
    y=20.3,
    detail="사람이 쓰러짐을 감지했습니다"
)

publisher.publish_smoke_alert(
    image="base64_encoded_image",
    x=15.2,
    y=8.7,
    detail="연기를 감지했습니다"
)
```

### 2. AI Data Simulator

AI 데이터 시뮬레이션

```python
from ai import AIDataSimulator

# 시뮬레이터 초기화
simulator = AIDataSimulator()

# 시뮬레이션 시작
simulator.start_simulation(interval=2.0)

# 긴급 상황 데이터 생성
simulator.create_emergency_data()

# 장애물 감지 데이터 생성
simulator.create_obstacle_data()
```

### 3. AI File Client

파일 기반 AI 데이터를 수신하고 모니터링

```python
from ai import AIFileClient

def on_ai_data(data):
    print(f"AI 데이터 수신: {data}")

# File Client 초기화
client = AIFileClient()

# 콜백 함수 설정
client.set_ai_data_callback(on_ai_data)

# 모니터링 시작
client.start_monitoring(interval=1.0)

# 현재 데이터 조회
current_data = client.get_current_data()

# 통계 정보 조회
stats = client.get_statistics()
```

### 4. AI Position Subscriber

AI 위치 및 명령 데이터를 구독

```python
from ai import AIPositionSubscriber, AICommand

def on_command(command):
    print(f"명령 수신: {command.value}")

def on_image(image_path):
    print(f"이미지 저장: {image_path}")

def on_position(x, y):
    print(f"위치 업데이트: ({x}, {y})")

# Subscriber 초기화
subscriber = AIPositionSubscriber()

# 콜백 함수 설정
subscriber.set_command_callback(on_command)
subscriber.set_image_callback(on_image)
subscriber.set_position_callback(on_position)
```

## 데이터 구조

### AI 데이터 형식

```json
{
  "serial": "AMR001",
  "x": 10.5,
  "y": 20.3,
  "img": "base64_encoded_image_data",
  "case": "obstacle_detected",
  "timeStamp": "2024-01-01T12:00:00"
}
```

### Alert 데이터 형식

```json
{
  "timestamp": 1704110400,
  "situation": "COLLAPSE",
  "x": 10.5,
  "y": 20.3,
  "amr_serial": "AMR001",
  "image": "base64_encoded_image",
  "detail": "사람이 쓰러짐을 감지했습니다"
}
```

## 설정

### AIConfig 클래스

```python
from ai import AIConfig

# MQTT 설정 조회
mqtt_config = AIConfig.get_mqtt_config()

# ROS 설정 조회
ros_config = AIConfig.get_ros_config()
```

## 유틸리티 함수

### 로깅

```python
from ai import setup_logger

logger = setup_logger("MyModule", "DEBUG")
logger.info("정보 메시지")
logger.error("오류 메시지")
```

### 이미지 처리

```python
from ai import encode_image_to_base64, decode_base64_to_image

# 이미지를 Base64로 인코딩
base64_data = encode_image_to_base64("image.jpg")

# Base64를 이미지로 디코딩
image_path = decode_base64_to_image(base64_data)
```

### JSON 처리

```python
from ai import create_json_message, parse_json_message, validate_ai_data

# JSON 메시지 생성
json_str = create_json_message({"key": "value"})

# JSON 메시지 파싱
data = parse_json_message(json_str)

# AI 데이터 유효성 검사
is_valid = validate_ai_data(data)
```

## 테스트

### 개별 모듈 테스트

```bash
# AI Data Simulator 테스트
python -m ai.ai_data_simulator

# AI File Client 테스트
python -m ai.ai_file_client

# AI Position Subscriber 테스트
python -m ai.ai_position_subscriber
```

### 통합 테스트

```python
from ai import AIDataSimulator, AIFileClient

# 시뮬레이터와 클라이언트를 함께 사용
simulator = AIDataSimulator()
client = AIFileClient()

def on_data(data):
    print(f"데이터 수신: {data}")

client.set_ai_data_callback(on_data)
client.start_monitoring()

# 시뮬레이션 시작
simulator.start_simulation()
```

## 문제 해결

### MQTT 연결 실패

1. MQTT 브로커가 실행 중인지 확인
2. 포트와 브로커 주소가 올바른지 확인
3. 방화벽 설정 확인

### 파일 접근 오류

1. 파일 경로가 올바른지 확인
2. 파일 권한 확인
3. 디렉토리가 존재하는지 확인

### ROS2 연결 실패

1. ROS2 환경이 설정되었는지 확인
2. ROS2 노드가 실행 중인지 확인
3. 토픽 이름이 올바른지 확인
