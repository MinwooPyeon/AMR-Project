# AMR Embedded System

자율주행 모바일 로봇(AMR)의 임베디드 제어 시스템입니다.  
Jetson Nano 위에서 동작하며 모터 제어, IMU 센서, MQTT 통신, 배터리 모니터링 등을 담당합니다.

---

## 목차

- [시스템 구조](#시스템-구조)
- [주요 모듈](#주요-모듈)
- [통신 흐름](#통신-흐름)
- [설치](#설치)
- [설정](#설정)
- [실행](#실행)
- [테스트](#테스트)

---

## 시스템 구조

```
Embedded/
├── config/               # 시스템 전역 설정
│   ├── system_config.py  # 통합 설정 진입점
│   ├── motor_config.py
│   ├── mqtt_config.py
│   └── sensor_config.py
│
├── motors/               # 모터 제어
│   ├── PCA9685.py        # I2C PWM 드라이버
│   ├── amr_motor_controller.py  # 메인 모터 컨트롤러 (IMU 포함)
│   ├── integrated_motor_control.py
│   └── motor_control_enhanced.py
│
├── mqtt_module/          # MQTT 통신
│   ├── base_mqtt_client.py      # 공통 추상 클라이언트
│   ├── ai_mqtt_client.py        # AI 서버 → 로봇 수신
│   ├── backend_mqtt_subscriber.py  # 백엔드 명령 수신
│   ├── sensor_data_transmitter.py  # 로봇 상태 송신
│   └── mqtt_manager.py          # 전체 MQTT 통합 관리
│
├── battery_module/       # 배터리 모니터링 (MAX17048)
├── ai_module/            # ROS2 기반 AI 연동
├── security/             # 인증 및 암호화
├── utilities/            # 로거, 예외 클래스
├── tests/                # 단위/통합/E2E 테스트
└── launch/               # 실행 진입점
    └── main.py           # 모터 드라이버 직접 테스트
```

---

## 주요 모듈

### 모터 제어 (`motors/`)

| 클래스 | 역할 |
|--------|------|
| `PCA9685` | I2C(smbus2)로 PWM 신호 생성 |
| `AMRMotorController` | 좌/우 모터 + IMU 기반 PID 회전 제어 |
| `IntegratedMotorControl` | 키보드 수동 조작 + MQTT 연동 |

**하드웨어 핀 배치 (PCA9685)**

| 채널 | 기능 |
|------|------|
| 0 (PWMA) | 좌측 모터 속도 |
| 1 (AIN1) | 좌측 모터 방향 A |
| 2 (AIN2) | 좌측 모터 방향 B |
| 3 (BIN1) | 우측 모터 방향 A |
| 4 (BIN2) | 우측 모터 방향 B |
| 5 (PWMB) | 우측 모터 속도 |

---

### MQTT 통신 (`mqtt_module/`)

```
AI 서버 ──[ai_data]──────────► AIMQTTClient       ─┐
                                                    ├─► MQTTManager ─► 모터 제어
Backend ──[command/{id}]─────► BackendSubscriber   ─┘

로봇 ────[status/{id}]───────► Backend (상태 전송)
         SensorDataTransmitter
```

| 토픽 | 방향 | 내용 |
|------|------|------|
| `ai_data` | AI → 로봇 | 위치, 상황, 이미지 |
| `status/{robot_id}` | 로봇 → 백엔드 | 속도, 위치, 상태 |
| `command/{robot_id}` | 백엔드 → 로봇 | 이동 명령 |
| `alert` | 백엔드 → 로봇 | 긴급 상황 알림 |

---

### 설정 (`config/`)

모든 설정은 환경변수 → 기본값 순으로 적용됩니다.

```python
from config.system_config import get_config
config = get_config()

config.MQTT_BROKER      # MQTT 브로커 IP
config.MOTOR_MAX_SPEED  # 모터 최대 속도
config.IMU_I2C_ADDRESS  # IMU I2C 주소
```

---

## 통신 흐름

```
┌─────────────┐    MQTT     ┌──────────────────┐    I2C      ┌───────────┐
│  AI 서버    │ ──────────► │  Jetson Nano     │ ──────────► │ PCA9685   │
│ (localhost) │   ai_data   │  (이 시스템)      │  0x40       │ (모터)    │
└─────────────┘             │                  │             └───────────┘
                            │                  │    I2C
┌─────────────┐    MQTT     │                  │ ──────────► ┌───────────┐
│  Backend    │ ◄─────────► │                  │  0x68       │ MPU-6050  │
│  서버       │   status/   │                  │             │ (IMU)     │
└─────────────┘   command/  └──────────────────┘             └───────────┘
```

---

## 설치

### 요구사항

- Python 3.8+
- Jetson Nano (또는 I2C 지원 Linux 보드)
- ROS2 Humble (ai_module 사용 시)

### 패키지 설치

```bash
pip install -r requirements.txt
```

주요 의존성:

| 패키지 | 용도 |
|--------|------|
| `smbus2` | I2C 통신 |
| `paho-mqtt` | MQTT 통신 |
| `cryptography` | 데이터 암호화 |
| `pyyaml` | YAML 설정 파일 |

### I2C 권한 설정

```bash
sudo chmod 666 /dev/i2c-*
# 또는 영구 설정
sudo usermod -aG i2c $USER
```

---

## 설정

`.env.example`을 복사해 실제 값을 채워넣으세요.

```bash
cp .env.example .env
```

```bash
# .env
MQTT_BROKER=192.168.100.141
MQTT_PORT=1883
MQTT_USERNAME=your_username
MQTT_PASSWORD=your_password

MQTT_LOCAL_BROKER=localhost
MQTT_LOCAL_PORT=1883

SYSTEM_NAME=AMR001
DEBUG_MODE=false
```

실행 전 환경변수 로드:

```bash
export $(cat .env | xargs)
```

### 사용자 인증 설정

비밀번호는 SHA-256 해시로 환경변수에 저장합니다.

```bash
# 해시 생성
python3 -c "import hashlib; print(hashlib.sha256(b'yourpassword').hexdigest())"

# .env에 추가
AMR_USER_ADMIN=<생성된_해시값>
```

---

## 실행

### 설정 확인

```bash
python3 -c "from config.system_config import get_config; get_config().print_config()"
```

### 모터 직접 테스트

```bash
cd Embedded
python3 launch/main.py
```

### 통합 모터 제어 (키보드 + MQTT)

```bash
python3 motors/integrated_motor_control.py
```

키 조작:

| 키 | 동작 |
|----|------|
| `W` | 전진 |
| `S` | 후진 |
| `A` | 좌회전 |
| `D` | 우회전 |
| `Space` | 정지 |
| `+` / `-` | 속도 증가/감소 |
| `Q` | 종료 |

### IMU 기반 정밀 회전 테스트

```bash
python3 motors/amr_motor_controller.py
```

---

## 테스트

```bash
# 전체 테스트
python3 -m pytest tests/ -v

# 단위 테스트
python3 -m pytest tests/unit/ -v

# 통합 테스트
python3 -m pytest tests/integration/ -v
```

---

## 트러블슈팅

### I2C 장치를 찾을 수 없음

```bash
# 연결된 I2C 장치 확인
i2cdetect -y 1
# PCA9685: 0x40, MPU-6050: 0x68 이 보여야 함

# jtop이 I2C를 점유하고 있을 경우
sudo pkill -f jtop
```

### MQTT 연결 실패

```bash
# 브로커 상태 확인
systemctl status mosquitto

# 연결 테스트
mosquitto_sub -h $MQTT_BROKER -t "#" -u $MQTT_USERNAME -P $MQTT_PASSWORD
```

### 환경변수 미설정

```bash
# 현재 설정 확인
python3 -c "from config.mqtt_config import MQTTConfig; c = MQTTConfig(); print(c.broker, c.username)"
```
