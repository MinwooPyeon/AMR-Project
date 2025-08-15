# IMU 센서 및 모터 통합 시스템 가이드

## 개요

이 문서는 GY-BN008x IMU 센서와 Motor Driver HAT을 사용한 자율주행 모바일 로봇(AMR)의 통합 시스템에 대한 가이드

## 하드웨어 구성

### 필요한 구성 요소

- **Jetson Nano 또는 Raspberry Pi**
- **GY-BN008x IMU 센서 (MPU6050 기반)**
- **Waveshare Motor Driver HAT (PCA9685 기반)**
- **DC 모터 2개 (차동 구동)**
- **I2C 연결 케이블**

### 연결 방법

```
IMU 센서 (GY-BN008x):
- VCC → 3.3V
- GND → GND
- SCL → I2C SCL
- SDA → I2C SDA
- 주소: 0x68

Motor Driver HAT:
- VCC → 5V
- GND → GND
- SCL → I2C SCL
- SDA → I2C SDA
- 주소: 0x40
```

## 설치 및 설정

### 1. 의존성 설치

```bash
# Python 패키지 설치
pip3 install smbus2 requests paho-mqtt Flask

# 또는 requirements.txt 사용
pip3 install -r motor_driver/motor_driver/Jetson\ Nano/python3/requirements.txt

# I2C 활성화 (Jetson Nano)
sudo usermod -a -G i2c $USER
sudo reboot
```

### 2. I2C 확인

```bash
# I2C 장치 확인
i2cdetect -y 1

# 예상 출력:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
# 70: -- -- -- -- -- -- -- --
```

### 3. 권한 설정

```bash
# I2C 접근 권한
sudo chmod 666 /dev/i2c-1
```

## 사용 방법

### 1. 기본 테스트

```bash
# 기본 동작 테스트
python3 test_imu_ai_simple.py

# 메뉴에서 테스트 선택:
# 1. 기본 동작 테스트
# 2. IMU 제어 테스트
# 3. AI 통합 테스트
# 4. PID 튜닝 테스트
# 5. 전체 테스트
```

### 2. 직접 사용

```python
from imu_ai_motor_controller import IMUAIMotorController

# 컨트롤러 초기화
controller = IMUAIMotorController(
    debug=True,
    api_url="http://localhost:5001/pose",
    backend_broker="192.168.100.141",
    backend_port=1883
)

# 시리얼 번호 설정
controller.set_serial_number("AMR001")

# Backend 연결
controller.connect_backend()

# IMU 제어 루프 시작
controller.start_control_loop()

# 90도 좌회전
controller.turn_left_90()

# 90도 우회전
controller.turn_right_90()

# 전진
controller.differential_drive(40, 40)

# 후진
controller.differential_drive(-40, -40)

# 정지
controller.stop_all()

# AI 제어 루프 실행
controller.run_ai_control_loop(interval=1.0)
```

### 3. 웹 API 명령 제어

```bash
# Flask 웹 서버 시작 (옵션 5 선택)
python3 imu_ai_motor_controller.py
# 메뉴에서 "5. Flask 웹 서버 + 명령 처리 시작" 선택
```

#### API 엔드포인트

- **GET /command**: 현재 명령 조회
- **POST /command**: 명령 설정 (JSON: `{"code": 0-4}`)
- **GET /status**: 서버 상태 조회

#### 명령 코드

- **0**: STOP (정지)
- **1**: MOVING_FORWARD (전진)
- **2**: MOVING_BACKWARD (후진)
- **3**: ROTATE_LEFT (좌회전)
- **4**: ROTATE_RIGHT (우회전)

#### 사용 예시

```bash
# 현재 명령 조회
curl http://localhost:5000/command

# 전진 명령
curl -X POST http://localhost:5000/command \
  -H "Content-Type: application/json" \
  -d '{"code": 1}'

# 정지 명령
curl -X POST http://localhost:5000/command \
  -H "Content-Type: application/json" \
  -d '{"code": 0}'
```

#### Python 테스트 스크립트

```bash
# API 테스트 실행
python3 test_command_api.py
```

## 설정 및 튜닝

### PID 게인 설정

```python
# PID 게인 조정
controller.set_pid_gains(kp=2.0, ki=0.1, kd=0.5)

# 권장 값:
# - Kp (비례): 1.5 ~ 3.0
# - Ki (적분): 0.05 ~ 0.2
# - Kd (미분): 0.3 ~ 0.8
```

### 모터 속도 설정

```python
# 모터 속도 설정
speeds = {
    'forward': 40,    # 전진 속도
    'backward': 40,   # 후진 속도
    'left': 40,       # 좌회전 속도
    'right': 40,      # 우회전 속도
    'stop': 0,        # 정지
    'custom': 40      # 커스텀
}
controller.set_motor_speed_config(speeds)
```

## API 참조

### 주요 메서드

#### 초기화 및 설정

- `__init__(i2c_address, i2c_bus, debug, api_url, backend_broker, backend_port)`
- `initialize_imu()`: IMU 초기화 및 캘리브레이션
- `set_serial_number(serial)`: 시리얼 번호 설정
- `connect_backend()`: Backend MQTT 연결

#### IMU 제어

- `turn_left_90()`: 90도 좌회전
- `turn_right_90()`: 90도 우회전
- `turn_to_angle(target_angle)`: 지정 각도로 회전
- `get_current_angle()`: 현재 각도 반환
- `is_turning_now()`: 회전 중인지 확인

#### 모터 제어

- `differential_drive(left_speed, right_speed)`: 차동 구동
- `stop_all()`: 모든 모터 정지
- `set_motor_speed(motor, direction, speed)`: 개별 모터 제어

#### AI 통합

- `run_ai_control_loop(interval)`: AI 제어 루프 실행

#### 웹 API 명령 제어

- `start_command_processor()`: 명령 처리 스레드 시작
- `stop_command_processor()`: 명령 처리 스레드 중지
- `process_command(command_code)`: 명령 코드 처리
- `get_current_command()`: 현재 명령 상태 반환
- `process_ai_command(ai_data)`: AI 명령 처리
- `send_to_backend(ai_data, motor_status)`: Backend 데이터 전송

#### 설정

- `set_pid_gains(kp, ki, kd)`: PID 게인 설정
- `set_motor_speed_config(speeds)`: 모터 속도 설정

## AI 명령 형식

### 입력 데이터 (AI API)

```json
{
  "serial": "AMR001",
  "x": 0.5, // X 좌표 (-1.0 ~ 1.0)
  "y": 0.8, // Y 좌표 (-1.0 ~ 1.0)
  "case": "forward",
  "timeStamp": "2024-01-01T12:00:00Z"
}
```

### 출력 데이터 (Backend)

```json
{
  "serial": "AMR001",
  "state": "RUNNING",
  "x": "10.0",
  "y": "10.0",
  "speed": "25.0",
  "battery_level": "85.5",
  "motor_status": {
    "left_speed": 40,
    "right_speed": 40,
    "direction": "forward"
  }
}
```

## 주의사항

### 안전 주의사항

1. **모터 연결 확인**: 모터가 올바르게 연결되었는지 확인
2. **전원 공급**: 안정적인 전원 공급 확인
3. **초기 테스트**: 낮은 속도로 먼저 테스트
4. **비상 정지**: Ctrl+C로 즉시 정지 가능

### 하드웨어 주의사항

1. **I2C 주소 확인**: IMU(0x68), 모터 드라이버(0x40)
2. **배선 확인**: VCC, GND, SCL, SDA 연결 상태
3. **권한 설정**: I2C 접근 권한 확인

## 문제 해결

### 일반적인 문제

#### 1. IMU 연결 실패

```bash
# I2C 장치 확인
i2cdetect -y 1

# 권한 확인
ls -la /dev/i2c-1

# 권한 수정
sudo chmod 666 /dev/i2c-1
```

#### 2. 모터 동작 안함

```python
# 모터 드라이버 연결 확인
controller = IMUAIMotorController(debug=True)
# 초기화 메시지 확인
```

#### 3. 각도 부정확

```python
# PID 게인 조정
controller.set_pid_gains(kp=2.5, ki=0.15, kd=0.6)

# 캘리브레이션 재실행
controller.initialize_imu()
```

#### 4. AI API 연결 실패

```python
# API URL 확인
controller = IMUAIMotorController(api_url="http://localhost:5001/pose")

# 네트워크 연결 확인
import requests
response = requests.get("http://localhost:5001/pose")
```

## 파일 구조

```
├── imu_ai_motor_controller.py    # 메인 컨트롤러
├── test_imu_ai_simple.py         # 테스트 스크립트
├── readme_imu_ai_motor.md        # 이 파일
├── pca9685.py                    # PWM 드라이버 (외부)
└── mqtt/
    └── sensor_data_transmitter.py # MQTT 통신 (외부)
```
