# 키보드 수동 제어 시스템

이 시스템은 키보드 입력을 통해 AMR 로봇을 수동으로 조작할 수 있는 컨트롤러입니다.

## 기능

- **키보드 입력 처리**: Windows와 Linux 환경 모두 지원
- **모터 제어**: 전진, 후진, 좌회전, 우회전
- **IMU 기반 정확한 회전**: 90도 정확한 회전
- **속도 조절**: 4단계 속도 설정 (20%, 50%, 80%, 100%)
- **서보 모터 제어**: UP/DOWN/중앙 위치 제어
- **실시간 상태 모니터링**: 모터 상태와 각도 정보 출력

## 파일 구조

```
motor_control/
├── manual_keyboard_controller.py    # 메인 키보드 컨트롤러
├── test_manual_control.py          # 테스트 스크립트
├── imu_ai_motor_controller.py      # 기존 IMU AI 모터 컨트롤러
└── README_manual_control.md        # 이 파일
```

## 설치 및 실행

### 1. 기본 실행

```bash
cd motor_control
python3 manual_keyboard_controller.py
```

### 2. 테스트 실행

```bash
cd motor_control
python3 test_manual_control.py
```

## 키보드 조작법

### 이동 제어

- **W** - 전진
- **S** - 후진
- **A** - 좌회전
- **D** - 우회전
- **SPACE** - 정지

### 회전 제어

- **Q** - 90도 좌회전 (IMU 기반 정확한 회전)
- **E** - 90도 우회전 (IMU 기반 정확한 회전)
- **R** - 정중앙 맞추기

### 속도 제어

- **1** - 저속 (20%)
- **2** - 중속 (50%) - 기본값
- **3** - 고속 (80%)
- **4** - 최고속 (100%)

### 서보 제어

- **Z** - 서보 UP
- **X** - 서보 DOWN
- **C** - 서보 중앙

### 기타

- **ESC** 또는 **Ctrl+C** - 종료

## 시스템 요구사항

### 하드웨어

- PCA9685 모터 드라이버 (I2C 주소: 0x40)
- GY-BN008x IMU 센서 (선택사항)
- 서보 모터 (선택사항)

### 소프트웨어

- Python 3.6+
- smbus2 라이브러리
- adafruit-circuitpython-bno08x (IMU 사용 시)

## 설정

### I2C 설정

기본적으로 I2C 버스 1을 사용합니다. 다른 버스를 사용하려면:

```python
controller = ManualKeyboardController(i2c_bus=0)  # 버스 0 사용
```

### Backend 연결 설정

MQTT Backend 연결을 위한 설정:

```python
controller = ManualKeyboardController(
    backend_broker="192.168.100.141",
    backend_port=1883
)
```

## 주요 클래스

### ManualKeyboardController

기존 `IMUAIMotorController`를 상속받아 키보드 입력 처리 기능을 추가한 클래스입니다.

#### 주요 메서드

- `start_keyboard_control()`: 키보드 제어 시작
- `stop_keyboard_control()`: 키보드 제어 정지
- `run_manual_control()`: 수동 제어 메인 루프
- `get_current_status()`: 현재 상태 정보 반환

## 상태 모니터링

시스템은 5초마다 현재 상태를 출력합니다:

```
[상태] 모터A: 50%(forward), 모터B: 50%(forward), 각도: 0.0°, 속도: 50%
```

## 문제 해결

### 1. 키보드 입력이 인식되지 않는 경우

**Windows 환경:**

- 관리자 권한으로 실행
- 다른 프로그램이 키보드를 점유하고 있는지 확인

**Linux 환경:**

- 터미널에서 실행 중인지 확인
- 다른 프로그램이 터미널을 점유하고 있는지 확인

### 2. 모터가 동작하지 않는 경우

- I2C 연결 확인
- PCA9685 드라이버 주소 확인 (기본: 0x40)
- 전원 공급 확인

### 3. IMU 센서 오류

- I2C 버스 번호 확인
- 센서 연결 상태 확인
- 필요한 라이브러리 설치 확인

## 예제 코드

### 기본 사용법

```python
from manual_keyboard_controller import ManualKeyboardController

# 컨트롤러 초기화
controller = ManualKeyboardController(debug=True)

# 수동 제어 시작
controller.run_manual_control()
```

### 커스텀 설정

```python
# 커스텀 설정으로 초기화
controller = ManualKeyboardController(
    i2c_address=0x40,
    i2c_bus=1,
    debug=True,
    backend_broker="192.168.100.141",
    backend_port=1883
)

# 시리얼 번호 설정
controller.set_serial_number("AMR001")

# 수동 제어 시작
controller.run_manual_control()
```

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다.

## 기여

버그 리포트나 기능 요청은 이슈 트래커를 통해 제출해 주세요.
