# Motors 폴더

Motor Driver HAT을 사용한 DC 모터 제어 및 AI 통신 기능이 포함된 통합 폴더

## 폴더 구조

```
motors/
├── ai_motor_controller.py      # AI 통신 모터 제어 클래스
├── amr_motor_controller.py     # AMR 모터 제어 클래스
├── integrated_motor_control.py # 통합 모터 제어 시스템
├── motor_control_enhanced.py   # 향상된 모터 제어
├── PCA9685.py                  # PCA9685 PWM 컨트롤러 라이브러리
├── servo_motor_controller.py   # 서보 모터 제어
├── requirements.txt            # Python 의존성
├── __init__.py                 # 패키지 초기화
└── readme.md                   # 이 파일
```

## 주요 파일

### ai_motor_controller.py

AI 통신을 위한 모터 제어 클래스:

- Motor Driver HAT을 사용한 DC 모터 제어
- AI 통신 기능
- Backend 통신
- 모터 방향 및 속도 제어
- 실시간 상태 모니터링

### amr_motor_controller.py

AMR(Autonomous Mobile Robot) 모터 제어 클래스:

- 자율주행 로봇을 위한 모터 제어
- 고급 모터 제어 알고리즘
- 안전 기능 및 제한 설정

### integrated_motor_control.py

통합 모터 제어 시스템:

- 다양한 모터 제어 방식 통합
- 하드웨어 추상화 계층
- 플러그인 방식의 모터 드라이버 지원

### motor_control_enhanced.py

향상된 모터 제어 기능:

- 고급 모터 제어 알고리즘
- 부드러운 가속/감속
- 안전 기능

### PCA9685.py

PCA9685 PWM 컨트롤러 라이브러리:

- I2C 통신을 통한 PWM 제어
- 16채널 PWM 출력
- 모터 및 서보 제어 지원

### servo_motor_controller.py

서보 모터 제어 시스템:

- 서보 모터 제어 기능
- 각도 제어 및 위치 제어
- 다중 서보 동시 제어

## 하드웨어 구성

### Motor Driver HAT

- **I2C 주소**: 0x40 (기본값)
- **PWM 주파수**: 20Hz (강한 토크)
- **모터 A**: PWMA(0), AIN1(1), AIN2(2)
- **모터 B**: PWMB(5), BIN1(3), BIN2(4)

### 모터 제어 방식

- **전진**: 양쪽 모터 동시 전진
- **후진**: 양쪽 모터 동시 후진
- **좌회전**: 왼쪽 모터 후진, 오른쪽 모터 전진
- **우회전**: 왼쪽 모터 전진, 오른쪽 모터 후진

## 사용 방법

### AI 모터 제어

```python
from ai_motor_controller import AIMotorController

motor = AIMotorController(debug=True)
motor.connect_backend()
motor.run_ai_control_loop()
```

### AMR 모터 제어

```python
from amr_motor_controller import AMRMotorController

motor = AMRMotorController()
motor.initialize()
motor.set_speed(50, 50)
```

### 통합 모터 제어

```python
from integrated_motor_control import IntegratedMotorControl

motor = IntegratedMotorControl()
motor.setup()
motor.forward(50)
```

### 서보 모터 제어

```python
from servo_motor_controller import ServoMotorController

servo = ServoMotorController()
servo.set_angle(0, 90)
servo.set_angle(1, 180)
```

## 모터 속도 설정

기본 모터 속도 설정:

- **전진**: 50%
- **후진**: 50%
- **좌회전**: 50%
- **우회전**: 50%
- **정지**: 0%

## AI 통신 설정

### API 설정

- **기본 URL**: http://127.0.0.1:5001
- **타임아웃**: 5초
- **재시도 횟수**: 3회

### MQTT 설정

- **Backend 브로커**: 192.168.100.141
- **포트**: 1883
- **클라이언트 ID**: AMR001

## 의존성

### Python 패키지

```
requests>=2.25.1
paho-mqtt>=1.5.1
smbus2>=0.4.1
```

### 시스템 요구사항

- Python 3.7+
- I2C 활성화
- 적절한 권한 설정

## 주의사항

1. **I2C 권한**: I2C 장치에 대한 적절한 권한이 필요합니다
2. **전원 공급**: 모터에 충분한 전원이 공급되어야 합니다
3. **연결 상태**: 하드웨어 연결 상태를 확인하세요
4. **안전**: 모터 테스트 시 안전한 환경에서 실행하세요

## 문제 해결

### 연결 오류

1. I2C 장치 권한 확인
2. 하드웨어 연결 상태 확인
3. 전원 공급 상태 확인

### 모터 동작 안됨

1. 전원 공급 상태 확인
2. I2C 주소 확인 (기본값: 0x40)
3. PWM 주파수 설정 확인

### AI 통신 오류

1. 서버 프로세스 실행 상태 확인
2. 네트워크 연결 상태 확인
3. API URL 및 포트 설정 확인
