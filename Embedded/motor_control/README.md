# 모터 제어 모듈

AMR의 모터와 서보모터를 제어하는 모듈들입니다.

## 파일 구조

- `pca9685_motor_controller.py`: **PCA9685 기반 통합 모터 컨트롤러** (추천)
- `real_motor_controller.py`: 실제 모터 컨트롤러
- `servo_motor_controller.py`: 서보모터 컨트롤러
- `lift_controller.py`: 리프트 컨트롤러

## PCA9685 통합 모터 컨트롤러

### 주요 기능

- **모터 제어**: 좌측/우측 모터 속도 및 방향 제어
- **서보모터 제어**: 4개 서보모터 각도 제어
- **통합 제어**: 하나의 PCA9685 드라이버로 모든 모터 제어
- **스레드 안전**: 멀티스레드 환경에서 안전한 제어

### 하드웨어 연결

#### 모터 연결 (PCA9685 채널)
- **좌측 모터**: 
  - PWM: 채널 0
  - IN1: 채널 1
  - IN2: 채널 2
- **우측 모터**:
  - PWM: 채널 5
  - IN1: 채널 3
  - IN2: 채널 4

#### 서보모터 연결 (PCA9685 채널)
- **servo1**: 채널 6
- **servo2**: 채널 7
- **servo3**: 채널 8
- **servo4**: 채널 9

### 사용법

```python
from motor_control.pca9685_motor_controller import PCA9685MotorController

# 컨트롤러 초기화
controller = PCA9685MotorController(
    i2c_address=0x40,  # PCA9685 I2C 주소
    i2c_bus=1,         # I2C 버스 번호
    debug=True          # 디버그 모드
)

# 모터 제어
controller.move_forward(50)    # 전진 (50% 속도)
controller.turn_left(30)       # 좌회전 (30% 속도)
controller.turn_right(30)      # 우회전 (30% 속도)
controller.move_backward(50)   # 후진 (50% 속도)
controller.stop_motors()       # 정지

# 서보모터 제어
controller.set_servo_angle('servo1', 90)  # servo1을 90도로
controller.set_all_servos(90)             # 모든 서보모터를 90도로

# 상태 조회
motor_status = controller.get_motor_status()
servo_status = controller.get_servo_status()

# 정리
controller.cleanup()
```

### 모터 제어 메서드

- `set_motor_speed(left_speed, right_speed)`: 좌우 모터 속도 설정 (-100 ~ 100)
- `move_forward(speed)`: 전진
- `move_backward(speed)`: 후진
- `turn_left(speed)`: 좌회전
- `turn_right(speed)`: 우회전
- `stop_motors()`: 정지

### 서보모터 제어 메서드

- `set_servo_angle(servo_name, angle)`: 특정 서보모터 각도 설정
- `set_all_servos(angle)`: 모든 서보모터 각도 설정
- `get_servo_status()`: 서보모터 상태 조회

### 테스트

```bash
python motor_control/pca9685_motor_controller.py
```

## 기존 모듈들

### RealMotorController
실제 모터 제어를 위한 클래스입니다.

### ServoMotorController
서보모터 제어를 위한 클래스입니다.

### LiftController
리프트 시스템 제어를 위한 클래스입니다. 