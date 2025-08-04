# 모터 제어 모듈

AMR의 모터 제어 및 속도 모니터링을 담당하는 모듈들입니다.

## 파일 구조

- `real_motor_controller.py`: 실제 모터 컨트롤러
- `motor_speed_monitor.py`: 모터 속도 모니터

## 주요 기능

- 모터 제어 (전진, 후진, 회전, 정지)
- 실시간 속도 모니터링
- 속도 이력 관리
- 안전 기능 (속도 제한)

## 사용법

```python
from motor_control import RealMotorController, MotorController, MotorSpeedMonitor

# 실제 모터 컨트롤러 생성
motor_controller = RealMotorController()

# 모터 제어
motor_controller.move_forward(50.0)
motor_controller.turn_left(30.0)
motor_controller.stop()

# 속도 모니터링
monitor = MotorSpeedMonitor()
monitor.set_motor_controller(motor_controller)
monitor.start_monitoring()
``` 