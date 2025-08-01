# AMR 센서 동작 및 데이터 전송 시스템

## 개요
이 시스템은 AMR(Autonomous Mobile Robot)의 센서 데이터를 동기화하고 MQTT를 통해 전송하는 기능을 제공합니다.

## 주요 기능
- **실제 모터 동작 테스트**: 모터 속도와 상태 확인
- **MQTT 전송 테스트**: 센서 데이터를 MQTT 브로커로 전송
- **지속적인 모터 동작**: 모터를 계속 돌리면서 데이터 송수신 확인
- **실제 모터 컨트롤러**: Motor_Driver_HAT 기반 실제 하드웨어 제어

## 실행 방법

### 메뉴 방식
```bash
./run_sensor_sync.sh
```

### 직접 실행
```bash
# 실제 모터 동작 테스트
python3 test_real_motor_speed.py

# MQTT 전송 테스트
python3 sensor_data_transmitter.py

# 지속적인 모터 동작 테스트
python3 test_continuous_motor.py

# 실제 모터 컨트롤러 테스트
python3 real_motor_controller.py
```

## 파일 구조

### 핵심 파일들
- `run_sensor_sync.sh` - 메인 실행 스크립트
- `amr_real_data_sync.py` - AMR 데이터 동기화 시스템
- `motor_speed_monitor.py` - 모터 속도 모니터링
- `real_motor_controller.py` - 실제 모터 컨트롤러
- `sensor_data_sync.py` - 센서 데이터 동기화
- `sensor_data_transmitter.py` - MQTT 데이터 전송

### 테스트 파일들
- `test_real_motor_speed.py` - 실제 모터 속도 테스트
- `test_continuous_motor.py` - 지속적인 모터 동작 테스트
- `test_motor_with_data_transmission.py` - 모터 + MQTT 통합 테스트

## 요구사항
- Python 3.6+
- paho-mqtt
- smbus (I2C 통신용)

## 설치
```bash
pip install paho-mqtt
```

## 주의사항
- 실제 모터 하드웨어를 사용하려면 Motor_Driver_HAT이 연결되어야 합니다
- MQTT 테스트를 위해서는 MQTT 브로커가 필요합니다
- 실제 하드웨어 제어 시 주의가 필요합니다 