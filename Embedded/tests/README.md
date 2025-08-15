# 테스트 모듈

AMR 시스템의 핵심 기능들을 테스트하는 모듈들입니다.

## 파일 구조

### 핵심 테스트

- `ai_communication_test.py`: AI 통신 테스트
- `angle_control_test.py`: 각도 제어 테스트
- `imu_sensor_test.py`: IMU 센서 테스트
- `mqtt_test.py`: MQTT 통신 테스트
- `obstacle_detection_test.py`: 장애물 감지 테스트
- `rotation_detection_test.py`: 회전 감지 테스트

### 테스트 실행 도구

- `run_tests.py`: 통합 테스트 실행 스크립트
- `motor_test.cpp`: 모터 테스트 (C++)

### AI 관련 테스트

- `test_ai_file_client.py`: AI 파일 클라이언트 테스트
- `test_ai_mqtt_client.py`: AI MQTT 클라이언트 테스트
- `test_ai_mqtt_receiver.py`: AI MQTT 수신기 테스트
- `test_backend_mqtt_subscriber.py`: 백엔드 MQTT 구독자 테스트
- `test_mqtt_manager.py`: MQTT 매니저 테스트

## 주요 기능

### 통신 테스트

- AI 서버와의 통신 검증
- MQTT 브로커 연결 및 메시지 송수신
- 백엔드 시스템과의 통신

### 센서 테스트

- IMU 센서 데이터 읽기 및 캘리브레이션
- 장애물 감지 알고리즘 검증
- 회전 감지 및 각도 측정

### 제어 테스트

- 모터 제어 시스템 검증
- 각도 제어 알고리즘 테스트
- PID 제어 성능 평가

## 사용법

### 개별 테스트 실행

```bash
# AI 통신 테스트
python tests/run_tests.py --test ai

# 각도 제어 테스트
python tests/run_tests.py --test angle

# IMU 센서 테스트
python tests/run_tests.py --test imu

# MQTT 테스트
python tests/run_tests.py --test mqtt

# 장애물 감지 테스트
python tests/run_tests.py --test obstacle

# 회전 감지 테스트
python tests/run_tests.py --test rotation
```

### 모든 테스트 실행

```bash
python tests/run_tests.py --test all
```

### 직접 테스트 실행

```bash
# 개별 테스트 파일 직접 실행
python tests/ai_communication_test.py
python tests/angle_control_test.py
python tests/imu_sensor_test.py
python tests/mqtt_test.py
python tests/obstacle_detection_test.py
python tests/rotation_detection_test.py
```

## 테스트 환경 요구사항

### 하드웨어

- Jetson Orin 또는 호환 보드
- IMU 센서 (MPU6050/MPU9250/BNO08x)
- 모터 드라이버
- I2C 인터페이스

### 소프트웨어

- Python 3.8+
- ROS2 Humble
- MQTT 브로커 (Mosquitto)
- 필요한 Python 패키지들

## 테스트 결과

각 테스트는 다음 정보를 제공합니다:

- 테스트 성공/실패 여부
- 성능 메트릭 (응답 시간, 정확도 등)
- 오류 메시지 및 디버깅 정보
- 로그 파일 생성

## 문제 해결

### 일반적인 문제들

1. **I2C 연결 오류**: I2C 인터페이스 활성화 확인
2. **MQTT 연결 실패**: 브로커 서비스 상태 확인
3. **센서 데이터 오류**: 센서 연결 및 캘리브레이션 확인
4. **모터 제어 실패**: 모터 드라이버 연결 및 설정 확인

### 디버깅

- 각 테스트는 상세한 로그를 생성합니다
- `--verbose` 옵션으로 더 자세한 정보 확인 가능
- 실패한 테스트는 오류 원인을 명시합니다
