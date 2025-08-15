# 유틸리티 모듈

AMR 시스템의 유틸리티 및 설정 도구들을 담당하는 모듈들입니다.

## 파일 구조

### 핵심 유틸리티

- `logger.py`: 통일된 로그 시스템
- `__init__.py`: 패키지 초기화

### 시스템 설정 스크립트

- `setup_env.sh`: 환경 설정 스크립트
- `install_requirements.sh`: 의존성 설치 스크립트
- `enable_i2c_orin.sh`: I2C 활성화 스크립트 (Jetson Orin용)

### 빌드 및 실행 스크립트

- `build_imu_motor.sh`: IMU-모터 통합 빌드 스크립트
- `run_sensor_sync.sh`: 센서 동기화 실행 스크립트

## 주요 기능

### 로깅 시스템

- 통일된 로그 포맷 (텍스트 기반 구분)
- MQTT 송신/수신 성공 로그 및 콜백
- 다양한 로그 레벨 지원

### 시스템 설정

- 환경 변수 설정
- 의존성 패키지 설치
- I2C 인터페이스 활성화
- 빌드 환경 구성

### 실행 스크립트

- 센서 데이터 동기화 실행
- IMU-모터 통합 시스템 빌드

## 사용법

### 환경 설정

```bash
# 환경 설정
source utilities/setup_env.sh

# 의존성 설치
./utilities/install_requirements.sh

# I2C 활성화 (Jetson Orin)
./utilities/enable_i2c_orin.sh
```

### 빌드 및 실행

```bash
# IMU-모터 통합 빌드
./utilities/build_imu_motor.sh

# 센서 동기화 실행
./utilities/run_sensor_sync.sh
```

### 로거 사용

```python
from utilities.logger import mqtt_logger

# 기본 로깅
mqtt_logger.info("정보 메시지")      # [INFO] 정보 메시지
mqtt_logger.warn("경고 메시지")      # [WARN] 경고 메시지
mqtt_logger.error("오류 메시지")     # [ERROR] 오류 메시지

# MQTT 전용 로깅
mqtt_logger.mqtt_send_success("topic", {"data": "value"})
mqtt_logger.mqtt_receive_success("topic", {"data": "value"})
```

## 의존성

- Python 3.8+
- bash shell
- Jetson Orin (I2C 활성화 스크립트용)
