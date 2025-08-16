# 설정 모듈

AMR(Autonomous Mobile Robot) 프로젝트의 설정 관리 기능

## 개요

이 모듈은 AMR 시스템의 모든 구성 요소에 대한 설정을 중앙 집중식으로 관리합니다:

- **ConfigManager**: 모든 설정 파일의 로드, 저장, 검증 관리
- **YAML 설정 파일들**: 각 하드웨어 및 소프트웨어 구성 요소별 설정
- **설정 검증**: 설정 파일의 유효성 검사 및 오류 처리

## 설치

### 의존성

```bash
pip install -r requirements.txt
```

## 설정 파일

### robot_config.yaml

로봇 기본 정보 및 하드웨어 활성화 설정

- 로봇 이름, 타입, 프레임 정보
- 휠 분리 거리, 직경, 속도 제한
- 하드웨어 활성화 설정
- 안전 관련 설정

### motor_config.yaml

모터 및 서보 제어 설정

- 모터 핀 할당
- PWM 주파수, 속도 범위
- 가속/감속 비율
- 서보 각도 및 펄스 범위

### imu_config.yaml

IMU 센서 설정

- I2C 버스 및 주소
- 발행 주기, 프레임 ID
- 센서 융합 알고리즘 설정
- 캘리브레이션 파일 경로

### camera_config.yaml

카메라 설정

- 발행 주기, 디바이스 ID
- 해상도 설정
- 토픽 이름

### mqtt_config.yaml

MQTT 통신 설정

- 브로커 주소, 포트
- 클라이언트 ID, 인증 정보
- 토픽 이름 정의
- QoS 및 연결 설정

### ai_config.yaml

AI 모델 설정

- 모델 경로, 신뢰도 임계값
- 객체 감지 클래스 및 색상
- 경로 계획 알고리즘 설정

### nav2_params.yaml

ROS2 Navigation2 설정

- 플래너, 컨트롤러 설정
- 비헤이비어 트리 설정
- 글로벌/로컬 코스트맵 설정

### slam_params.yaml

SLAM 설정

- 최대 특징점 수
- 감지 임계값
- 토픽 이름

## 사용법

### 설정 매니저

```python
from config import ConfigManager

manager = ConfigManager()

# 모든 설정 로드
manager.load_all_configs()

# 특정 설정 가져오기
robot_config = manager.get_robot_config()
motor_config = manager.get_motor_config()

# 파라미터 가져오기
robot_name = manager.get_parameter("robot_config.yaml", "robot.ros__parameters.robot_name")
max_speed = manager.get_parameter("motor_config.yaml", "motor.ros__parameters.max_speed")

# 파라미터 설정
manager.set_parameter("robot_config.yaml", "robot.ros__parameters.robot_name", "AMR002")

# 설정 저장
manager.save_config("robot_config.yaml", robot_config)

# 설정 검증
is_valid = manager.validate_config("robot_config.yaml")
```

### 설정 파일 직접 사용

```python
import yaml

with open("config/robot_config.yaml", "r") as f:
    config = yaml.safe_load(f)

robot_name = config["robot"]["ros__parameters"]["robot_name"]
```

## 기능

### 설정 관리

- **자동 로드**: 모든 설정 파일 자동 로드
- **동적 저장**: 설정 변경 시 자동 저장
- **검증**: 설정 파일 구조 및 필수 필드 검증
- **백업**: 설정 변경 전 자동 백업

### 파라미터 접근

- **점 표기법**: `robot.ros__parameters.robot_name`
- **중첩 구조**: 깊은 중첩 구조 지원
- **타입 안전성**: 파라미터 타입 검증

### 설정 검증

- **필수 필드**: 필수 설정 필드 확인
- **타입 검증**: 파라미터 타입 검증
- **범위 검증**: 파라미터 값 범위 검증

## 환경 변수

```env
# 설정 디렉토리
CONFIG_DIR=config

# 로그 레벨
CONFIG_LOG_LEVEL=INFO
```

## 테스트

```bash
# 설정 매니저 테스트
python -m config.config_manager

# 설정 파일 검증
python -c "from config import ConfigManager; ConfigManager().validate_config('robot_config.yaml')"
```
