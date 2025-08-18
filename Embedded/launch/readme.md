# Launch 폴더

ROS2 런치 파일과 AMR 시스템 실행 스크립트들이 포함된 폴더

## 폴더 구조

```
launch/
├── amr_robot_launch.py    # ROS2 런치 파일
├── main.py                # 모터 드라이버 테스트 스크립트
├── run_amr.py             # AMR 메인 실행 스크립트
└── readme.md              # 이 파일
```

## 주요 파일

### amr_robot_launch.py

ROS2 런치 파일로 다음 노드들을 실행합니다:

- **visual_lidar_slam_node**: SLAM 노드
- **webcam_publisher**: 웹캠 이미지 발행 노드
- **navigation_client**: 내비게이션 클라이언트 노드
- **sensor_data_publisher_node**: 센서 데이터 발행 노드
- **imu_data_publisher_node**: IMU 데이터 발행 노드
- **motor_cmd_subscriber_node**: 모터 명령 구독 노드
- **angle_control_node**: 각도 제어 노드
- **amr_main**: AMR 메인 노드
- **rosbridge_websocket**: 웹소켓 브리지 노드

### main.py

PCA9685 기반 모터 드라이버 테스트 스크립트입니다:

- 모터 초기화 및 설정
- 전진/후진 테스트
- 좌회전/우회전 테스트
- 모터 정지 기능

### run_amr.py

AMR 시스템 메인 실행 스크립트입니다:

- AMR 시스템 모듈 import
- 모터 상태 확인
- 시스템 초기화 및 실행

## 사용 방법

### ROS2 런치 실행

```bash
ros2 launch amr_project amr_robot_launch.py
```

### 모터 드라이버 테스트

```bash
python3 main.py
```

### AMR 시스템 실행

```bash
python3 run_amr.py
```

## 설정 파일

런치 파일에서 사용하는 설정 파일들:

- `config/slam_params.yaml`: SLAM 파라미터
- `config/camera_config.yaml`: 카메라 설정
- `config/nav2_params.yaml`: 내비게이션 파라미터

## 노드 파라미터

### IMU 데이터 발행 노드

- `frame_id`: 'imu_link'
- `publish_rate`: 50.0 Hz
- `enable_temperature`: True
- `imu_type`: 4 (BNO08X)
- `i2c_address`: 0x4B

### 각도 제어 노드

- `imu_type`: 'BNO08X'
- `imu_i2c_address`: 0x4B
- `motor_i2c_address`: 0x40
- `angle_tolerance`: 2.0도
- `turn_speed`: 50.0%
- `control_frequency`: 50.0 Hz
- PID 게인: Kp=2.0, Ki=0.1, Kd=0.5

## 의존성

- ROS2 Humble 이상
- Python 3.7+
- PCA9685 라이브러리
- AMR 프로젝트 패키지

## 주의사항

1. ROS2 환경이 소스되어 있어야 합니다
2. I2C 장치 권한이 설정되어 있어야 합니다
