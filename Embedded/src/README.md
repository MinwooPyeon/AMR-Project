# SRC 폴더

## 폴더 구조

### 주요 파일들

- `main.cpp` - 메인 실행 파일
- `angle_controller.cpp` - 각도 제어 로직
- `imu_motor_integration.cpp` - IMU와 모터 통합 제어
- `imu_sensor.cpp` - IMU 센서 데이터 처리
- `sensor_data_sync.cpp` - 센서 데이터 동기화
- `backend_ws_client.cpp` - 백엔드 웹소켓 클라이언트
- `obstacle_detector.cpp` - 장애물 감지 시스템
- `led_driver.cpp` - LED 제어 드라이버

### 하위 디렉토리

#### `/ros/`

ROS2 관련 노드들

- `angle_control_node.cpp` - 각도 제어 노드
- `imu_data_publisher.cpp` - IMU 데이터 발행자
- `sensor_data_publisher.cpp` - 센서 데이터 발행자

#### `/module/`

모터 제어 모듈

- `motor_controller.cpp` - 모터 컨트롤러
- `motor_driver.cpp` - 모터 드라이버
- `frame.cpp` - 프레임 관련 코드

#### `/mqtt/`

MQTT 통신 모듈

- `device_data_builder.cpp` - 디바이스 데이터 빌더
- `image_publisher.cpp` - 이미지 발행자
- `mqtt_publisher.cpp` - MQTT 발행자
- `robot_data_publisher.cpp` - 로봇 데이터 발행자
- `state_publisher.cpp` - 상태 발행자

#### `/slam/`

SLAM 관련 코드

- `visual_lidar_slam_node.cpp` - 시각적 LiDAR SLAM 노드

#### `/i2c/`

I2C 통신 관련

- `i2c_address_manager.cpp` - I2C 주소 관리자
- `MyI2CImplementation.cpp` - I2C 구현체

#### `/sysfsInterface/`

시스템 파일 시스템 인터페이스

- `jetson_gpio.cpp` - Jetson GPIO 제어

#### `/ai/`

AI 관련 코드 (현재 비어있음)

## 주요 기능

### 1. 센서 통합

- IMU 센서 데이터 수집 및 처리
- LiDAR 데이터 처리
- 센서 데이터 동기화

### 2. 모터 제어

- 차동 구동 모터 제어
- PID 기반 각도 제어
- 다양한 이동 패턴 지원

### 3. 통신

- ROS2 노드 통신
- MQTT 메시징
- 웹소켓 클라이언트

### 4. 장애물 감지

- IMU 기반 충돌 감지
- 다중 센서 융합
- 실시간 장애물 분석

### 5. SLAM

- 시각적 SLAM
- LiDAR 기반 매핑
- 위치 추정

## 빌드 방법

```bash
# 프로젝트 루트에서
mkdir build && cd build
cmake ..
make -j4
```

## 실행 방법

```bash
# 메인 프로그램 실행
./amr_main

# ROS2 노드 실행
ros2 run amr_package angle_control_node
```

## 의존성

- ROS2 Humble
- OpenCV
- Paho MQTT
- libgpiod
- Eigen3
- JsonCpp
