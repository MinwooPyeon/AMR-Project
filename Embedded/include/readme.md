# Include 폴더

C++ 헤더 파일들이 포함된 폴더

## 폴더 구조

```
include/
└── amr/
    ├── ai/                    # AI 관련 헤더
    ├── battery/               # 배터리 모니터링 헤더
    ├── camera/                # 카메라 관련 헤더
    ├── i2c/                   # I2C 통신 헤더
    ├── module/                # 모터 제어 모듈 헤더
    ├── mqtt/                  # MQTT 통신 헤더
    ├── navigation/            # 내비게이션 헤더
    ├── ros/                   # ROS2 통합 헤더
    ├── slam/                  # SLAM 관련 헤더
    ├── sysfsInterface/        # GPIO 인터페이스 헤더
    ├── angle_controller.h     # 각도 제어 헤더
    ├── backend_ws_client.h    # 백엔드 웹소켓 클라이언트 헤더
    ├── imu_motor_integration.h # IMU 모터 통합 헤더
    ├── imu_sensor.h           # IMU 센서 헤더
    ├── led_driver.h           # LED 드라이버 헤더
    ├── level_controller.h     # 레벨 제어 헤더
    ├── lidar_sensor.h         # 라이다 센서 헤더
    └── obstacle_detector.h    # 장애물 감지 헤더
```

## 주요 헤더 파일

### 센서 관련

- **imu_sensor.h**: IMU 센서 제어 및 데이터 처리
- **lidar_sensor.h**: 라이다 센서 제어
- **camera/webcam_publisher.h**: 웹캠 이미지 발행

### 제어 관련

- **angle_controller.h**: 각도 제어 시스템
- **level_controller.h**: 레벨 제어 시스템
- **module/motor_controller.h**: 모터 제어
- **module/motor_driver.h**: 모터 드라이버

### 통신 관련

- **mqtt/**: MQTT 통신 관련 헤더들
- **backend_ws_client.h**: 백엔드 웹소켓 클라이언트
- **i2c/**: I2C 통신 인터페이스

### ROS2 통합

- **ros/**: ROS2 노드 및 토픽 관련 헤더들

### 시스템 관련

- **battery/battery_publisher.h**: 배터리 상태 발행
- **sysfsInterface/jetson_gpio.h**: Jetson GPIO 인터페이스
- **obstacle_detector.h**: 장애물 감지 시스템

## 사용 방법

### 기본 포함 방법

```cpp
#include "amr/imu_sensor.h"
#include "amr/angle_controller.h"
#include "amr/module/motor_controller.h"
```

### 네임스페이스 사용

```cpp
using namespace amr;

auto imu = std::make_shared<IMUSensor>(i2c_interface);
auto controller = std::make_shared<AngleController>(motor_controller, imu);
```

## 컴파일 설정

### CMakeLists.txt 예시

```cmake
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
```

### 컴파일러 플래그

```bash
g++ -I./include -std=c++17 your_file.cpp
```

## 의존성

- C++17 이상
- ROS2 (ros/ 폴더 사용 시)
- I2C 라이브러리
- MQTT 라이브러리
- OpenCV (카메라 관련)

## 주의사항

1. 모든 헤더 파일은 `#pragma once` 또는 `#ifndef` 가드 사용
2. 스레드 안전성을 위해 적절한 뮤텍스 사용
3. 메모리 관리를 위해 스마트 포인터 사용
4. 예외 처리를 위한 적절한 에러 핸들링 구현
