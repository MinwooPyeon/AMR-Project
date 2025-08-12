# IMU 기반 장애물 감지 가이드

## 개요

이 가이드는 MPU9250 9축 IMU 센서를 사용하여 장애물을 감지하고 거리를 측정하는 방법을 설명합니다. 가속도계, 자이로스코프, 자력계 데이터를 분석하여 이상 패턴을 감지하고 거리를 추정합니다.

## 하드웨어 요구사항

### 센서
- **MPU9250 9축 IMU 센서**
  - 가속도계: ±2g, ±4g, ±8g, ±16g
  - 자이로스코프: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
  - 자력계: ±4900 μT (16비트 해상도)

### 연결
```
MPU9250 ----- Jetson/라즈베리파이
VCC    ----- 3.3V
GND    ----- GND
SDA    ----- I2C SDA (GPIO2)
SCL    ----- I2C SCL (GPIO3)
```

### I2C 주소
- **MPU9250**: 0x68 (ADO=0) 또는 0x69 (ADO=1)
- **AK8963 자력계**: 0x0C

## 소프트웨어 구성

### 1. IMU 센서 클래스 (`IMUSensor`)

```cpp
#include "amr/imu_sensor.h"

// IMU 센서 생성
auto i2c = std::make_shared<MyI2CImplementation>();
auto imu = std::make_shared<IMUSensor>(i2c, IMUType::MPU9250, 0x68, "IMU");

// 초기화
if (imu->initialize()) {
    std::cout << "IMU 센서 초기화 성공" << std::endl;
}

// 데이터 읽기
if (imu->readData()) {
    auto data = imu->getLatestData();
    std::cout << "가속도: " << data.accel_x << ", " << data.accel_y << ", " << data.accel_z << std::endl;
}
```

### 2. 장애물 감지 클래스 (`ObstacleDetector`)

```cpp
#include "amr/obstacle_detector.h"

// 장애물 감지기 생성
ObstacleDetectionConfig config;
config.acceleration_threshold = 2.0f;  // m/s²
config.gyro_threshold = 50.0f;        // deg/s
config.magnetic_threshold = 100.0f;    // μT
config.window_size = 10;
config.detection_confidence = 0.7f;

auto detector = std::make_shared<ObstacleDetector>(imu, config);

// 초기화
if (detector->initialize()) {
    std::cout << "장애물 감지기 초기화 성공" << std::endl;
}

// 장애물 감지
auto result = detector->detectObstacle();
if (result.obstacle_detected) {
    std::cout << "장애물 감지! 거리: " << result.distance_estimate << "m" << std::endl;
    std::cout << "신뢰도: " << result.confidence << std::endl;
    std::cout << "충격력: " << result.impact_force << std::endl;
}
```

## 장애물 감지 알고리즘

### 1. 이상 감지 방법

#### 가속도 기반 감지
- **원리**: 갑작스러운 가속도 변화 감지
- **임계값**: 2.0 m/s² (설정 가능)
- **가중치**: 40% (신뢰도 계산)

#### 자이로스코프 기반 감지
- **원리**: 갑작스러운 각속도 변화 감지
- **임계값**: 50 deg/s (설정 가능)
- **가중치**: 30% (신뢰도 계산)

#### 자력계 기반 감지
- **원리**: 자기장 변화 감지 (금속 장애물)
- **임계값**: 100 μT (설정 가능)
- **가중치**: 30% (신뢰도 계산)

### 2. 거리 추정 방법

#### 가속도 기반 거리 추정
```cpp
float estimateDistanceFromAcceleration() {
    // 가속도 적분을 통한 거리 추정
    float accel_magnitude = sqrt(ax² + ay² + az²);
    float dt = timestamp_diff / 1000.0f;
    return accel_magnitude * dt² * 0.5f;
}
```

#### 자기장 기반 거리 추정
```cpp
float estimateDistanceFromMagneticField() {
    // 자기장 변화를 거리로 변환
    float mag_change = sqrt(Δmx² + Δmy² + Δmz²);
    return mag_change / 1000.0f;  // μT를 미터로 변환
}
```

#### 모션 기반 거리 추정
```cpp
float estimateDistanceFromMotion() {
    // 각속도를 거리로 변환
    float motion_magnitude = sqrt(gx² + gy² + gz²);
    return motion_magnitude / 100.0f;
}
```

### 3. 최종 거리 계산
```cpp
float final_distance = 
    distance_accel * 0.5f +    // 가속도 기반 (50%)
    distance_mag * 0.3f +      // 자기장 기반 (30%)
    distance_motion * 0.2f;    // 모션 기반 (20%)
```

## 설정 옵션

### ObstacleDetectionConfig

```cpp
struct ObstacleDetectionConfig {
    float acceleration_threshold = 2.0f;    // 가속도 임계값 (m/s²)
    float gyro_threshold = 50.0f;          // 자이로 임계값 (deg/s)
    float magnetic_threshold = 100.0f;      // 자기장 임계값 (μT)
    int window_size = 10;                   // 이동 평균 윈도우 크기
    float detection_confidence = 0.7f;      // 감지 신뢰도 임계값
    bool enable_magnetic_detection = true;  // 자기장 기반 감지 활성화
    bool enable_motion_detection = true;    // 모션 기반 감지 활성화
};
```

### 필터링 설정
```cpp
detector->enableFiltering(true);           // 저역 통과 필터 활성화
detector->setFilterStrength(0.1f);        // 필터 강도 설정 (0.0 ~ 1.0)
```

## 테스트 방법

### 1. Python 테스트 스크립트 실행

```bash
# I2C 권한 설정
sudo chmod 666 /dev/i2c-1

# 테스트 실행
python3 tests/obstacle_detection_test.py
```

### 2. 테스트 시나리오

#### 캘리브레이션 테스트
- 센서를 안정적으로 유지
- 100개 샘플 수집
- 평균 및 표준편차 계산

#### 모션 테스트
- 센서를 다양한 방향으로 움직임
- 가속도, 자이로, 자기장 크기 모니터링

#### 연속 모니터링
- 실시간 장애물 감지
- 임계값 초과 시 알림
- 거리 및 신뢰도 표시

### 3. 예상 결과

#### 정상 상태
```
가속도 크기: ~9.81 m/s² (중력)
자이로 크기: ~0-5 deg/s (미세 진동)
자기장 크기: ~25-65 μT (지구 자기장)
```

#### 장애물 감지 시
```
가속도 크기: >11 m/s² (임계값 초과)
자이로 크기: >50 deg/s (임계값 초과)
자기장 크기: >125 μT (임계값 초과)
```

## 성능 최적화

### 1. 샘플링 레이트 조정
```cpp
// 높은 정확도 (낮은 샘플링 레이트)
imu->setSampleRate(100);  // 100Hz

// 빠른 응답 (높은 샘플링 레이트)
imu->setSampleRate(1000); // 1000Hz
```

### 2. 임계값 조정
```cpp
// 민감한 감지
config.acceleration_threshold = 1.0f;  // 낮은 임계값
config.gyro_threshold = 25.0f;

// 안정적인 감지
config.acceleration_threshold = 3.0f;  // 높은 임계값
config.gyro_threshold = 75.0f;
```

### 3. 윈도우 크기 조정
```cpp
// 빠른 응답
config.window_size = 5;   // 작은 윈도우

// 안정적인 감지
config.window_size = 20;  // 큰 윈도우
```

## 문제 해결

### 1. I2C 연결 문제
```bash
# I2C 버스 확인
i2cdetect -y 1

# 권한 문제 해결
sudo usermod -a -G i2c $USER
sudo chmod 666 /dev/i2c-1
```

### 2. 센서 초기화 실패
- 전원 공급 확인 (3.3V)
- I2C 주소 확인 (0x68 또는 0x69)
- 배선 연결 확인

### 3. 부정확한 데이터
- 센서 캘리브레이션 수행
- 필터링 활성화
- 임계값 조정

### 4. 거리 추정 오류
- 알고리즘 파라미터 조정
- 여러 센서 데이터 융합
- 환경 보정 수행

## 확장 가능성

### 1. 추가 센서 지원
- BNO055 (Bosch)
- LSM9DS1 (STMicroelectronics)
- ICM20948 (TDK InvenSense)

### 2. 고급 알고리즘
- 칼만 필터 적용
- 머신러닝 기반 감지
- 딥러닝 모델 통합

### 3. ROS2 통합
```cpp
// ROS2 노드로 발행
auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
auto obstacle_pub = node->create_publisher<std_msgs::msg::Bool>("/obstacle_detected", 10);
```

## 참고 자료

- [MPU9250 데이터시트](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [AK8963 데이터시트](https://www.akm.com/akm/en/file/datasheet/AK8963.pdf)
- [I2C 통신 가이드](https://www.i2c-bus.org/)
- [센서 융합 알고리즘](https://en.wikipedia.org/wiki/Sensor_fusion)

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 