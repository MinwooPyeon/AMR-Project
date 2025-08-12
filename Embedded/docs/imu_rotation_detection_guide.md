# IMU 센서 회전 감지 가이드

## 개요

이 가이드는 MPU6050/MPU9250 IMU 센서를 사용하여 **정확한 회전 각도를 감지**하고 **90도 회전을 확인**하는 방법을 설명합니다. 자이로스코프 데이터를 분석하여 회전 시작, 진행, 완료를 감지하고 목표 각도 도달을 확인합니다.

## 하드웨어 요구사항

### 센서
- **MPU6050 6축 IMU 센서** 또는 **MPU9250 9축 IMU 센서**
- 자이로스코프: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- 가속도계: ±2g, ±4g, ±8g, ±16g

### 연결
```
MPU6050/MPU9250 ----- Jetson/라즈베리파이
VCC    ----- 3.3V
GND    ----- GND
SDA    ----- I2C SDA (GPIO2)
SCL    ----- I2C SCL (GPIO3)
```

### I2C 주소
- **MPU6050**: 0x68 (ADO=0) 또는 0x69 (ADO=1)
- **MPU9250**: 0x68 (ADO=0) 또는 0x69 (ADO=1)

## 소프트웨어 구성

### 1. 회전 감지 클래스 (`RotationDetector`)

```cpp
#include "amr/rotation_detector.h"

// 회전 감지기 생성
auto i2c = std::make_shared<MyI2CImplementation>();
auto imu = std::make_shared<IMUSensor>(i2c, IMUType::MPU6050, 0x68, "IMU");

RotationConfig config;
config.target_angle = 90.0f;        // 목표 회전 각도 (도)
config.angle_tolerance = 2.0f;      // 각도 허용 오차 (도)
config.rotation_threshold = 5.0f;   // 회전 감지 임계값 (도/초)
config.stable_threshold = 1.0f;     // 안정 상태 임계값 (도/초)

auto detector = std::make_shared<RotationDetector>(imu, config);

// 초기화
if (detector->initialize()) {
    std::cout << "회전 감지기 초기화 성공" << std::endl;
}

// 회전 감지
auto result = detector->detectRotation();
if (result.rotation_detected) {
    if (result.target_angle_reached) {
        std::cout << "✅ 90도 회전 완료!" << std::endl;
        std::cout << "실제 회전: " << result.actual_angle << "도" << std::endl;
        std::cout << "정확도: " << result.accuracy * 100 << "%" << std::endl;
    } else {
        std::cout << "회전 중: " << result.actual_angle << "도" << std::endl;
    }
}
```

### 2. 회전 감지 알고리즘

#### 회전 상태 감지
```cpp
RotationStatus calculateRotationStatus(float yaw_rate) const {
    if (std::abs(yaw_rate) > rotation_threshold) {
        return RotationStatus::ROTATING;        // 회전 중
    } else if (std::abs(yaw_rate) < stable_threshold) {
        return RotationStatus::STABLE;          // 안정 상태
    } else {
        return RotationStatus::INVALID;         // 유효하지 않음
    }
}
```

#### 회전 방향 감지
```cpp
RotationDirection calculateRotationDirection(float yaw_change) const {
    if (yaw_change > 0) {
        return RotationDirection::CLOCKWISE;        // 시계방향
    } else if (yaw_change < 0) {
        return RotationDirection::COUNTERCLOCKWISE; // 반시계방향
    } else {
        return RotationDirection::NONE;
    }
}
```

#### 각도 계산
```cpp
float calculateAngleDifference(float angle1, float angle2) const {
    float diff = angle2 - angle1;
    
    // 최단 경로 계산 (-180 ~ 180)
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    
    return diff;
}

float calculateTotalRotation(float start_angle, float end_angle) const {
    return std::abs(calculateAngleDifference(start_angle, end_angle));
}
```

### 3. 목표 각도 도달 확인

```cpp
bool isTargetAngleReached(const RotationData& data) const {
    float angle_diff = std::abs(data.total_rotation - target_angle);
    return angle_diff <= angle_tolerance;
}

float calculateRotationAccuracy(const RotationData& data) const {
    if (data.total_rotation == 0.0f) return 0.0f;
    
    float angle_diff = std::abs(data.total_rotation - target_angle);
    float max_error = target_angle * 0.1f; // 10% 오차 허용
    
    float accuracy = 1.0f - (angle_diff / max_error);
    return std::clamp(accuracy, 0.0f, 1.0f);
}
```

## 설정 옵션

### RotationConfig

```cpp
struct RotationConfig {
    float target_angle = 90.0f;        // 목표 회전 각도 (도)
    float angle_tolerance = 2.0f;      // 각도 허용 오차 (도)
    float rotation_threshold = 5.0f;   // 회전 감지 임계값 (도/초)
    float stable_threshold = 1.0f;     // 안정 상태 임계값 (도/초)
    int window_size = 10;              // 이동 평균 윈도우 크기
    bool enable_smoothing = true;      // 스무딩 필터 활성화
    float smoothing_factor = 0.1f;     // 스무딩 계수
    int min_rotation_time = 500;       // 최소 회전 시간 (ms)
    int max_rotation_time = 5000;      // 최대 회전 시간 (ms)
};
```

### 회전 감지 임계값 설정

#### 민감한 감지 (빠른 회전)
```cpp
config.rotation_threshold = 3.0f;   // 3도/초 이상에서 회전 감지
config.stable_threshold = 0.5f;     // 0.5도/초 이하에서 안정 상태
config.angle_tolerance = 1.0f;      // 1도 오차 허용
```

#### 안정적인 감지 (천천히 회전)
```cpp
config.rotation_threshold = 10.0f;  // 10도/초 이상에서 회전 감지
config.stable_threshold = 2.0f;     // 2도/초 이하에서 안정 상태
config.angle_tolerance = 3.0f;      // 3도 오차 허용
```

## 테스트 방법

### 1. Python 테스트 스크립트 실행

```bash
# I2C 권한 설정
sudo chmod 666 /dev/i2c-1

# 테스트 실행
python3 tests/rotation_detection_test.py
```

### 2. 테스트 시나리오

#### 기본 회전 감지 테스트
```python
# 90도 회전을 수행하고 감지 확인
test.continuous_monitoring(30)  # 30초 동안 모니터링
```

#### 정확도 테스트
```python
# 여러 번 90도 회전을 수행하여 정확도 측정
results = test.test_rotation_accuracy()
# 결과: 평균 회전 각도, 표준편차, 정확도
```

#### 캘리브레이션 테스트
```python
# 요 각도 캘리브레이션
if test.calibrate_yaw():
    print("캘리브레이션 성공")
else:
    print("캘리브레이션 실패")
```

### 3. 예상 결과

#### 정상 상태
```
현재 요: 0.0도, 각속도: 0.2도/초
상태: 안정
```

#### 회전 시작
```
회전 시작: 0.0도, 각속도: 15.3도/초
방향: 시계방향
```

#### 회전 완료
```
회전 종료: 89.8도
총 회전 각도: 89.8도
회전 시간: 2340ms
✅ 목표 각도(90도) 도달 성공!
실제 회전: 89.8도
```

## 정확도 및 성능

### 1. 정확도 요인

#### 센서 정확도
- **자이로스코프 드리프트**: 시간에 따른 누적 오차
- **온도 영향**: 온도 변화에 따른 센서 특성 변화
- **진동 노이즈**: 외부 진동에 의한 노이즈

#### 알고리즘 정확도
- **적분 오차**: 자이로 적분 시 누적 오차
- **필터링**: 노이즈 제거와 반응성의 트레이드오프
- **임계값 설정**: 민감도와 안정성의 균형

### 2. 성능 최적화

#### 필터링 설정
```cpp
// 노이즈 제거 강화
detector->enableSmoothing(true);
detector->setSmoothingFactor(0.05f);  // 강한 스무딩

// 반응성 강화
detector->setSmoothingFactor(0.3f);   // 약한 스무딩
```

#### 임계값 조정
```cpp
// 빠른 회전 감지
config.rotation_threshold = 3.0f;
config.stable_threshold = 0.5f;

// 느린 회전 감지
config.rotation_threshold = 15.0f;
config.stable_threshold = 3.0f;
```

### 3. 정확도 개선 방법

#### 칼만 필터 적용
```cpp
// 자이로 적분 대신 칼만 필터 사용
// 가속도계와 자이로스코프 데이터 융합
```

#### 온도 보정
```cpp
// 온도 변화에 따른 센서 특성 보정
float temperature_compensation = calculateTemperatureCompensation(temp);
```

#### 다중 센서 융합
```cpp
// 여러 IMU 센서 데이터 융합
// 평균 및 표준편차 기반 신뢰도 계산
```

## 문제 해결

### 1. 일반적인 문제

#### 회전이 감지되지 않음
- **원인**: 임계값이 너무 높음
- **해결**: `rotation_threshold` 낮추기
```cpp
config.rotation_threshold = 2.0f;  // 더 민감하게 설정
```

#### 잘못된 각도 측정
- **원인**: 캘리브레이션 부족
- **해결**: 정확한 캘리브레이션 수행
```cpp
detector->calibrateYaw();  // 캘리브레이션 실행
```

#### 노이즈가 많음
- **원인**: 필터링 부족
- **해결**: 스무딩 강화
```cpp
detector->enableSmoothing(true);
detector->setSmoothingFactor(0.1f);
```

### 2. 디버깅 방법

#### 데이터 모니터링
```cpp
// 실시간 데이터 출력
std::cout << "요: " << detector->getCurrentYaw() << "도" << std::endl;
std::cout << "각속도: " << detector->getYawRate() << "도/초" << std::endl;
std::cout << "총 회전: " << detector->getTotalRotation() << "도" << std::endl;
```

#### 히스토리 확인
```cpp
// 데이터 히스토리 크기 확인
size_t history_size = detector->getHistorySize();
std::cout << "히스토리 크기: " << history_size << std::endl;
```

## 확장 기능

### 1. 다양한 각도 감지
```cpp
// 45도 회전 감지
config.target_angle = 45.0f;

// 180도 회전 감지
config.target_angle = 180.0f;

// 360도 회전 감지
config.target_angle = 360.0f;
```

### 2. 다축 회전 감지
```cpp
// 롤, 피치, 요 모든 축 회전 감지
// 3D 회전 매트릭스 계산
```

### 3. 연속 회전 감지
```cpp
// 여러 번의 회전을 연속으로 감지
// 회전 패턴 분석
```

## 결론

IMU 센서를 이용한 회전 감지는 **정확한 각도 측정**과 **목표 각도 도달 확인**이 가능합니다. 적절한 설정과 캘리브레이션을 통해 **90도 회전을 정확히 감지**할 수 있으며, 필터링과 알고리즘 최적화를 통해 성능을 향상시킬 수 있습니다.

### 주요 특징
- **정확한 각도 측정**: ±2도 이내 정확도
- **실시간 감지**: 50Hz 업데이트 주기
- **방향 감지**: 시계방향/반시계방향 구분
- **정확도 계산**: 회전 정확도 자동 계산
- **캘리브레이션**: 자동 요 각도 캘리브레이션

### 권장 사용법
1. **초기 캘리브레이션** 수행
2. **환경에 맞는 임계값** 설정
3. **적절한 필터링** 적용
4. **정기적인 재캘리브레이션** 수행 