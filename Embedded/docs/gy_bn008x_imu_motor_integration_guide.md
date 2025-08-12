# GY-BN008x IMU-모터 통합 시스템 가이드

## 개요

이 문서는 GY-BN008x IMU 센서를 사용하여 로봇의 우회전/좌회전을 정확히 90도로 제어하는 시스템에 대한 가이드입니다.

## 시스템 구성

### 하드웨어 요구사항

1. **GY-BN008x IMU 센서** (MPU6050 기반)
   - 6축 IMU (가속도계 + 자이로스코프)
   - I2C 통신 (주소: 0x68)
   - 전원: 3.3V

2. **모터 드라이버**
   - Waveshare Motor Driver HAT
   - I2C 통신 (주소: 0x60)
   - DC 모터 2개

3. **연결 방법**
   ```
   GY-BN008x IMU:
   - VCC → 3.3V
   - GND → GND
   - SCL → I2C SCL
   - SDA → I2C SDA
   
   Motor Driver HAT:
   - VCC → 5V
   - GND → GND
   - SCL → I2C SCL
   - SDA → I2C SDA
   ```

## 소프트웨어 구성

### 주요 클래스

1. **IMUMotorIntegration**: IMU와 모터를 통합 제어하는 메인 클래스
2. **IMUSensor**: GY-BN008x IMU 센서 제어
3. **MotorController**: 모터 제어
4. **MotorDriver**: 모터 드라이버 하드웨어 제어

### 핵심 기능

- **정확한 90도 회전**: PID 제어를 통한 정밀한 각도 제어
- **실시간 각도 모니터링**: 100Hz 샘플링으로 실시간 각도 추적
- **자동 캘리브레이션**: 초기 각도 자동 캘리브레이션
- **안전 제어**: 각도 허용 오차 내에서 정확한 제어

## 사용 방법

### 1. 빌드 및 설치

```bash
# 프로젝트 빌드
mkdir build && cd build
cmake ..
make -j4

# 실행 파일 설치
sudo make install
```

### 2. 실행

```bash
# IMU-모터 통합 테스트 실행
./imu_motor_test
```

### 3. API 사용법

```cpp
#include "amr/imu_motor_integration.h"

// 시스템 초기화
amr::IMUMotorIntegration imuMotorSystem("/dev/i2c-1");
if (!imuMotorSystem.initialize()) {
    std::cerr << "초기화 실패!" << std::endl;
    return -1;
}

// 제어 루프 시작
imuMotorSystem.start();

// 좌회전 90도
imuMotorSystem.turnLeft90();

// 우회전 90도
imuMotorSystem.turnRight90();

// 회전 정지
imuMotorSystem.stopTurning();

// 상태 확인
imuMotorSystem.printStatus();

// 시스템 정지
imuMotorSystem.stop();
```

## 제어 파라미터

### PID 제어 게인

```cpp
// 기본값
double kp = 2.0;  // 비례 게인
double ki = 0.1;  // 적분 게인
double kd = 0.5;  // 미분 게인

// 설정 방법
imuMotorSystem.setPIDGains(kp, ki, kd);
```

### 제어 파라미터

- **각도 허용 오차**: 2.0도
- **최대 속도**: 50%
- **최소 속도**: 10%
- **제어 주파수**: 100Hz
- **IMU 샘플링**: 100Hz

## 테스트 시나리오

### 1. 기본 회전 테스트

```cpp
// 좌회전 90도 테스트
imuMotorSystem.turnLeft90();

// 회전 완료 대기
while (imuMotorSystem.isTurning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// 우회전 90도 테스트
imuMotorSystem.turnRight90();
```

### 2. 연속 회전 테스트

```cpp
// 4방향 회전 테스트
for (int i = 0; i < 4; i++) {
    if (i % 2 == 0) {
        imuMotorSystem.turnLeft90();
    } else {
        imuMotorSystem.turnRight90();
    }
    
    // 회전 완료 대기
    while (imuMotorSystem.isTurning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 2초 대기
    std::this_thread::sleep_for(std::chrono::seconds(2));
}
```

### 3. 상태 모니터링

```cpp
// 실시간 상태 출력
while (true) {
    imuMotorSystem.printStatus();
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
```

## 문제 해결

### 1. IMU 초기화 실패

**증상**: "IMU 센서 초기화 실패" 메시지

**해결 방법**:
- I2C 연결 확인: `i2cdetect -y 1`
- 전원 공급 확인 (3.3V)
- I2C 주소 확인 (0x68)
- 센서 물리적 연결 확인

### 2. 모터 드라이버 초기화 실패

**증상**: "모터 드라이버 초기화 실패" 메시지

**해결 방법**:
- I2C 연결 확인: `i2cdetect -y 1`
- 전원 공급 확인 (5V)
- I2C 주소 확인 (0x60)
- 모터 연결 확인

### 3. 각도 제어 부정확

**증상**: 90도 회전이 부정확함

**해결 방법**:
- PID 게인 조정
- 초기 캘리브레이션 재실행
- 센서 노이즈 확인
- 제어 주파수 조정

### 4. 회전 속도 문제

**증상**: 회전이 너무 빠르거나 느림

**해결 방법**:
- 최대/최소 속도 조정
- PID 게인 조정
- 모터 전원 공급 확인

## 성능 최적화

### 1. PID 게인 튜닝

```cpp
// 빠른 응답을 위한 설정
imuMotorSystem.setPIDGains(3.0, 0.05, 0.8);

// 안정적인 제어를 위한 설정
imuMotorSystem.setPIDGains(1.5, 0.15, 0.3);
```

### 2. 제어 주파수 조정

- 높은 주파수: 빠른 응답, 높은 CPU 사용률
- 낮은 주파수: 느린 응답, 낮은 CPU 사용률

### 3. 필터링 설정

```cpp
// 노이즈 감소를 위한 설정
imuSensor_->enableLowPassFilter(true);
imuSensor_->setFilterCutoff(3.0f);  // 3Hz
```

## 로그 및 디버깅

### 1. 로그 레벨

- **INFO**: 일반적인 동작 정보
- **DEBUG**: 상세한 제어 정보
- **ERROR**: 오류 정보

### 2. 디버그 출력

```cpp
// 상세한 상태 정보 출력
imuMotorSystem.printStatus();

// 실시간 각도 모니터링
std::cout << "현재 각도: " << imuMotorSystem.getCurrentAngle() << "도" << std::endl;
std::cout << "목표 각도: " << imuMotorSystem.getTargetAngle() << "도" << std::endl;
```

## 안전 주의사항

1. **전원 공급**: 안정적인 전원 공급 필수
2. **물리적 안전**: 회전 중 안전 거리 확보
3. **비상 정지**: Ctrl+C로 안전한 종료
4. **온도 관리**: IMU 센서 과열 방지
5. **진동 방지**: 센서 진동 최소화

## 참고 자료

- [GY-BN008x 데이터시트](https://www.gyroscope.com/datasheet/GY-BN008x.pdf)
- [MPU6050 레지스터 맵](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
- [PID 제어 이론](https://en.wikipedia.org/wiki/PID_controller)
- [I2C 통신 프로토콜](https://en.wikipedia.org/wiki/I%C2%B2C)
