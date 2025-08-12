# IMU 센서 통합 가이드

## 개요

이 프로젝트에 IMU 센서를 통합하는 방법을 설명합니다. 현재 지원하는 센서는 다음과 같습니다:

- **MPU6050**: 6축 IMU (가속도계 + 자이로스코프)
- **MPU9250**: 9축 IMU (가속도계 + 자이로스코프 + 자력계)
- **BNO055**: 9축 IMU with 센서퓨전 (구현 예정)
- **LSM9DS1**: 9축 IMU (구현 예정)

## 하드웨어 연결

### MPU6050/MPU9250 연결
```
VCC  -> 3.3V
GND  -> GND
SCL  -> I2C SCL (보통 GPIO 2)
SDA  -> I2C SDA (보통 GPIO 3)
```

### I2C 주소 확인
```bash
# I2C 장치 스캔
sudo i2cdetect -y 1
```

## 소프트웨어 설정

### 1. 의존성 설치
```bash
# I2C 도구 설치
sudo apt-get install i2c-tools

# Python 의존성 (테스트용)
pip install smbus2
```

### 2. I2C 활성화
```bash
# /boot/config.txt에 추가
sudo nano /boot/config.txt

# 다음 라인 추가/해제
dtparam=i2c_arm=on
```

### 3. 권한 설정
```bash
# i2c 그룹에 사용자 추가
sudo usermod -a -G i2c $USER
sudo chmod a+rw /dev/i2c-1
```

## 사용 방법

### 1. 기본 테스트
```bash
# Python 테스트 스크립트 실행
cd tests
python3 imu_sensor_test.py
```

### 2. C++ 빌드 및 실행
```bash
# 빌드
mkdir build && cd build
cmake ..
make

# 실행
./amr_main
```

### 3. ROS2 실행
```bash
# Launch 파일 실행
ros2 launch amr_project amr_robot_launch.py
```

## 데이터 확인

### ROS2 토픽 확인
```bash
# IMU 데이터 확인
ros2 topic echo /imu/data

# 온도 데이터 확인
ros2 topic echo /imu/temperature

# 토픽 목록 확인
ros2 topic list
```

### RViz에서 시각화
```bash
# RViz 실행
rviz2

# IMU 데이터 시각화:
# 1. Add -> By topic -> /imu/data -> Imu
# 2. Fixed Frame을 'imu_link'로 설정
```

## 코드 구조

### 주요 클래스
- `IMUSensor`: IMU 센서 제어 클래스
- `IMUDataPublisher`: ROS2 퍼블리셔
- `IMUTest`: Python 테스트 클래스

### 데이터 구조
```cpp
struct IMUData {
    float gyro_x, gyro_y, gyro_z;      // 각속도 (도/초)
    float accel_x, accel_y, accel_z;    // 가속도 (m/s²)
    float mag_x, mag_y, mag_z;          // 자력계 (μT)
    float roll, pitch, yaw;             // 오일러 각도 (도)
    float quaternion_w, x, y, z;        // 쿼터니언
    float temperature;                   // 온도 (섭씨)
    uint64_t timestamp;                 // 타임스탬프
};
```

## 설정 옵션

### ROS2 파라미터
```yaml
# launch 파일에서 설정 가능
frame_id: 'imu_link'           # TF 프레임 ID
publish_rate: 50.0             # 퍼블리시 주파수 (Hz)
enable_temperature: true        # 온도 데이터 활성화
imu_type: 0                    # 센서 타입 (0: MPU6050)
i2c_address: 0x68             # I2C 주소
```

### 센서 설정
```cpp
// 자이로스코프 범위 설정
imu->setGyroRange(250);    // ±250°/s
imu->setGyroRange(500);    // ±500°/s
imu->setGyroRange(1000);   // ±1000°/s
imu->setGyroRange(2000);   // ±2000°/s

// 가속도계 범위 설정
imu->setAccelRange(2);     // ±2g
imu->setAccelRange(4);     // ±4g
imu->setAccelRange(8);     // ±8g
imu->setAccelRange(16);    // ±16g

// 샘플레이트 설정
imu->setSampleRate(1000);  // 1000Hz
```

## 캘리브레이션

### 자동 캘리브레이션
```cpp
// 캘리브레이션 시작
imu->startCalibration();

// 진행률 확인
float progress = imu->getCalibrationProgress();

// 캘리브레이션 중지
imu->stopCalibration();
```

### 수동 캘리브레이션
1. 센서를 평평한 곳에 놓기
2. 움직이지 않고 5초 대기
3. 다양한 방향으로 회전
4. 캘리브레이션 완료 확인

## 필터링

### 저역 통과 필터
```cpp
// 필터 활성화
imu->enableLowPassFilter(true);

// 차단 주파수 설정
imu->setFilterCutoff(5.0f);  // 5Hz
```

## 문제 해결

### 일반적인 문제

1. **I2C 연결 실패**
   ```bash
   # I2C 활성화 확인
   ls /dev/i2c*
   
   # 권한 확인
   ls -l /dev/i2c-1
   ```

2. **센서 응답 없음**
   ```bash
   # I2C 주소 스캔
   sudo i2cdetect -y 1
   
   # 연결 확인
   sudo i2cget -y 1 0x68 0x75
   ```

3. **데이터 노이즈**
   - 센서를 진동이 적은 곳에 설치
   - 저역 통과 필터 활성화
   - 캘리브레이션 수행

### 디버깅

```bash
# ROS2 로그 확인
ros2 run amr_project amr_main --ros-args --log-level debug

# IMU 데이터 모니터링
ros2 topic echo /imu/data --once
```

## 성능 최적화

### 1. 샘플레이트 조정
- 높은 정밀도 필요: 1000Hz
- 일반적인 사용: 100Hz
- 배터리 절약: 10Hz

### 2. 필터 설정
- 정적 환경: 낮은 차단 주파수 (1-5Hz)
- 동적 환경: 높은 차단 주파수 (10-20Hz)

### 3. 메모리 사용량
- 버퍼 크기 조정
- 불필요한 데이터 필터링

## 확장 가능성

### 추가 센서 지원
1. BNO055 구현
2. LSM9DS1 구현
3. 커스텀 센서 추가

### 고급 기능
1. Madgwick/Mahony 필터
2. 센서퓨전 알고리즘
3. 자동 캘리브레이션
4. 온도 보정

## 참고 자료

- [MPU6050 데이터시트](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU9250 데이터시트](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [ROS2 IMU 메시지](https://docs.ros.org/en/humble/p/sensor_msgs/html/msg/Imu.html) 