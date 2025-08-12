# IMU 기반 각도 제어 시스템 가이드

## 개요

이 문서는 IMU 센서를 사용하여 로봇의 정확한 각도 제어를 수행하는 시스템에 대한 가이드입니다. 이 시스템은 초기 0도 유지와 좌우회전 시 90도 회전을 정확히 제어할 수 있습니다.

## 시스템 구성

### 주요 컴포넌트

1. **IMUSensor** - IMU 센서 데이터 읽기 및 처리
2. **RotationDetector** - 회전 감지 및 각도 계산
3. **AngleController** - 각도 제어 로직 및 PID 제어
4. **MotorController** - 모터 제어 인터페이스
5. **AngleControlNode** - ROS2 노드 인터페이스

### 지원 센서

- **MPU6050** (6축) - 가속도계, 자이로스코프
- **MPU9250** (9축) - 가속도계, 자이로스코프, 자기계

## 설치 및 설정

### 1. 하드웨어 연결

```
IMU 센서 (MPU9250/MPU6050)
├── VCC → 3.3V
├── GND → GND
├── SCL → I2C SCL (보드에 따라 다름)
└── SDA → I2C SDA (보드에 따라 다름)

모터 드라이버
├── VCC → 5V
├── GND → GND
├── SCL → I2C SCL
└── SDA → I2C SDA
```

### 2. 소프트웨어 빌드

```bash
# 프로젝트 디렉토리로 이동
cd /path/to/amr_project

# 빌드 디렉토리 생성
mkdir build && cd build

# CMake 구성
cmake ..

# 빌드
make -j4
```

### 3. ROS2 환경 설정

```bash
# ROS2 환경 소싱
source /opt/ros/humble/setup.bash

# 워크스페이스 빌드
colcon build

# 환경 소싱
source install/setup.bash
```

## 사용법

### 1. 기본 각도 제어 테스트

```bash
# Python 테스트 스크립트 실행
cd tests
python3 angle_control_test.py
```

테스트 옵션:
- **대화형 테스트**: 실시간 각도 제어 명령 입력
- **90도 회전 테스트**: 자동 90도 좌우회전 테스트

### 2. ROS2 노드 실행

```bash
# 각도 제어 노드 실행
ros2 launch amr_project amr_robot_launch.py
```

### 3. 각도 제어 명령

#### 토픽을 통한 제어

```bash
# 각도 제어 활성화
ros2 topic pub /angle_control_enable std_msgs/msg/Bool "data: true"

# 목표 각도 설정 (90도)
ros2 topic pub /target_angle std_msgs/msg/Float64 "data: 90.0"

# 문자열 명령
ros2 topic pub /angle_control_command std_msgs/msg/String "data: 'turn_left 90'"
ros2 topic pub /angle_control_command std_msgs/msg/String "data: 'turn_right 90'"
ros2 topic pub /angle_control_command std_msgs/msg/String "data: 'maintain 0'"
```

#### 서비스를 통한 제어

```bash
# 각도 제어 활성화/비활성화
ros2 service call /angle_control_enable std_srvs/srv/SetBool "data: true"

# 캘리브레이션
ros2 service call /angle_control_calibrate std_srvs/srv/Trigger

# 리셋
ros2 service call /angle_control_reset std_srvs/srv/Trigger
```

#### cmd_vel을 통한 제어

```bash
# 좌회전 (angular.z > 0)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# 우회전 (angular.z < 0)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"

# 직진 (linear.x > 0)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 4. 모니터링

```bash
# 현재 각도 모니터링
ros2 topic echo /current_angle

# 목표 각도 모니터링
ros2 topic echo /target_angle_pub

# 각도 오차 모니터링
ros2 topic echo /angle_error

# 상태 모니터링
ros2 topic echo /angle_control_status

# IMU 데이터 모니터링
ros2 topic echo /imu/data
```

## 설정 파라미터

### 각도 제어 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `angle_tolerance` | 2.0 | 각도 허용 오차 (도) |
| `turn_speed` | 50.0 | 회전 속도 (0-100) |
| `maintain_speed` | 30.0 | 직진 속도 (0-100) |
| `control_frequency` | 50.0 | 제어 주파수 (Hz) |

### PID 제어 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `pid_kp` | 2.0 | 비례 게인 |
| `pid_ki` | 0.1 | 적분 게인 |
| `pid_kd` | 0.5 | 미분 게인 |

### 스무딩 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `enable_smoothing` | true | 각도 스무딩 활성화 |
| `smoothing_factor` | 0.8 | 스무딩 팩터 (0-1) |

### 하드웨어 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `imu_type` | "MPU9250" | IMU 센서 타입 |
| `imu_i2c_address` | 0x68 | IMU I2C 주소 |
| `motor_i2c_address` | 0x40 | 모터 I2C 주소 |

## 고급 설정

### 1. PID 튜닝

```python
# 각도 컨트롤러에서 PID 게인 설정
angle_controller.setPIDGains(kp=2.0, ki=0.1, kd=0.5)

# PID 제어 활성화/비활성화
angle_controller.enablePID(True)
```

### 2. 스무딩 설정

```python
# 스무딩 활성화/비활성화
angle_controller.enableSmoothing(True)

# 스무딩 팩터 설정 (0-1)
angle_controller.setSmoothingFactor(0.8)
```

### 3. 각도 제어 설정

```python
# 각도 제어 설정
config = AngleControlConfig()
config.angleTolerance = 2.0
config.turnSpeed = 50.0
config.maintainSpeed = 30.0
config.controlFrequency = 50.0

angle_controller.setConfig(config)
```

## 문제 해결

### 1. IMU 센서 연결 문제

**증상**: IMU 초기화 실패, 데이터 읽기 오류

**해결 방법**:
```bash
# I2C 장치 확인
i2cdetect -y 1

# 권한 확인
sudo usermod -a -G i2c $USER
sudo chmod 666 /dev/i2c-1
```

### 2. 각도 드리프트

**증상**: 각도가 시간이 지나면서 점진적으로 변화

**해결 방법**:
- IMU 캘리브레이션 수행
- 자기계 보정 활성화 (MPU9250)
- 스무딩 팩터 조정

### 3. 회전 정확도 문제

**증상**: 90도 회전이 부정확함

**해결 방법**:
- PID 게인 튜닝
- 각도 허용 오차 조정
- 모터 속도 조정

### 4. 응답성 문제

**증상**: 각도 제어가 느리거나 불안정함

**해결 방법**:
- 제어 주파수 증가
- PID 게인 조정
- 스무딩 팩터 감소

## 성능 최적화

### 1. 정확도 향상

- **캘리브레이션**: 정기적인 IMU 캘리브레이션 수행
- **필터링**: 적절한 스무딩 팩터 사용
- **PID 튜닝**: 환경에 맞는 PID 게인 설정

### 2. 응답성 향상

- **제어 주파수**: 높은 제어 주파수 사용 (50-100Hz)
- **스무딩**: 적절한 스무딩 팩터 사용
- **모터 속도**: 적절한 회전 속도 설정

### 3. 안정성 향상

- **안전 조건**: 각도 오차 제한 설정
- **타임아웃**: 회전 작업 타임아웃 설정
- **비상 정지**: 비상 상황 시 모터 정지

## API 참조

### AngleController 클래스

#### 주요 메서드

```cpp
// 초기화
bool initialize();
bool calibrate();

// 각도 제어
bool maintainAngle(double targetAngle = 0.0);
bool turnLeft(double targetAngle = 90.0);
bool turnRight(double targetAngle = -90.0);
bool turnToAngle(double targetAngle);
bool stop();

// 제어 루프
void startControlLoop();
void stopControlLoop();

// PID 제어
void setPIDGains(double kp, double ki, double kd);
void enablePID(bool enable);
void resetPID();

// 스무딩
void enableSmoothing(bool enable);
void setSmoothingFactor(double factor);
```

#### 상태 확인

```cpp
AngleControlStatus getStatus() const;
bool isActive() const;
double getCurrentAngle() const;
double getTargetAngle() const;
AngleControlResult getCurrentResult() const;
```

### ROS2 토픽

#### 발행 토픽

- `/imu/data` - IMU 센서 데이터
- `/current_angle` - 현재 각도
- `/target_angle_pub` - 목표 각도
- `/angle_error` - 각도 오차
- `/angle_control_status` - 제어 상태

#### 구독 토픽

- `/cmd_vel` - 속도 명령
- `/target_angle` - 목표 각도 설정
- `/angle_control_enable` - 제어 활성화
- `/angle_control_command` - 문자열 명령

#### 서비스

- `/angle_control_enable` - 제어 활성화/비활성화
- `/angle_control_calibrate` - 캘리브레이션
- `/angle_control_reset` - 리셋

## 예제 코드

### C++ 예제

```cpp
#include "amr/angle_controller.h"
#include "amr/module/motor_controller.h"
#include "amr/imu_sensor.h"

int main() {
    // 컴포넌트 초기화
    auto imuSensor = std::make_shared<IMUSensor>(0x68, "MPU9250");
    auto motorController = std::make_shared<MotorController>(leftMotor, rightMotor);
    
    auto angleController = std::make_unique<AngleController>(motorController, imuSensor);
    
    // 초기화
    if (!angleController->initialize()) {
        std::cerr << "각도 컨트롤러 초기화 실패" << std::endl;
        return -1;
    }
    
    // 캘리브레이션
    angleController->calibrate();
    
    // 제어 루프 시작
    angleController->startControlLoop();
    
    // 90도 좌회전
    angleController->turnLeft(90.0);
    
    // 5초 대기
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 90도 우회전
    angleController->turnRight(90.0);
    
    // 5초 대기
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // 정지
    angleController->stop();
    angleController->stopControlLoop();
    
    return 0;
}
```

### Python 예제

```python
from tests.angle_control_test import AngleControlTest

# 테스트 객체 생성
test = AngleControlTest()

# 초기화
if test.initialize_mpu9250():
    print("IMU 초기화 성공")
    
    # 90도 회전 테스트
    test.test_90_degree_turns()
else:
    print("IMU 초기화 실패")
```

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다.

## 기여

버그 리포트, 기능 요청, 풀 리퀘스트는 언제든 환영합니다.

## 연락처

프로젝트 관련 문의사항이 있으시면 이슈를 통해 연락해 주세요. 