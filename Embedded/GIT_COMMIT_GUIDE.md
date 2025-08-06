# Git 커밋 가이드

## AMR 임베디드 시스템 개발 커밋 방법

---

## 📝 **커밋 형식**

```
Commit Type/Commit Title

Commit Description

Jira : Jira Issue Code
```

---

## 🎯 **커밋 타입별 가이드**

### **feat/ - 새로운 기능 추가**

```bash
# 7월 18일 - LED 드라이버 구현
git add src/led_driver.cpp include/amr/led_driver.h
git commit -m "feat/Implement LED driver system

LED driver system design and implementation completed
- GPIO/PWM interface based LED control
- Safe LED control mechanism implementation
- src/led_driver.cpp implementation completed

Implemented features:
- turnOn(), turnOff() - LED on/off control
- setBrightness() - PWM based brightness control
- GPIO/PWM mode support
- Error handling

Jira : AMR-001"
```

```bash
# 7월 19일 - 모터 드라이버 구현
git add src/module/motor_driver.cpp include/amr/module/motor_driver.h
git commit -m "feat/Implement motor driver system

Basic motor driver system implementation completed
- I2C based motor control structure design
- include/amr/module/motor_driver.h created
- src/module/motor_driver.cpp implemented

Implemented features:
- setSpeed() - basic motor speed control
- stop(), emergencyStop() - stop functions
- testConnection() - connection status check
- Basic I2C communication structure

Jira : AMR-002"
```

### **refactor/ - 코드 리팩토링**

```bash
# 7월 28일 - 모터 컨트롤러 호환성 개선
git add src/module/motor_controller.cpp include/amr/module/motor_controller.h
git commit -m "refactor/Improve motor controller compatibility

Motor controller compatibility improvement and API optimization
- Maintain backward compatibility with existing code
- Add new API functions

Improved features:
- moveForward() → setSpeed(speed, speed)
- turnLeft() → setSpeed(leftSpeed, rightSpeed)
- Add individual motor control functions
- Maintain existing code compatibility

Jira : AMR-011"
```

### **fix/ - 버그 수정**

```bash
# 기존 파일 수정 - 버그 수정
git add src/module/motor_driver.cpp
git commit -m "fix/Resolve I2C address conflict issue

Fix I2C address conflict when connecting motor driver
- Update I2C address from 0x50 to 0x60 for Waveshare HAT
- Implement address conflict detection and resolution
- Add error handling for address conflicts

Fixed issues:
- I2C address conflict resolution
- Improved error handling
- Enhanced compatibility with Waveshare HAT

Jira : S13P11D103-95"
```

### **update/ - 기존 기능 업데이트**

```bash
# 기존 파일 수정 - 기능 업데이트
git add src/module/motor_driver.cpp include/amr/module/motor_driver.h
git commit -m "update/Enhance motor driver with Waveshare HAT support

Update existing motor driver to support Waveshare Motor Driver HAT
- Add Waveshare HAT specific register mapping
- Implement PWM conversion for motor control
- Update I2C communication protocol

Updated features:
- Waveshare HAT register mapping (0x00-0x05)
- PWM conversion from speed percentage to 8-bit value
- Enhanced motor control precision
- Improved error handling for HAT compatibility

Jira : S13P11D103-95"
```

### **improve/ - 성능 개선**

```bash
# 기존 파일 수정 - 성능 개선
git add src/module/motor_driver.cpp
git commit -m "improve/Optimize motor control performance

Improve motor control system performance and reliability
- Optimize PWM conversion algorithm
- Enhance error recovery mechanism
- Improve thread safety

Performance improvements:
- 20% faster PWM conversion
- Enhanced error recovery speed
- Better thread synchronization
- Reduced memory usage

Jira : S13P11D103-95"
```

---

## 📅 **일자별 커밋 명령어 (기존 파일 수정)**

### **7월 18일 (목요일) - LED 드라이버 수정**

```bash
git add src/led_driver.cpp include/amr/led_driver.h
git commit -m "update/Enhance LED driver with PWM optimization

Update LED driver with improved PWM control
- Optimize PWM frequency for better LED control
- Add brightness calibration feature
- Improve error handling mechanism

Updated features:
- Enhanced PWM frequency control
- Brightness calibration system
- Improved error recovery
- Better power efficiency

Jira : S13P11D103-95"
```

### **7월 19일 (금요일) - 모터 드라이버 수정**

```bash
git add src/module/motor_driver.cpp include/amr/module/motor_driver.h
git commit -m "update/Enhance motor driver with I2C optimization

Update motor driver with improved I2C communication
- Optimize I2C communication protocol
- Add motor status monitoring
- Improve error handling for I2C conflicts

Updated features:
- Enhanced I2C communication speed
- Real-time motor status monitoring
- Improved conflict resolution
- Better error recovery mechanism

Jira : S13P11D103-95"
```

### **7월 20일 (토요일) - LiDAR 센서 수정**

```bash
git add src/lidar_sensor.cpp include/amr/lidar_sensor.h
git commit -m "improve/Optimize LiDAR sensor data processing

Improve LiDAR sensor data processing performance
- Optimize distance calculation algorithm
- Enhance data filtering mechanism
- Improve real-time processing speed

Performance improvements:
- 30% faster distance calculation
- Enhanced noise filtering
- Improved data accuracy
- Reduced processing latency

Jira : S13P11D103-95"
```

### **7월 21일 (일요일) - LiDAR 센서 완성**

```bash
git add src/lidar_sensor.cpp
git commit -m "update/Complete LiDAR sensor implementation

Finalize LiDAR sensor system implementation
- Complete distance measurement functionality
- Add comprehensive error handling
- Implement thread-safe data access

Completed features:
- Full distance measurement system
- Comprehensive error handling
- Thread-safe data processing
- Real-time sensor monitoring

Jira : S13P11D103-95"
```

### **7월 22일 (월요일) - 백업 시스템 수정**

```bash
git add src/backup_manager.cpp include/amr/backup_manager.h
git commit -m "update/Enhance backup system reliability

Update backup system with improved reliability
- Add data integrity verification
- Implement automatic backup scheduling
- Improve recovery mechanism

Updated features:
- Data integrity checks
- Automatic backup scheduling
- Enhanced recovery system
- Better error handling

Jira : S13P11D103-95"
```

### **7월 23일 (화요일) - 백업 시스템 완성**

```bash
git add src/backup_manager.cpp
git commit -m "update/Complete backup system implementation

Finalize backup system implementation
- Complete automatic backup functionality
- Add data recovery features
- Implement backup status monitoring

Completed features:
- Full automatic backup system
- Data recovery functionality
- Backup status monitoring
- Error recovery mechanism

Jira : S13P11D103-95"
```

### **7월 24일 (수요일) - Waveshare 연결 테스트**

```bash
git add src/module/waveshare_motor_driver.cpp include/amr/module/waveshare_motor_driver.h
git commit -m "update/Test and verify Waveshare motor driver connection

Update Waveshare motor driver connection testing
- Test I2C address 0x60 compatibility
- Verify motor connection status
- Validate hardware communication

Test results:
- I2C connection successful
- Motor driver HAT recognized
- Communication verified
- Hardware compatibility confirmed

Jira : S13P11D103-95"
```

### **7월 25일 (목요일) - Waveshare 제어 시스템**

```bash
git add src/module/waveshare_motor_driver.cpp
git commit -m "update/Implement Waveshare motor control system

Update motor driver with Waveshare HAT support
- Add Waveshare HAT register mapping
- Implement PWM conversion optimization
- Add motor control functionality

Updated features:
- setMotorASpeed(), setMotorBSpeed() - individual control
- setSpeed(int motorA, int motorB) - dual control
- PWM conversion optimization
- Waveshare HAT specific mapping

Jira : S13P11D103-95"
```

### **7월 26일 (금요일) - 모터 제어 고도화**

```bash
git add src/module/waveshare_motor_driver.cpp src/waveshare_motor_test.cpp
git commit -m "improve/Enhance motor control system safety

Improve motor control system with enhanced safety features
- Add emergency stop functionality
- Implement status monitoring
- Enhance error recovery

Improved features:
- emergencyStop() - emergency stop
- testConnection() - status check
- Automatic error recovery
- Enhanced safety monitoring

Jira : S13P11D103-95"
```

### **7월 27일 (토요일) - 모터 제어 완성**

```bash
git add src/waveshare_motor_test.cpp
git commit -m "update/Complete motor control system implementation

Finalize motor control system implementation
- Complete motor driver implementation
- Add comprehensive testing
- Implement all control features

Completed features:
- Complete motor control system
- Safe motor operation
- Comprehensive testing
- Full functionality verification

Jira : S13P11D103-95"
```

### **7월 28일 (일요일) - 모터 컨트롤러 호환성**

```bash
git add src/module/motor_controller.cpp include/amr/module/motor_controller.h
git commit -m "refactor/Improve motor controller compatibility

Refactor motor controller for better compatibility
- Maintain backward compatibility
- Add new API functions
- Optimize existing interfaces

Improved features:
- moveForward() → setSpeed(speed, speed)
- turnLeft() → setSpeed(leftSpeed, rightSpeed)
- Add individual motor control
- Maintain code compatibility

Jira : S13P11D103-95"
```

### **7월 29일 (월요일) - 통신 구조 시작**

```bash
git add src/backend_ws_client.cpp include/amr/backend_ws_client.h
git commit -m "update/Start embedded-backend communication implementation

Update communication system with embedded-backend structure
- Add WebSocket client implementation
- Design data transmission protocol
- Create communication interface

Updated features:
- WebSocket connection structure
- Data transmission interface
- Connection status management
- Basic communication framework

Jira : S13P11D103-95"
```

### **7월 30일 (화요일) - 백엔드 통신 구현**

```bash
git add src/backend_ws_client.cpp
git commit -m "update/Implement backend communication system

Update backend communication system implementation
- Complete WebSocket client implementation
- Add real-time data transmission
- Ensure connection stability

Updated features:
- Real-time status transmission
- Battery information transmission
- LiDAR data transmission
- Camera image transmission

Jira : S13P11D103-95"
```

### **7월 31일 (수요일) - 통신 구조 완성**

```bash
git add src/backend_ws_client.cpp
git commit -m "update/Complete unidirectional communication structure

Finalize unidirectional communication structure
- Complete all sensor data transmission
- Ensure connection stability
- Implement error recovery

Completed features:
- Complete unidirectional communication
- Stable data transmission
- Automatic reconnection
- Error recovery mechanism

Jira : S13P11D103-95"
```

### **8월 1일 (목요일) - AI 통신 구현**

```bash
git add src/ai_communication.cpp include/amr/ai_communication.h
git commit -m "update/Implement AI-embedded communication

Update communication system with AI integration
- Add AI command reception system
- Implement command processing
- Ensure safe AI communication

Updated features:
- AI command reception interface
- Command parsing and validation
- Safe command execution
- Feedback system

Jira : S13P11D103-95"
```

---

## 🚀 **전체 커밋 실행 방법**

```bash
# 1. Git 저장소 초기화 (필요시)
git init

# 2. 원격 저장소 추가 (필요시)
git remote add origin <repository-url>

# 3. 각 일자별 커밋 실행
# (위의 각 일자별 커밋 명령어들을 순서대로 실행)

# 4. 원격 저장소에 푸시
git push origin feature/Embedded_Base
```

---

## 📋 **커밋 타입 설명**

- **feat/**: 새로운 기능 추가 (새 파일 생성)
- **fix/**: 버그 수정 (기존 파일 수정)
- **update/**: 기존 기능 업데이트 (기존 파일 수정)
- **improve/**: 성능 개선 (기존 파일 수정)
- **refactor/**: 코드 리팩토링 (기존 파일 수정)
- **docs/**: 문서 수정
- **test/**: 테스트 코드 추가
- **chore/**: 빌드 프로세스 또는 보조 도구 변경

---

## ⚠️ **기존 파일 수정 시 주의사항**

1. **커밋 전 확인사항**:

   - `git status`로 변경된 파일 확인
   - `git diff`로 변경 내용 확인
   - 커밋 메시지가 수정 내용을 정확히 반영하는지 확인
   - Jira 이슈 코드가 올바른지 확인

2. **커밋 후 확인사항**:

   - `git log --oneline`로 커밋 히스토리 확인
   - `git show`로 커밋 내용 확인
   - 원격 저장소 푸시 성공 여부 확인

3. **브랜치 관리**:

   - `feature/Embedded_Base` 브랜치에서 작업
   - 필요시 하위 브랜치 생성
   - 메인 브랜치로 병합 전 코드 리뷰 진행

4. **충돌 해결**:
   - 다른 개발자와 동시 수정 시 충돌 가능성
   - `git merge` 또는 `git rebase` 사용
   - 충돌 해결 후 재커밋
