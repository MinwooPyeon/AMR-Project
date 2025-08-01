#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
파일 정리 스크립트
프로젝트의 파일들을 기능별로 폴더에 정리합니다.
"""

import os
import shutil
import sys

def create_directories():
    """필요한 디렉토리들을 생성합니다."""
    directories = [
        "python/ai_subscriber",      # AI 구독자 관련
        "python/amr_sync",          # AMR 동기화 관련
        "python/backend",            # 백엔드 관련
        "python/motor_control",      # 모터 제어 관련
        "python/test",               # 테스트 파일들
        "scripts",                   # 실행 스크립트
        "docs"                       # 문서
    ]
    
    for directory in directories:
        os.makedirs(directory, exist_ok=True)
        print(f"✅ 디렉토리 생성: {directory}")

def move_files():
    """파일들을 적절한 폴더로 이동합니다."""
    file_moves = [
        # AI 구독자 관련
        ("ai_position_subscriber.py", "python/ai_subscriber/"),
        
        # AMR 동기화 관련
        ("amr_real_data_sync.py", "python/amr_sync/"),
        
        # 백엔드 관련
        ("backend_mqtt_subscriber.py", "python/backend/"),
        
        # 모터 제어 관련
        ("real_motor_controller.py", "python/motor_control/"),
        
        # 테스트 파일들
        ("test_amr_with_ai_subscriber.py", "python/test/"),
        ("test_backup_system.py", "python/test/"),
        ("test_bidirectional_communication.py", "python/test/"),
        
        # 스크립트
        ("run_sensor_sync.sh", "scripts/"),
    ]
    
    for source, destination in file_moves:
        if os.path.exists(source):
            shutil.move(source, destination)
            print(f"✅ 파일 이동: {source} -> {destination}")
        else:
            print(f"⚠️  파일 없음: {source}")

def create_readme():
    """각 폴더에 README 파일을 생성합니다."""
    
    # AI Subscriber README
    ai_readme = """# AI Subscriber

AI에서 전송하는 위치 및 명령 데이터를 수신하는 모듈입니다.

## 파일
- `ai_position_subscriber.py`: AI 위치 데이터 구독자

## 기능
- AI 위치 데이터 수신 (Pose2D, Point, Float64MultiArray, JSON)
- AI 명령 데이터 수신
- ROS2 토픽 구독
- 콜백 함수 지원

## 사용법
```python
from ai_position_subscriber import AIPositionSubscriber

subscriber = AIPositionSubscriber()
subscriber.set_position_callback(your_callback_function)
```
"""
    
    # AMR Sync README
    amr_readme = """# AMR Sync

실제 AMR 데이터 동기화 시스템입니다.

## 파일
- `amr_real_data_sync.py`: AMR 실제 데이터 동기화

## 기능
- 센서 데이터 동기화
- MQTT 통신
- 백업 시스템
- AI 명령 처리
- 모터 제어

## 사용법
```python
from amr_real_data_sync import AMRRealDataSync

amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True, enable_backup=True)
amr_sync.start_sync()
```
"""
    
    # Backend README
    backend_readme = """# Backend

백엔드 MQTT Subscriber 모듈입니다.

## 파일
- `backend_mqtt_subscriber.py`: 백엔드 MQTT 구독자

## 기능
- AMR 데이터 수신
- 명령 전송
- MQTT 브로커 연결
- 양방향 통신

## 사용법
```python
from backend_mqtt_subscriber import BackendMQTTSubscriber

backend = BackendMQTTSubscriber("192.168.100.141", 1883)
backend.connect_mqtt()
backend.subscribe_to_amr_data("AMR001")
```
"""
    
    # Motor Control README
    motor_readme = """# Motor Control

실제 모터 제어 모듈입니다.

## 파일
- `real_motor_controller.py`: 실제 모터 컨트롤러

## 기능
- PCA9685 기반 모터 제어
- Motor Driver HAT 지원
- 시뮬레이션 모드
- 속도 및 방향 제어

## 사용법
```python
from real_motor_controller import RealMotorController

controller = RealMotorController()
controller.set_speed(50.0, 50.0)  # 좌측, 우측 속도
controller.stop()
```
"""
    
    # Test README
    test_readme = """# Test

테스트 파일들입니다.

## 파일
- `test_amr_with_ai_subscriber.py`: AI Subscriber 통합 테스트
- `test_backup_system.py`: 백업 시스템 테스트
- `test_bidirectional_communication.py`: 양방향 통신 테스트

## 실행 방법
```bash
python3 test_amr_with_ai_subscriber.py
python3 test_backup_system.py
python3 test_bidirectional_communication.py
```
"""
    
    # Scripts README
    scripts_readme = """# Scripts

실행 스크립트들입니다.

## 파일
- `run_sensor_sync.sh`: 센서 동기화 실행 스크립트

## 사용법
```bash
chmod +x run_sensor_sync.sh
./run_sensor_sync.sh
```

## 메뉴 옵션
1. AI에서 수신 테스트
2. 백엔드와 양방향 통신 테스트
3. 모터 동작 테스트
4. 백업 시스템 테스트
"""
    
    readme_files = [
        ("python/ai_subscriber/README.md", ai_readme),
        ("python/amr_sync/README.md", amr_readme),
        ("python/backend/README.md", backend_readme),
        ("python/motor_control/README.md", motor_readme),
        ("python/test/README.md", test_readme),
        ("scripts/README.md", scripts_readme),
    ]
    
    for filepath, content in readme_files:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✅ README 생성: {filepath}")

def create_main_readme():
    """메인 README 파일을 생성합니다."""
    main_readme = """# Embedded AMR System

임베디드 AMR 시스템의 Python 모듈들입니다.

## 폴더 구조

```
Embedded/
├── python/
│   ├── ai_subscriber/     # AI 구독자 관련
│   ├── amr_sync/         # AMR 동기화 관련
│   ├── backend/          # 백엔드 관련
│   ├── motor_control/    # 모터 제어 관련
│   └── test/             # 테스트 파일들
├── scripts/              # 실행 스크립트
├── config/               # 설정 파일들
├── include/              # C++ 헤더 파일들
├── src/                  # C++ 소스 파일들
└── Motor_Driver_HAT_Code/ # 모터 드라이버 코드
```

## 주요 기능

### 1. AI Subscriber (`python/ai_subscriber/`)
- AI에서 전송하는 위치 및 명령 데이터 수신
- ROS2 토픽 구독
- 다양한 메시지 타입 지원 (Pose2D, Point, JSON 등)

### 2. AMR Sync (`python/amr_sync/`)
- 실제 AMR 데이터 동기화
- 센서 데이터 통합
- MQTT 통신
- 백업 시스템

### 3. Backend (`python/backend/`)
- 백엔드 MQTT Subscriber
- AMR 데이터 수신
- 명령 전송
- 양방향 통신

### 4. Motor Control (`python/motor_control/`)
- 실제 모터 제어
- PCA9685 기반
- Motor Driver HAT 지원
- 시뮬레이션 모드

### 5. Test (`python/test/`)
- 통합 테스트
- 백업 시스템 테스트
- 양방향 통신 테스트

## 빠른 시작

1. **AI Subscriber 테스트**
   ```bash
   cd python/test
   python3 test_amr_with_ai_subscriber.py
   ```

2. **양방향 통신 테스트**
   ```bash
   cd python/test
   python3 test_bidirectional_communication.py
   ```

3. **백업 시스템 테스트**
   ```bash
   cd python/test
   python3 test_backup_system.py
   ```

4. **전체 시스템 실행**
   ```bash
   cd scripts
   ./run_sensor_sync.sh
   ```

## 데이터 구조

### AMR -> Backend
```json
{
  "serial": "AMR001",
  "state": "RUNNING",
  "x": "10.0",
  "y": "10.0",
  "speed": "25.0",
  "battery_level": "85.5"
}
```

### AI -> Embedded
```json
{
  "MOVING_FORWARD": "",
  "ROTATE_LEFT": "",
  "ROTATE_RIGHT": "",
  "MOVING_BACKWARD": "",
  "STOP": "",
  "img": ".jpg",
  "situation": "",
  "x": "10.0",
  "y": "10.0"
}
```

## 의존성

- Python 3.7+
- ROS2 (AI Subscriber용)
- paho-mqtt (MQTT 통신용)
- PCA9685 (모터 제어용)

## 라이센스

이 프로젝트는 S13P11D103 팀의 임베디드 AMR 시스템입니다.
"""
    
    with open("README.md", 'w', encoding='utf-8') as f:
        f.write(main_readme)
    print("✅ 메인 README 생성: README.md")

def main():
    """메인 함수"""
    print("=== Embedded AMR System 파일 정리 ===")
    print()
    
    # 1. 디렉토리 생성
    print("1. 디렉토리 생성 중...")
    create_directories()
    print()
    
    # 2. 파일 이동
    print("2. 파일 이동 중...")
    move_files()
    print()
    
    # 3. README 파일 생성
    print("3. README 파일 생성 중...")
    create_readme()
    create_main_readme()
    print()
    
    print("✅ 파일 정리 완료!")
    print()
    print("📁 새로운 폴더 구조:")
    print("  python/ai_subscriber/     - AI 구독자 관련")
    print("  python/amr_sync/         - AMR 동기화 관련")
    print("  python/backend/          - 백엔드 관련")
    print("  python/motor_control/    - 모터 제어 관련")
    print("  python/test/             - 테스트 파일들")
    print("  scripts/                 - 실행 스크립트")
    print("  docs/                    - 문서")
    print()
    print("📖 각 폴더에 README.md 파일이 생성되었습니다.")

if __name__ == "__main__":
    main() 