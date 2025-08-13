# 간단한 키보드 수동 제어 시스템

IMU와 서보모터 없이 기본적인 모터 제어만으로 키보드 입력을 통해 AMR 로봇을 수동으로 조작할 수 있는 간단한 컨트롤러입니다. Backend MQTT와 AI 통신 기능은 유지됩니다.

## 기능

- **키보드 입력 처리**: Windows와 Linux 환경 모두 지원
- **기본 모터 제어**: 전진, 후진, 좌회전, 우회전
- **속도 조절**: 4단계 속도 설정 (20%, 50%, 80%, 100%)
- **실시간 상태 모니터링**: 모터 상태 정보 출력
- **Backend MQTT 통신**: 로봇 상태를 Backend로 전송
- **AI 통신**: AI 서버와의 통신 지원 (선택사항)

## 파일 구조

```
motor_control/
├── simple_manual_controller.py    # 메인 간단한 컨트롤러
├── test_simple_manual.py         # 테스트 스크립트
├── PCA9685.py                    # PCA9685 모터 드라이버
└── README_simple_manual.md       # 이 파일
```

## 설치 및 실행

### 1. 기본 실행

```bash
cd motor_control
python3 simple_manual_controller.py
```

### 2. 테스트 실행

```bash
cd motor_control
python3 test_simple_manual.py
```

## 키보드 조작법

### 이동 제어

- **W** - 전진
- **S** - 후진
- **A** - 좌회전
- **D** - 우회전
- **SPACE** - 정지

### 속도 제어

- **1** - 저속 (20%)
- **2** - 중속 (50%) - 기본값
- **3** - 고속 (80%)
- **4** - 최고속 (100%)

### 기타

- **ESC** 또는 **Ctrl+C** - 종료

## 시스템 요구사항

### 하드웨어

- PCA9685 모터 드라이버 (I2C 주소: 0x40)
- 2개의 DC 모터 (왼쪽/오른쪽)

### 소프트웨어

- Python 3.6+
- smbus2 라이브러리
- requests 라이브러리 (AI 통신용)

## 설정

### I2C 설정

기본적으로 I2C 버스 1을 사용합니다. 다른 버스를 사용하려면:

```python
controller = SimpleManualController(i2c_bus=0)  # 버스 0 사용
```

### 모터 드라이버 주소 설정

다른 I2C 주소를 사용하려면:

```python
controller = SimpleManualController(i2c_address=0x41)  # 주소 0x41 사용
```

### Backend MQTT 설정

Backend MQTT 연결을 위한 설정:

```python
controller = SimpleManualController(
    backend_broker="192.168.100.141",
    backend_port=1883
)
```

### AI 통신 설정

AI 서버와의 통신을 위한 설정:

```python
controller = SimpleManualController(
    api_url="http://localhost:5001/command"
)
```

## 주요 클래스

### SimpleManualController

기본적인 모터 제어와 키보드 입력 처리 기능을 제공하는 클래스입니다.

#### 주요 메서드

- `start_keyboard_control()`: 키보드 제어 시작
- `stop_keyboard_control()`: 키보드 제어 정지
- `run_manual_control()`: 수동 제어 메인 루프
- `differential_drive(left_speed, right_speed)`: 차동 구동 제어
- `stop_all()`: 모든 모터 정지
- `get_current_status()`: 현재 상태 정보 반환
- `connect_backend()`: Backend MQTT 연결
- `disconnect_backend()`: Backend MQTT 연결 해제
- `send_to_backend()`: Backend로 상태 데이터 전송
- `get_ai_data()`: AI 서버에서 명령 데이터 가져오기

## 상태 모니터링

시스템은 5초마다 현재 상태를 출력합니다:

```
[상태] 모터A: 50%(forward), 모터B: 50%(forward), 속도: 50%, Backend: 연결됨
```

## Backend MQTT 통신

### 전송되는 데이터

- **serial**: 로봇 시리얼 번호 (기본값: "AMR001")
- **state**: 로봇 상태 (기본값: "RUNNING")
- **x, y**: 위치 좌표 (IMU 없으므로 0, 0)
- **speed**: 현재 속도 (%)
- **angle**: 현재 각도 (IMU 없으므로 0도)

### 연결 설정

```python
# Backend 연결
if controller.connect_backend():
    print("Backend 연결 성공")
else:
    print("Backend 연결 실패")
```

## AI 통신

### AI 명령 수신

```python
# AI 서버에서 명령 가져오기
ai_data = controller.get_ai_data()
if ai_data:
    print(f"AI 명령: {ai_data}")
```

### AI API 설정

```python
controller = SimpleManualController(
    api_url="http://localhost:5001/command"
)
```

## 문제 해결

### 1. 키보드 입력이 인식되지 않는 경우

**Windows 환경:**

- 관리자 권한으로 실행
- 다른 프로그램이 키보드를 점유하고 있는지 확인

**Linux 환경:**

- 터미널에서 실행 중인지 확인
- 다른 프로그램이 터미널을 점유하고 있는지 확인

### 2. 모터가 동작하지 않는 경우

- I2C 연결 확인
- PCA9685 드라이버 주소 확인 (기본: 0x40)
- 전원 공급 확인
- 모터 배선 확인

### 3. Backend MQTT 연결 실패

- MQTT 브로커 주소와 포트 확인
- 네트워크 연결 상태 확인
- MQTT 브로커 서비스 실행 여부 확인

### 4. AI 통신 실패

- AI 서버 URL 확인
- AI 서버 실행 여부 확인
- 네트워크 연결 상태 확인

### 5. 모터 방향이 반대인 경우

모터 A와 B의 방향이 반대로 연결된 경우, `differential_drive` 메서드에서 방향을 조정할 수 있습니다.

## 예제 코드

### 기본 사용법

```python
from simple_manual_controller import SimpleManualController

# 컨트롤러 초기화
controller = SimpleManualController(debug=True)

# 수동 제어 시작
controller.run_manual_control()
```

### Backend MQTT 포함 사용법

```python
# Backend MQTT 포함 초기화
controller = SimpleManualController(
    debug=True,
    backend_broker="192.168.100.141",
    backend_port=1883
)

# 시리얼 번호 설정
controller.set_serial_number("AMR001")

# Backend 연결
controller.connect_backend()

# 수동 제어 시작
controller.run_manual_control()
```

### AI 통신 포함 사용법

```python
# AI 통신 포함 초기화
controller = SimpleManualController(
    debug=True,
    api_url="http://localhost:5001/command",
    backend_broker="192.168.100.141",
    backend_port=1883
)

# 시리얼 번호 설정
controller.set_serial_number("AMR001")

# Backend 연결
controller.connect_backend()

# 수동 제어 시작
controller.run_manual_control()
```

### 직접 모터 제어

```python
# 컨트롤러 초기화
controller = SimpleManualController()

# 전진 (양쪽 모터 50% 속도)
controller.differential_drive(50, 50)

# 2초 대기
time.sleep(2)

# 정지
controller.stop_all()
```

## 차이점 (기존 IMU 버전과 비교)

| 기능          | IMU 버전     | 간단한 버전  |
| ------------- | ------------ | ------------ |
| IMU 센서      | ✅ 지원      | ❌ 미지원    |
| 서보 모터     | ✅ 지원      | ❌ 미지원    |
| 정확한 회전   | ✅ 90도 정확 | ❌ 시간 기반 |
| 정중앙 맞추기 | ✅ 지원      | ❌ 미지원    |
| Backend MQTT  | ✅ 지원      | ✅ 지원      |
| AI 통신       | ✅ 지원      | ✅ 지원      |
| 복잡도        | 높음         | 낮음         |
| 의존성        | 많음         | 적음         |

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다.

## 기여

버그 리포트나 기능 요청은 이슈 트래커를 통해 제출해 주세요.
