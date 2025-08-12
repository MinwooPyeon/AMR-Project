# AI 모터 컨트롤러 가이드

## 개요

AI 모터 컨트롤러는 Motor Driver HAT을 사용한 Jetson Nano에서 AI와의 통신을 위한 별도의 모터 제어 시스템입니다. 기존 모터 제어 코드와 독립적으로 동작하며, AI 서버로부터 받은 JSON 데이터를 처리하여 모터를 제어하고 Backend로 상태를 전송합니다.

## 주요 기능

### 1. AI 통신 기능
- AI 서버로부터 JSON 데이터 수신
- 시리얼 번호 기반 명령 필터링
- 실시간 상태 보고

### 2. Backend 통신 기능
- MQTT를 통한 Backend 서버 연결
- AI 데이터를 Backend 형식으로 변환
- 주기적 상태 전송

### 3. 모터 제어 기능
- 개별 모터 속도 및 방향 제어
- 차동 구동을 통한 로봇 이동
- 안전한 모터 정지 기능
- 하드코딩된 속도 값 사용

### 4. 지원하는 AI 케이스
- `forward` / `전진`: 로봇 전진
- `backward` / `후진`: 로봇 후진
- `left` / `좌회전`: 로봇 좌회전
- `right` / `우회전`: 로봇 우회전
- `stop` / `정지`: 로봇 정지
- `custom` / `커스텀`: X, Y 좌표 기반 세밀한 제어

## 좌표 기반 모터 제어

### 자동 모터 제어 판단
AI 데이터의 `case` 필드는 상황에 대한 설명일 뿐이며, 실제 모터 제어는 좌표 값만으로 자동 판단됩니다.

### 판단 로직
```python
# 임계값 설정
threshold = 0.3

# 정지 조건 (좌표가 모두 임계값 이하)
if abs_x <= threshold and abs_y <= threshold:
    return 'stop'

# 전진/후진 판단 (Y 좌표가 X 좌표보다 크고, Y가 양수면 전진, 음수면 후진)
if abs_y > abs_x and abs_y > threshold:
    if y > 0:
        return 'forward'
    else:
        return 'backward'

# 좌회전/우회전 판단 (X 좌표가 Y 좌표보다 크거나 비슷할 때)
if abs_x >= abs_y and abs_x > threshold:
    if x > 0:
        return 'right'
    else:
        return 'left'

# 기본값은 커스텀 (좌표 기반 개별 모터 제어)
return 'custom'
```

### 좌표 기반 판단 예시

| 좌표 | 판단된 모터 제어 | 설명 |
|------|----------------|------|
| (0.1, 0.1) | `stop` | 작은 값으로 정지 |
| (0.2, 0.8) | `forward` | Y가 양수이고 크므로 전진 |
| (0.2, -0.8) | `backward` | Y가 음수이고 크므로 후진 |
| (0.8, 0.2) | `right` | X가 양수이고 크므로 우회전 |
| (-0.8, 0.2) | `left` | X가 음수이고 크므로 좌회전 |
| (0.5, 0.5) | `custom` | X, Y가 비슷하므로 커스텀 |

### 임계값 조정
`determine_case_from_coordinates` 메서드에서 `threshold` 값을 조정하여 민감도를 변경할 수 있습니다:
- `threshold = 0.1`: 매우 민감 (작은 움직임에도 반응)
- `threshold = 0.3`: 기본값 (적당한 민감도)
- `threshold = 0.5`: 덜 민감 (큰 움직임에만 반응)

## AI JSON 구조

AI 서버로부터 받는 JSON 데이터 구조:

```json
{
    "serial": "string",      // 로봇 시리얼 번호
    "x": "float",           // X 좌표 (-1.0 ~ 1.0)
    "y": "float",           // Y 좌표 (-1.0 ~ 1.0)
    "img": "Base64",        // Base64 인코딩된 이미지
    "case": "string",       // 상황 설명 (모터 제어에는 사용하지 않음)
    "timeStamp": "string"   // 타임스탬프
}
```

**참고**: `case` 필드는 상황 설명일 뿐이며, 실제 모터 제어는 좌표 값만으로 자동 판단됩니다.

## Backend JSON 구조

Backend로 전송하는 JSON 데이터 구조:

```json
{
    "serial": "string",      // 로봇 시리얼 번호
    "state": "string",       // 로봇 상태 (항상 "RUNNING")
    "x": "float",           // X 좌표
    "y": "float",           // Y 좌표
    "speed": "float",       // 속도 (0-100)
    "angle": "float"        // 각도
}
```

**참고**: state는 항상 "RUNNING"으로 통일되어 전송됩니다.

## 하드코딩된 모터 속도 설정

### 기본 속도 설정
```python
motor_speeds = {
    'forward': 40,    # 전진 속도 (40%)
    'backward': 40,   # 후진 속도 (40%)
    'left': 20,       # 좌회전 속도 (20%)
    'right': 20,      # 우회전 속도 (20%)
    'stop': 0,        # 정지 속도 (0%)
    'custom': 20      # 커스텀 기본 속도 (20%)
}
```

### 속도 설정 변경
```python
# 새로운 속도 설정
new_speeds = {
    'forward': 60,    # 전진 속도 증가
    'backward': 50,   # 후진 속도 증가
    'left': 30,       # 좌회전 속도 증가
    'right': 30,      # 우회전 속도 증가
    'custom': 30      # 커스텀 속도 증가
}

# 속도 설정 업데이트
motor.set_motor_speed_config(new_speeds)
```

## 설치 및 설정

### 1. 의존성 설치

```bash
pip3 install -r requirements_ai.txt
```

### 2. API URL 설정

`ai_motor_controller.py` 파일에서 AI API URL을 설정:

```python
ai_api_url = "http://your-ai-server.com/api/control"
```

### 3. Backend MQTT 설정

```python
backend_broker = "192.168.100.141"  # Backend MQTT 브로커 주소
backend_port = 1883                  # Backend MQTT 포트
```

### 4. 시리얼 번호 설정

```python
motor.set_serial_number("AMR001")  # 실제 로봇 시리얼 번호로 변경
```

## 사용법

### 기본 사용법

```python
from ai_motor_controller import AIMotorController

# AI 모터 컨트롤러 초기화
motor = AIMotorController(
    debug=True,
    api_url="http://your-ai-server.com/api/control",
    backend_broker="192.168.100.141",
    backend_port=1883
)

# 시리얼 번호 설정
motor.set_serial_number("AMR001")

# Backend 연결
motor.connect_backend()

# AI 제어 루프 시작
motor.run_ai_control_loop(interval=1.0)
```

### 속도 설정 변경

```python
# 모터 속도 설정 변경
motor.set_motor_speed_config({
    'forward': 80,    # 전진 속도를 80%로 증가
    'backward': 70,   # 후진 속도를 70%로 증가
    'left': 60,       # 좌회전 속도를 60%로 증가
    'right': 60,      # 우회전 속도를 60%로 증가
    'custom': 50      # 커스텀 속도를 50%로 증가
})
```

### 수동 테스트

```bash
# AI 모터 컨트롤러 테스트
python3 test_ai_motor_controller.py

# AI 모터 컨트롤러와 Backend 통신 통합 테스트
python3 test_ai_backend_integration.py

# AI 모터 컨트롤러 실행
python3 ai_motor_controller.py
```

## API 엔드포인트

### GET 요청 (AI 데이터 수신)
- URL: `http://your-ai-server.com/api/control`
- 응답: JSON 형태의 AI 제어 데이터

### POST 요청 (상태 전송)
- URL: `http://your-ai-server.com/api/control`
- 데이터: 로봇 상태 정보

### MQTT 토픽 (Backend 전송)
- 토픽: `status`
- 브로커: `192.168.100.141:1883`
- 데이터: Backend JSON 형태

## 데이터 매핑

### AI 케이스 -> Backend 상태 매핑

| AI 케이스 | Backend 상태 | 하드코딩 속도 | 설명 |
|-----------|-------------|-------------|------|
| `forward` / `전진` | `RUNNING` | 40% | 전진 상태 |
| `backward` / `후진` | `RUNNING` | 40% | 후진 상태 |
| `left` / `좌회전` | `RUNNING` | 20% | 좌회전 상태 |
| `right` / `우회전` | `RUNNING` | 20% | 우회전 상태 |
| `stop` / `정지` | `RUNNING` | 0% | 정지 상태 |
| `custom` / `커스텀` | `RUNNING` | 20% | 커스텀 제어 상태 |
| 기타 | `RUNNING` | 20% | 대기 상태 |

**참고**: 모든 케이스에서 state는 항상 "RUNNING"으로 통일되어 전송됩니다.

### 속도 처리 방식

#### 하드코딩된 속도 값 사용
- AI 데이터에 speed 필드가 없어도 하드코딩된 속도 값 사용
- 각 케이스별로 미리 정의된 속도 값 적용
- `set_motor_speed_config()` 메서드로 런타임에 속도 변경 가능

#### 커스텀 제어
- 커스텀 케이스에서는 좌표 기반 개별 모터 제어
- Y 좌표 → 왼쪽 모터 속도
- X 좌표 → 오른쪽 모터 속도

## 오류 처리

### 시리얼 번호 불일치
- AI 데이터의 시리얼 번호가 설정된 시리얼 번호와 다르면 명령을 무시합니다.

### 잘못된 데이터
- None 데이터나 빈 딕셔너리는 무시됩니다.
- 잘못된 좌표 값은 0으로 처리됩니다.
- 알 수 없는 케이스는 무시됩니다.

### 네트워크 오류
- API 요청 실패 시 재시도하지 않고 다음 폴링까지 대기합니다.
- MQTT 연결 실패 시 재연결을 시도합니다.
- 타임아웃은 5초로 설정되어 있습니다.

### Backend 연결 오류
- Backend MQTT 브로커 연결 실패 시 오류 로그 출력
- 연결이 끊어진 경우 자동 재연결 시도

## 클래스 구조

### AIMotorController 클래스

#### 주요 메서드
- `__init__()`: AI 모터 컨트롤러 초기화
- `set_serial_number()`: 시리얼 번호 설정
- `set_motor_speed_config()`: 모터 속도 설정 변경
- `get_motor_speed()`: 케이스별 모터 속도 반환
- `connect_backend()`: Backend MQTT 연결
- `disconnect_backend()`: Backend MQTT 연결 해제
- `send_to_backend()`: Backend로 데이터 전송
- `get_ai_data()`: AI API에서 데이터 가져오기
- `process_ai_command()`: AI 명령 처리
- `execute_ai_case()`: 케이스별 모터 제어 실행
- `send_status_to_ai()`: 상태 전송
- `run_ai_control_loop()`: AI 제어 루프 실행

#### 모터 제어 메서드
- `set_motor_speed()`: 개별 모터 속도 설정
- `stop_motor()`: 개별 모터 정지
- `stop_all()`: 모든 모터 정지
- `differential_drive()`: 차동 구동
- `get_motor_status()`: 모터 상태 조회
- `test_motors()`: 모터 테스트

## 예제 시나리오

### 1. 전진 명령
```json
{
    "serial": "AMR001",
    "x": "0.5",
    "y": "0.3",
    "case": "forward",
    "timeStamp": "2024-01-01T12:00:00Z"
}
```
결과: 
- 양쪽 모터가 60% 속도로 전진 (하드코딩된 값)
- Backend로 `FORWARD` 상태 전송

### 2. 좌회전 명령
```json
{
    "serial": "AMR001",
    "x": "-0.2",
    "y": "0.8",
    "case": "left",
    "timeStamp": "2024-01-01T12:01:00Z"
}
```
결과:
- 왼쪽 모터 후진, 오른쪽 모터 전진 (50% 속도, 하드코딩된 값)
- Backend로 `LEFT_TURN` 상태 전송

### 3. 커스텀 제어
```json
{
    "serial": "AMR001",
    "x": "0.7",
    "y": "0.4",
    "case": "custom",
    "timeStamp": "2024-01-01T12:02:00Z"
}
```
결과:
- 왼쪽 모터 40% 전진, 오른쪽 모터 70% 전진 (좌표 기반)
- Backend로 `CUSTOM` 상태 전송

## 테스트

### 테스트 파일
- `test_ai_motor_controller.py`: AI 모터 컨트롤러 테스트
- `test_ai_backend_integration.py`: AI와 Backend 통신 통합 테스트

### 테스트 항목
1. AI 데이터 처리 테스트
2. 하드코딩된 speed 값 테스트
3. 속도 설정 변경 테스트
4. Backend 연결 테스트
5. AI -> Backend 데이터 흐름 테스트
6. 주기적 Backend 전송 테스트
7. Backend 데이터 매핑 테스트
8. 오류 처리 테스트

### 테스트 실행
```bash
# AI 모터 컨트롤러 테스트
python3 test_ai_motor_controller.py

# AI와 Backend 통신 통합 테스트
python3 test_ai_backend_integration.py
```

## 문제 해결

### 1. 모터가 움직이지 않음
- I2C 연결 확인
- 모터 배선 확인
- 전원 공급 확인

### 2. AI 명령이 실행되지 않음
- API URL 확인
- 네트워크 연결 확인
- 시리얼 번호 일치 확인

### 3. Backend 연결 실패
- MQTT 브로커 주소 확인
- 네트워크 연결 확인
- 방화벽 설정 확인

### 4. 속도가 예상과 다름
- 하드코딩된 속도 설정 확인
- `motor_speeds` 딕셔너리 값 확인
- 디버그 로그 확인

## 기존 코드와의 차이점

### 기존 motor_control_enhanced.py
- 기본적인 모터 제어 기능
- AI 통신 기능 없음
- Backend 통신 기능 없음
- 단순한 테스트 기능

### 새로운 ai_motor_controller.py
- AI 통신 기능 포함
- Backend MQTT 통신 기능 포함
- 하드코딩된 속도 값 사용
- 실시간 상태 보고
- 시리얼 번호 기반 필터링
- 더 강력한 오류 처리
- 데이터 매핑 및 변환
- 런타임 속도 설정 변경 가능

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다.
