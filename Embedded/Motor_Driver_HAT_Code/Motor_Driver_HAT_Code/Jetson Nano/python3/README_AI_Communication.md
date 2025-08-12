# Motor Driver HAT AI 통신 가이드

## 개요

이 문서는 Motor Driver HAT을 사용한 Jetson Nano에서 AI와의 통신을 위한 기능을 설명합니다.

## AI JSON 구조

AI 서버로부터 받는 JSON 데이터 구조:

```json
{
    "serial": "string",      // 로봇 시리얼 번호
    "x": "float",           // X 좌표 (-1.0 ~ 1.0)
    "y": "float",           // Y 좌표 (-1.0 ~ 1.0)
    "img": "Base64",        // Base64 인코딩된 이미지
    "case": "string",       // 제어 케이스
    "timeStamp": "string"   // 타임스탬프
}
```

## 지원하는 케이스

### 기본 이동 케이스
- `forward` / `전진`: 로봇 전진
- `backward` / `후진`: 로봇 후진
- `left` / `좌회전`: 로봇 좌회전
- `right` / `우회전`: 로봇 우회전
- `stop` / `정지`: 로봇 정지

### 커스텀 제어
- `custom` / `커스텀`: X, Y 좌표를 기반으로 한 세밀한 제어

## 설치 및 설정

### 1. 의존성 설치

```bash
pip3 install -r requirements.txt
```

### 2. API URL 설정

`motor_control_enhanced.py` 파일에서 AI API URL을 설정:

```python
ai_api_url = "http://your-ai-server.com/api/control"
```

### 3. 시리얼 번호 설정

```python
motor.set_serial_number("AMR001")  # 실제 로봇 시리얼 번호로 변경
```

## 사용법

### 기본 사용법

```python
from motor_control_enhanced import MotorDriverHAT

# 모터 드라이버 초기화
motor = MotorDriverHAT(
    debug=True,
    api_url="http://your-ai-server.com/api/control"
)

# 시리얼 번호 설정
motor.set_serial_number("AMR001")

# AI 제어 루프 시작
motor.run_ai_control_loop(interval=1.0)
```

### 수동 테스트

```bash
# AI 통신 테스트
python3 test_ai_communication.py

# 기본 모터 테스트
python3 motor_control_enhanced.py
```

## API 엔드포인트

### GET 요청 (AI 데이터 수신)
- URL: `http://your-ai-server.com/api/control`
- 응답: JSON 형태의 AI 제어 데이터

### POST 요청 (상태 전송)
- URL: `http://your-ai-server.com/api/control`
- 데이터: 로봇 상태 정보

```json
{
    "serial": "AMR001",
    "motor_a_speed": 50,
    "motor_b_speed": 50,
    "motor_a_direction": "forward",
    "motor_b_direction": "forward",
    "timestamp": "2024-01-01T12:00:00Z"
}
```

## 속도 계산

### 기본 케이스
속도는 X, Y 좌표의 절댓값 합계로 계산됩니다:
```
속도 = min(100, max(20, |x| + |y|))
```

### 커스텀 케이스
- 왼쪽 모터 속도 = Y 좌표 × 100
- 오른쪽 모터 속도 = X 좌표 × 100

## 오류 처리

### 시리얼 번호 불일치
- AI 데이터의 시리얼 번호가 설정된 시리얼 번호와 다르면 명령을 무시합니다.

### 잘못된 데이터
- None 데이터나 빈 딕셔너리는 무시됩니다.
- 잘못된 좌표 값은 0으로 처리됩니다.
- 알 수 없는 케이스는 무시됩니다.

### 네트워크 오류
- API 요청 실패 시 재시도하지 않고 다음 폴링까지 대기합니다.
- 타임아웃은 5초로 설정되어 있습니다.

## 디버그 모드

디버그 모드를 활성화하면 다음 정보가 출력됩니다:
- AI 데이터 수신 로그
- 모터 제어 명령 실행 로그
- 오류 메시지
- 상태 전송 결과

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
결과: 양쪽 모터가 80% 속도로 전진

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
결과: 왼쪽 모터 후진, 오른쪽 모터 전진 (100% 속도)

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
결과: 왼쪽 모터 40% 전진, 오른쪽 모터 70% 전진

## 문제 해결

### 1. 모터가 움직이지 않음
- I2C 연결 확인
- 모터 배선 확인
- 전원 공급 확인

### 2. AI 명령이 실행되지 않음
- API URL 확인
- 네트워크 연결 확인
- 시리얼 번호 일치 확인

### 3. 속도가 예상과 다름
- 좌표 값 범위 확인 (-1.0 ~ 1.0)
- 케이스 이름 확인
- 디버그 로그 확인

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다.
