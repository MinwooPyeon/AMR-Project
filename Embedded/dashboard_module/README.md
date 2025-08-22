# 웹 대시보드 모듈

## 개요

AMR 시스템의 실시간 상태를 웹 브라우저를 통해 모니터링하고 제어할 수 있는 대시보드
실시간 센서 데이터, 로봇 상태, 시스템 로그 등을 직관적인 인터페이스로 제공

## 주요 기능

### 1. 실시간 모니터링
- **로봇 상태**: 현재 위치, 방향, 속도, 배터리 상태
- **센서 데이터**: LiDAR, IMU, 카메라 데이터 실시간 표시
- **시스템 상태**: CPU, 메모리, 네트워크 사용량
- **작업 진행률**: 현재 수행 중인 작업의 진행 상황

### 2. 제어 인터페이스
- **수동 제어**: 키보드/마우스를 통한 로봇 조작
- **작업 명령**: 경로 설정, 작업 시작/중지
- **설정 변경**: 시스템 파라미터 실시간 조정
- **긴급 정지**: 비상 상황 시 즉시 정지

### 3. 데이터 시각화
- **지도 표시**: SLAM 맵과 로봇 위치 실시간 표시
- **센서 데이터 차트**: 센서 데이터의 시간별 변화 그래프
- **시스템 메트릭**: 성능 지표 대시보드
- **로그 뷰어**: 실시간 로그 확인 및 검색

### 4. 알림 시스템
- **실시간 알림**: 중요 이벤트 발생 시 즉시 알림
- **알림 히스토리**: 과거 알림 내역 조회
- **알림 설정**: 알림 유형별 설정 관리

## 파일 구조

```
dashboard_module/
├── README.md             
├── web_dashboard.py       # 메인 웹 대시보드 애플리케이션
├── static/                # 정적 파일 (CSS, JS, 이미지)
│   ├── css/
│   ├── js/
│   └── images/
├── templates/             # HTML 템플릿
│   ├── index.html
│   ├── dashboard.html
│   └── components/
└── requirements.txt       # 의존성 패키지
```

## 설치 및 실행

### 1. 의존성 설치

```bash
pip install -r requirements.txt
```

### 2. 대시보드 실행

```bash
python web_dashboard.py
```

### 3. 웹 브라우저 접속

```
http://localhost:8000
```

## 사용 방법

### 1. 메인 대시보드

메인 대시보드에서는 다음 정보를 확인할 수 있습니다:

- **로봇 상태 카드**: 현재 위치, 배터리, 상태
- **센서 데이터**: 실시간 센서 값
- **시스템 메트릭**: CPU, 메모리, 네트워크
- **작업 진행률**: 현재 작업 상태

### 2. 지도 뷰

SLAM 맵과 로봇 위치를 실시간으로 표시합니다:

- **맵 표시**: SLAM으로 생성된 환경 맵
- **로봇 위치**: 실시간 위치 및 방향
- **경로 표시**: 계획된 경로 및 실제 이동 경로
- **장애물**: 감지된 장애물 표시

### 3. 제어 패널

로봇을 직접 제어할 수 있는 인터페이스:

- **수동 제어**: 방향키를 통한 이동 제어
- **작업 명령**: 자동 주행 시작/중지
- **긴급 정지**: 비상 정지 버튼
- **설정 조정**: 속도, 감도 등 파라미터 조정

### 4. 로그 뷰어

시스템 로그를 실시간으로 확인:

- **실시간 로그**: 최신 로그 메시지
- **로그 필터링**: 레벨별, 모듈별 필터링
- **로그 검색**: 키워드 검색 기능
- **로그 다운로드**: 로그 파일 다운로드

## API 엔드포인트

### 웹소켓 API

실시간 데이터 통신을 위한 WebSocket 엔드포인트:

```javascript
// WebSocket 연결
const ws = new WebSocket('ws://localhost:8000/ws');

// 로봇 상태 수신
ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    if (data.type === 'robot_status') {
        updateRobotStatus(data.data);
    }
};

// 명령 전송
function sendCommand(command) {
    ws.send(JSON.stringify({
        type: 'command',
        data: command
    }));
}
```

### REST API

HTTP 기반 API 엔드포인트:

```bash
# 로봇 상태 조회
GET /api/robot/status

# 센서 데이터 조회
GET /api/sensors/data

# 시스템 메트릭 조회
GET /api/system/metrics

# 로그 조회
GET /api/logs?level=INFO&limit=100

# 명령 전송
POST /api/robot/command
Content-Type: application/json

{
    "command": "move_forward",
    "parameters": {
        "speed": 0.5,
        "distance": 1.0
    }
}
```

## 설정 옵션

### 대시보드 설정

```python
config = {
    'host': '0.0.0.0',           # 서버 호스트
    'port': 8000,                # 서버 포트
    'debug': False,              # 디버그 모드
    'auto_reload': True,         # 자동 리로드
    'websocket_enabled': True,   # WebSocket 활성화
    'mqtt_enabled': True,        # MQTT 통신 활성화
    'log_level': 'INFO'          # 로그 레벨
}
```

### 보안 설정

```python
security_config = {
    'authentication_required': True,    # 인증 필요
    'ssl_enabled': False,               # SSL 활성화
    'allowed_hosts': ['localhost'],     # 허용된 호스트
    'session_timeout': 3600,            # 세션 타임아웃 (초)
    'max_connections': 100              # 최대 연결 수
}
```

## 사용자 인터페이스

### 반응형 디자인

대시보드는 다양한 화면 크기에 대응하는 반응형 디자인을 적용했습니다:

- **데스크톱**: 전체 기능 제공
- **태블릿**: 터치 인터페이스 최적화
- **모바일**: 핵심 기능 중심의 간소화된 인터페이스

### 테마 및 커스터마이징

```css
/* 다크 테마 */
.dark-theme {
    --bg-color: #1a1a1a;
    --text-color: #ffffff;
    --card-bg: #2d2d2d;
    --border-color: #404040;
}

/* 라이트 테마 */
.light-theme {
    --bg-color: #ffffff;
    --text-color: #000000;
    --card-bg: #f5f5f5;
    --border-color: #e0e0e0;
}
```

## 성능 최적화

### 데이터 최적화

- **데이터 압축**: WebSocket 통신 시 데이터 압축
- **캐싱**: 정적 데이터 캐싱
- **폴링 최적화**: 필요한 데이터만 주기적 업데이트
- **메모리 관리**: 불필요한 데이터 자동 정리

### 네트워크 최적화

```python
# WebSocket 최적화 설정
websocket_config = {
    'compression': True,         # 압축 활성화
    'ping_interval': 30,         # 핑 간격 (초)
    'ping_timeout': 10,          # 핑 타임아웃 (초)
    'max_message_size': 1024*1024 # 최대 메시지 크기
}
```

## 보안 기능

### 인증 및 권한

```python
# 사용자 인증
users = {
    'admin': {
        'password': 'hashed_password',
        'role': 'admin',
        'permissions': ['read', 'write', 'control']
    },
    'operator': {
        'password': 'hashed_password',
        'role': 'operator',
        'permissions': ['read', 'control']
    },
    'viewer': {
        'password': 'hashed_password',
        'role': 'viewer',
        'permissions': ['read']
    }
}
```

### 데이터 보안

- **HTTPS**: SSL/TLS 암호화
- **세션 관리**: 안전한 세션 처리
- **입력 검증**: 모든 사용자 입력 검증
- **CSRF 보호**: Cross-Site Request Forgery 방지

## 문제 해결

### 일반적인 문제

1. **대시보드 접속 불가**
   - 서버 실행 상태 확인
   - 포트 충돌 확인
   - 방화벽 설정 확인

2. **실시간 데이터 업데이트 안됨**
   - WebSocket 연결 상태 확인
   - MQTT 브로커 연결 확인
   - 네트워크 연결 상태 확인

3. **성능 문제**
   - 브라우저 캐시 정리
   - 데이터 업데이트 주기 조정
   - 불필요한 기능 비활성화

### 디버깅

```python
# 디버그 모드 활성화
app.run(debug=True, host='0.0.0.0', port=8000)

# 로그 레벨 설정
logging.basicConfig(level=logging.DEBUG)
```

## 확장 기능

### 플러그인 시스템

대시보드는 플러그인 시스템을 지원하여 기능을 확장할 수 있습니다:

```python
# 플러그인 예시
class CustomPlugin:
    def __init__(self):
        self.name = "Custom Plugin"
        self.version = "1.0.0"
    
    def register_routes(self, app):
        @app.route('/custom')
        def custom_page():
            return render_template('custom.html')
    
    def register_websocket_handlers(self, socketio):
        @socketio.on('custom_event')
        def handle_custom_event(data):
            # 커스텀 이벤트 처리
            pass
```

### API 확장

새로운 API 엔드포인트 추가:

```python
@app.route('/api/custom/endpoint', methods=['GET'])
@require_auth
def custom_endpoint():
    return jsonify({
        'status': 'success',
        'data': 'custom data'
    })
```

## 배포

### Docker 배포

```dockerfile
FROM python:3.9-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .
EXPOSE 8000

CMD ["python", "web_dashboard.py"]
```

### 시스템 서비스

```ini
[Unit]
Description=AMR Web Dashboard
After=network.target

[Service]
Type=simple
User=amr
WorkingDirectory=/opt/amr/dashboard_module
ExecStart=/usr/bin/python3 web_dashboard.py
Restart=always

[Install]
WantedBy=multi-user.target
```