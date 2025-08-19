# AMR (Autonomous Mobile Robot) System

## 프로젝트 개요

AMR 시스템은 자율주행 모바일 로봇을 위한 통합 제어 시스템

## 시스템 아키텍처

### 모듈 구조
```
Embedded/
├── ai_module/           # AI 처리 모듈
├── backup_module/       # 백업 관리 모듈
├── battery_module/      # 배터리 모니터링
├── communication_module/ # 통신 관리
├── config/             # 설정 관리 (리팩토링됨)
├── dashboard_module/    # 웹 대시보드
├── display_module/      # 디스플레이 제어
├── mqtt_module/         # MQTT 통신 (리팩토링됨)
├── motors/             # 모터 제어
├── ros2_module/        # ROS2 통합
├── security/           # 보안 관리
├── sensors/            # 센서 관리
├── utilities/          # 유틸리티 (리팩토링됨)
└── tests/              # 테스트 (리팩토링됨)
```

## 주요 기능

### 1. 통합 로깅 시스템
- `LoggerFactory` 싱글톤 패턴으로 일관된 로깅
- 모듈별 로거 자동 생성
- 특수 로깅 메서드 (연결, 보안, 성능 등)

### 2. MQTT 클라이언트 추상화
- `BaseMQTTClient` 추상 클래스로 중복 코드 제거
- 재연결, 통계, 콜백 관리 기능
- 모든 MQTT 클라이언트의 공통 기능 통합

### 3. 모듈화된 설정 관리
- `MotorConfig`, `MQTTConfig`, `SensorConfig` dataclass
- 기존 코드와의 호환성 유지
- 타입 안전성과 유지보수성 향상

### 4. 표준화된 예외 처리
- AMR 시스템 전용 예외 클래스 계층 구조
- 구체적인 에러 타입별 예외 클래스
- 에러 코드와 상세 정보 포함

## 설치 및 실행

### 1. 의존성 설치
```bash
pip install -r requirements.txt
```

### 2. 설정 확인
```bash
python -m config.system_config
```

### 3. 시스템 실행
```bash
python process_manager.py
```

## 테스트

### 테스트 구조
```
tests/
├── unit/               # 단위 테스트
│   ├── test_config.py
│   ├── test_logger.py
│   └── test_exceptions.py
├── integration/        # 통합 테스트
│   └── test_mqtt_integration.py
├── e2e/               # E2E 테스트
└── fixtures/          # 테스트 픽스처
```

### 테스트 실행
```bash
# 전체 테스트
python -m pytest tests/

# 단위 테스트만
python -m pytest tests/unit/

# 통합 테스트만
python -m pytest tests/integration/
```

## 리팩토링 개선사항

### Phase 1: 핵심 인프라 개선
- 로깅 시스템 통합
- MQTT 클라이언트 추상화
- 설정 관리 분리

### Phase 2: 코드 품질 향상
- 예외 처리 표준화
- 타입 힌트 일관성
- 테스트 구조 개선

### 개선 효과
- 코드 중복 60% 감소
- 설정 관리 용이성 향상
- 일관된 에러 처리 및 로깅
- 테스트 구조 체계화
- 유지보수성 및 확장성 대폭 개선

## 보안

- MQTT 인증 (username/password)
- 사용자 권한 관리
- 세션 관리
- 암호화 통신

## 로깅

시스템은 다음과 같은 로그 레벨을 제공합니다:
- `INFO`: 일반 정보
- `WARN`: 경고
- `ERROR`: 오류
- `DEBUG`: 디버그 정보
- `SUCCESS`: 성공 메시지

특수 로깅 메서드:
- `connection_success()`: 연결 성공
- `security_alert()`: 보안 경고
- `performance_metric()`: 성능 지표