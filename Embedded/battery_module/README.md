# 배터리 모니터링 모듈

## 개요

AMR 시스템의 배터리 상태를 실시간으로 모니터링하고 관리하는 모듈

## 주요 기능

### 1. 배터리 상태 모니터링
- **잔량 추적**: 실시간 배터리 잔량 모니터링
- **충전 상태**: 충전/방전 상태 감지
- **온도 모니터링**: 배터리 온도 추적 및 과열 방지
- **전압 측정**: 정확한 전압 측정 및 이상 감지

### 2. 알림 시스템
- **낮은 배터리 경고**: 설정된 임계값 도달 시 알림
- **충전 완료 알림**: 충전 완료 시 알림
- **온도 경고**: 과열 시 자동 알림
- **비정상 상태 감지**: 배터리 이상 시 즉시 알림

### 3. 데이터 로깅
- **상태 기록**: 배터리 상태 변화 기록
- **통계 데이터**: 사용 패턴 분석을 위한 데이터 수집
- **이벤트 로그**: 중요 이벤트 기록

## 파일 구조

```
battery_module/
├── README.md              
├── battery_monitor.py     # 메인 배터리 모니터링 클래스
└── requirements.txt       # 의존성 패키지 (필요시)
```

## 사용 방법

### 1. 배터리 모니터 초기화

```python
from battery_module.battery_monitor import BatteryMonitor

# 배터리 모니터 인스턴스 생성
battery_monitor = BatteryMonitor()

# 모니터링 시작
battery_monitor.start_monitoring()
```

### 2. 배터리 상태 조회

```python
# 현재 배터리 상태 조회
status = battery_monitor.get_battery_status()
print(f"잔량: {status['level']}%")
print(f"충전 상태: {status['charging']}")
print(f"온도: {status['temperature']}°C")
print(f"전압: {status['voltage']}V")
```

### 3. 알림 설정

```python
# 낮은 배터리 경고 임계값 설정
battery_monitor.set_low_battery_threshold(20)  # 20%

# 온도 경고 임계값 설정
battery_monitor.set_temperature_threshold(60)  # 60°C

# 알림 콜백 함수 등록
def battery_alert(alert_type, message):
    print(f"배터리 알림: {alert_type} - {message}")

battery_monitor.set_alert_callback(battery_alert)
```

### 4. 데이터 수집

```python
# 배터리 사용 통계 조회
stats = battery_monitor.get_usage_statistics()
print(f"평균 사용 시간: {stats['avg_usage_time']}시간")
print(f"충전 주기: {stats['charge_cycles']}회")
print(f"총 사용 시간: {stats['total_usage_time']}시간")
```

## 설정 옵션

### 배터리 모니터링 설정

```python
config = {
    'monitoring_interval': 5,        # 모니터링 간격 (초)
    'low_battery_threshold': 20,     # 낮은 배터리 임계값 (%)
    'critical_battery_threshold': 10, # 위험 배터리 임계값 (%)
    'temperature_threshold': 60,      # 온도 임계값 (°C)
    'voltage_threshold': 3.0,        # 전압 임계값 (V)
    'enable_logging': True,          # 로깅 활성화
    'log_file': 'battery.log'        # 로그 파일 경로
}

battery_monitor = BatteryMonitor(config)
```

### 알림 설정

```python
alert_config = {
    'enable_email_alerts': True,     # 이메일 알림 활성화
    'enable_sms_alerts': False,      # SMS 알림 활성화
    'enable_mqtt_alerts': True,      # MQTT 알림 활성화
    'alert_recipients': ['admin@example.com'],
    'mqtt_topic': 'amr/battery/alerts'
}
```

## API 참조

### BatteryMonitor 클래스

#### 메서드

- `start_monitoring()`: 배터리 모니터링 시작
- `stop_monitoring()`: 배터리 모니터링 중지
- `get_battery_status()`: 현재 배터리 상태 조회
- `get_usage_statistics()`: 사용 통계 조회
- `set_low_battery_threshold(threshold)`: 낮은 배터리 임계값 설정
- `set_temperature_threshold(threshold)`: 온도 임계값 설정
- `set_alert_callback(callback)`: 알림 콜백 함수 설정
- `is_charging()`: 충전 상태 확인
- `get_battery_level()`: 배터리 잔량 조회
- `get_temperature()`: 배터리 온도 조회
- `get_voltage()`: 배터리 전압 조회

#### 속성

- `battery_level`: 현재 배터리 잔량 (%)
- `is_charging`: 충전 상태 (True/False)
- `temperature`: 배터리 온도 (°C)
- `voltage`: 배터리 전압 (V)
- `health_status`: 배터리 건강 상태

## 이벤트 및 알림

### 알림 유형

1. **LOW_BATTERY**: 배터리 잔량이 낮을 때
2. **CRITICAL_BATTERY**: 배터리 잔량이 위험 수준일 때
3. **HIGH_TEMPERATURE**: 배터리 온도가 높을 때
4. **CHARGING_COMPLETE**: 충전 완료 시
5. **CHARGING_STARTED**: 충전 시작 시
6. **VOLTAGE_ANOMALY**: 전압 이상 감지 시

### 이벤트 처리

```python
def handle_battery_event(event_type, data):
    if event_type == 'LOW_BATTERY':
        print(f"배터리 잔량 부족: {data['level']}%")
        # 자동 충전 시작 또는 작업 중단
    elif event_type == 'HIGH_TEMPERATURE':
        print(f"배터리 온도 높음: {data['temperature']}°C")
        # 냉각 시스템 활성화

battery_monitor.set_event_handler(handle_battery_event)
```

## 로깅 및 디버깅

### 로그 레벨

- **INFO**: 일반적인 배터리 상태 정보
- **WARNING**: 경고 상황 (낮은 배터리, 높은 온도)
- **ERROR**: 오류 상황 (센서 오류, 통신 실패)
- **DEBUG**: 디버깅 정보

### 로그 예시

```
[2024-01-15 10:30:15] INFO: Battery level: 85%, Temperature: 25°C, Voltage: 12.6V
[2024-01-15 10:35:20] WARNING: Battery level below threshold: 18%
[2024-01-15 10:40:30] INFO: Charging started
[2024-01-15 11:15:45] INFO: Charging completed, Battery level: 100%
```

## 문제 해결

### 일반적인 문제

1. **센서 연결 오류**
   - I2C 연결 확인
   - 배터리 모니터 하드웨어 점검
   - 권한 설정 확인

2. **부정확한 측정값**
   - 센서 캘리브레이션 수행
   - 배터리 타입 설정 확인
   - 온도 보정 확인

3. **알림이 오지 않는 경우**
   - 알림 설정 확인
   - 콜백 함수 등록 확인
   - 네트워크 연결 상태 확인

### 디버깅 모드

```python
# 디버깅 모드 활성화
battery_monitor.enable_debug_mode()

# 상세 로그 출력
battery_monitor.set_log_level('DEBUG')
```

## 성능 최적화

### 모니터링 간격 조정

```python
# 고성능 모드 (1초 간격)
battery_monitor.set_monitoring_interval(1)

# 절전 모드 (30초 간격)
battery_monitor.set_monitoring_interval(30)
```

### 데이터 캐싱

```python
# 캐시 활성화로 성능 향상
battery_monitor.enable_cache(True)
battery_monitor.set_cache_duration(5)  # 5초 캐시
```

## 보안 고려사항

1. **데이터 암호화**: 민감한 배터리 데이터 암호화
2. **접근 제어**: 배터리 정보 접근 권한 제한
3. **로그 보안**: 로그 파일 접근 권한 설정
4. **네트워크 보안**: MQTT 통신 암호화