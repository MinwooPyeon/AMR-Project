# AMR 시스템 설정 관리

## 개요

`config/system_config.py` 파일은 AMR 시스템의 모든 수치 값들을 중앙에서 관리하는 설정 파일입니다. 하드코딩된 값들을 변수로 관리하여 일관성과 유지보수성을 높였습니다.

## 주요 기능

### 1. 중앙 집중식 설정 관리
- 모든 수치 값들을 한 곳에서 관리
- 환경 변수를 통한 동적 설정 변경
- 타입 안전성 보장

### 2. 카테고리별 설정 그룹화
- **MQTT 설정**: 브로커 주소, 포트, 타임아웃
- **모터 설정**: I2C 주소, PWM 주파수, 속도 제한
- **IMU 설정**: 샘플링 레이트, 센서 범위
- **각도 제어 설정**: PID 파라미터, 제어 주파수
- **통신 설정**: 웹소켓 포트, HTTP 포트
- **성능 설정**: 제어 루프 주파수, 안전 제한

## 사용법

### 1. 기본 사용법

```python
from config.system_config import get_config

# 설정 인스턴스 가져오기
config = get_config()

# 개별 값 사용
mqtt_broker = config.MQTT_BROKER
motor_speed = config.MOTOR_DEFAULT_SPEED
imu_address = config.IMU_I2C_ADDRESS
```

### 2. 카테고리별 설정 가져오기

```python
from config.system_config import get_mqtt_config, get_motor_config

# MQTT 설정 전체 가져오기
mqtt_config = get_mqtt_config()
# {'broker': '192.168.100.141', 'port': 1883, 'timeout': 60, 'keepalive': 60}

# 모터 설정 전체 가져오기
motor_config = get_motor_config()
# {'i2c_address': 64, 'pwm_frequency': 50, 'max_speed': 100, ...}
```

### 3. 환경 변수를 통한 설정 변경

```bash
# 환경 변수로 설정 변경
export MQTT_BROKER="192.168.1.100"
export MQTT_PORT="1884"
export SYSTEM_NAME="AMR002"
export DEBUG_MODE="true"

# Python에서 자동으로 환경 변수 반영
python your_script.py
```

## 설정 항목

### MQTT 설정
```python
config.MQTT_BROKER = "192.168.100.141"    # MQTT 브로커 주소
config.MQTT_PORT = 1883                   # MQTT 포트
config.MQTT_TIMEOUT = 60                  # 연결 타임아웃 (초)
config.MQTT_KEEPALIVE = 60                # Keep-alive 간격 (초)
```

### 모터 설정
```python
config.MOTOR_I2C_ADDRESS = 0x40           # 모터 드라이버 I2C 주소
config.MOTOR_PWM_FREQUENCY = 50           # PWM 주파수 (Hz)
config.MOTOR_MAX_SPEED = 100              # 최대 속도 (%)
config.MOTOR_MIN_SPEED = 10               # 최소 속도 (%)
config.MOTOR_DEFAULT_SPEED = 50           # 기본 속도 (%)
config.MOTOR_TURN_SPEED = 40              # 회전 속도 (%)
config.MOTOR_PWM_RESOLUTION = 4095        # PWM 해상도
config.MOTOR_SPEED_MULTIPLIER = 40.95     # 속도 변환 계수
```

### IMU 센서 설정
```python
config.IMU_I2C_ADDRESS = 0x4B             # IMU I2C 주소
config.IMU_SAMPLE_RATE = 100              # 샘플링 레이트 (Hz)
config.IMU_GYRO_RANGE = 250               # 자이로스코프 범위 (±°/s)
config.IMU_ACCEL_RANGE = 2                # 가속도계 범위 (±g)
config.IMU_MAG_RANGE = 1300               # 자기계 범위 (±μT)
```

### 각도 제어 설정
```python
config.ANGLE_CONTROL_FREQUENCY = 50.0     # 제어 주파수 (Hz)
config.ANGLE_TOLERANCE = 1.0              # 허용 오차 (도)
config.ANGLE_MAX_ERROR = 180.0            # 최대 오차 (도)
config.ANGLE_PID_KP = 2.0                 # PID 비례 게인
config.ANGLE_PID_KI = 0.1                 # PID 적분 게인
config.ANGLE_PID_KD = 0.5                 # PID 미분 게인
```

### 회전 각도 설정
```python
config.ROTATION_90_DEGREES = 90.0         # 90도 회전
config.ROTATION_180_DEGREES = 180.0       # 180도 회전
config.ROTATION_360_DEGREES = 360.0       # 360도 회전
```

### 성능 설정
```python
config.CONTROL_LOOP_FREQUENCY = 50.0      # 제어 루프 주파수 (Hz)
config.CONTROL_LOOP_PERIOD_MS = 20        # 제어 루프 주기 (ms)
config.MAX_SPEED_LIMIT = 80.0             # 최대 속도 제한 (%)
config.EMERGENCY_STOP_DELAY = 0.1         # 비상 정지 지연 (초)
```

## 설정 파일 수정

### 1. 직접 수정
```python
# config/system_config.py 파일에서 직접 수정
config.MOTOR_MAX_SPEED = 80  # 최대 속도를 80%로 변경
config.MQTT_BROKER = "192.168.1.100"  # MQTT 브로커 변경
```

### 2. 환경 변수로 오버라이드
```bash
# 시스템 시작 전 환경 변수 설정
export MOTOR_MAX_SPEED="80"
export MQTT_BROKER="192.168.1.100"
```

### 3. 런타임에 동적 변경
```python
from config.system_config import get_config

config = get_config()
config.MOTOR_MAX_SPEED = 80  # 런타임에 값 변경
```

## 설정 검증

### 1. 설정 출력
```python
from config.system_config import get_config

config = get_config()
config.print_config()
```

### 2. 설정 테스트
```bash
# 설정 파일 직접 실행
python config/system_config.py
```

## 모듈별 설정 적용

### MQTT 모듈
```python
# 기존: 하드코딩된 값
def __init__(self, broker="192.168.100.141", port=1883):
    self.broker = broker
    self.port = port

# 개선: 설정 파일 사용
def __init__(self, broker=None, port=None):
    from config.system_config import get_config
    config = get_config()
    self.broker = broker or config.MQTT_BROKER
    self.port = port or config.MQTT_PORT
```

### 모터 제어 모듈
```python
# 기존: 하드코딩된 값
self.pwm.setPWMFreq(50)
self.pwm.setPWM(self.PWMA, 0, int(speed * 40.95))

# 개선: 설정 파일 사용
from config.system_config import get_config
config = get_config()
self.pwm.setPWMFreq(config.MOTOR_PWM_FREQUENCY)
self.pwm.setPWM(self.PWMA, 0, int(speed * config.MOTOR_SPEED_MULTIPLIER))
```

## 장점

1. **일관성**: 모든 모듈에서 동일한 설정 값 사용
2. **유지보수성**: 한 곳에서 수정하면 전체 시스템에 반영
3. **유연성**: 환경 변수로 동적 설정 변경 가능
4. **타입 안전성**: Python 타입 힌트로 오류 방지
5. **문서화**: 설정 값들의 의미와 단위가 명확히 문서화됨

## 주의사항

1. **설정 파일 import**: 모듈에서 설정을 사용할 때 올바른 경로로 import
2. **환경 변수**: 대문자로 작성하고 문자열 값 사용
3. **타입 변환**: 환경 변수는 문자열이므로 필요시 타입 변환
4. **기본값**: 설정 파일에 기본값을 정의하여 안정성 보장
