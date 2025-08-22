# API 참조 문서

## AI 모듈

### AIAlertPublisher

AI 알림 발행자 클래스입니다.

#### 메서드

```python
__init__(robot_id: str = "AMR001", mqtt_broker: str = "localhost", mqtt_port: int = 1883)
```

AI 알림 발행자를 초기화합니다.

```python
publish_alert(situation: AlertSituation, message: str = "", data: dict = None)
```

알림을 발행합니다.

```python
publish_emergency_stop(reason: str = "Emergency stop activated")
```

비상 정지 알림을 발행합니다.

```python
cleanup()
```

리소스를 정리합니다.

### AIDataSimulator

AI 데이터 시뮬레이터 클래스입니다.

#### 메서드

```python
__init__(robot_id: str = "AMR001", data_file: str = "ai_data.json")
```

AI 데이터 시뮬레이터를 초기화합니다.

```python
generate_sample_data()
```

샘플 AI 데이터를 생성합니다.

```python
simulate_ai_data(interval: float = 1.0)
```

AI 데이터 시뮬레이션을 시작합니다.

### AIFileClient

AI 파일 클라이언트 클래스입니다.

#### 메서드

```python
__init__(data_file: str = "ai_data.json")
```

AI 파일 클라이언트를 초기화합니다.

```python
get_current_data() -> dict
```

현재 AI 데이터를 반환합니다.

```python
get_statistics() -> dict
```

통계 정보를 반환합니다.

```python
reset_statistics()
```

통계를 초기화합니다.

### AIPositionSubscriber

AI 위치 구독자 클래스입니다.

#### 메서드

```python
__init__(robot_id: str = "AMR001", mqtt_broker: str = "localhost", mqtt_port: int = 1883)
```

AI 위치 구독자를 초기화합니다.

```python
set_command_callback(callback: Callable[[AICommand], None])
```

명령 콜백 함수를 설정합니다.

```python
save_image(image_data: str, filename: str = None) -> str
```

이미지를 저장합니다.

```python
cleanup()
```

리소스를 정리합니다.

## 통신 모듈

### CommunicationManager

통신 관리자 클래스입니다.

#### 메서드

```python
__init__(mqtt_broker: str = "localhost", mqtt_port: int = 1883)
```

통신 관리자를 초기화합니다.

```python
setup_mqtt(client_id: str = None, username: str = None, password: str = None)
```

MQTT 연결을 설정합니다.

```python
subscribe_mqtt(topic: str, callback: Callable)
```

MQTT 토픽을 구독합니다.

```python
publish_mqtt(topic: str, message: str, qos: int = 0)
```

MQTT 메시지를 발행합니다.

```python
setup_websocket_server(host: str = "localhost", port: int = 8765)
```

WebSocket 서버를 설정합니다.

```python
setup_http_client(base_url: str)
```

HTTP 클라이언트를 설정합니다.

```python
http_get(endpoint: str) -> requests.Response
```

HTTP GET 요청을 수행합니다.

```python
http_post(endpoint: str, data: dict) -> requests.Response
```

HTTP POST 요청을 수행합니다.

```python
send_json_message(protocol: str, message: dict)
```

JSON 메시지를 전송합니다.

```python
broadcast_message(message: dict)
```

메시지를 브로드캐스트합니다.

```python
get_connection_status() -> dict
```

연결 상태를 반환합니다.

```python
cleanup()
```

리소스를 정리합니다.

### ProtocolHandler

프로토콜 처리기 클래스입니다.

#### 메서드

```python
encode_json(data: dict) -> str
```

데이터를 JSON으로 인코딩합니다.

```python
decode_json(json_str: str) -> dict
```

JSON 문자열을 디코딩합니다.

```python
encode_binary(data: bytes) -> str
```

바이너리 데이터를 인코딩합니다.

```python
decode_binary(encoded_data: str) -> bytes
```

인코딩된 데이터를 디코딩합니다.

```python
create_message(message_type: str, payload: dict) -> dict
```

메시지를 생성합니다.

```python
parse_message(message: dict) -> tuple
```

메시지를 파싱합니다.

```python
validate_message(message: dict) -> bool
```

메시지를 검증합니다.

```python
create_sensor_data(sensor_type: str, values: dict) -> dict
```

센서 데이터를 생성합니다.

```python
create_command(command_type: str, parameters: dict) -> dict
```

명령을 생성합니다.

```python
create_status(status: str, details: dict) -> dict
```

상태를 생성합니다.

```python
create_error(error_code: str, message: str) -> dict
```

오류를 생성합니다.

```python
create_ack(message_id: str) -> dict
```

확인 메시지를 생성합니다.

```python
pack_binary_data(data: bytes) -> str
```

바이너리 데이터를 패킹합니다.

```python
unpack_binary_data(packed_data: str) -> bytes
```

패킹된 데이터를 언패킹합니다.

```python
create_heartbeat(device_id: str) -> dict
```

하트비트를 생성합니다.

```python
create_config_update(config: dict) -> dict
```

설정 업데이트를 생성합니다.

```python
extract_message_type(message: dict) -> str
```

메시지 타입을 추출합니다.

```python
extract_payload(message: dict) -> dict
```

페이로드를 추출합니다.

## 설정 모듈

### ConfigManager

설정 관리자 클래스입니다.

#### 메서드

```python
__init__(config_dir: str = "config")
```

설정 관리자를 초기화합니다.

```python
load_all_configs()
```

모든 설정을 로드합니다.

```python
load_config(config_name: str) -> dict
```

특정 설정을 로드합니다.

```python
save_config(config_name: str, config_data: dict)
```

설정을 저장합니다.

```python
get_config(config_name: str) -> dict
```

설정을 반환합니다.

```python
get_parameter(config_name: str, parameter_path: str, default=None)
```

매개변수를 반환합니다.

```python
set_parameter(config_name: str, parameter_path: str, value)
```

매개변수를 설정합니다.

```python
get_robot_config() -> dict
```

로봇 설정을 반환합니다.

```python
get_motor_config() -> dict
```

모터 설정을 반환합니다.

```python
get_imu_config() -> dict
```

IMU 설정을 반환합니다.

```python
get_camera_config() -> dict
```

카메라 설정을 반환합니다.

```python
get_mqtt_config() -> dict
```

MQTT 설정을 반환합니다.

```python
get_ai_config() -> dict
```

AI 설정을 반환합니다.

```python
get_nav2_config() -> dict
```

Nav2 설정을 반환합니다.

```python
get_slam_config() -> dict
```

SLAM 설정을 반환합니다.

```python
validate_config(config_name: str) -> bool
```

설정을 검증합니다.

```python
list_configs() -> list
```

설정 목록을 반환합니다.

```python
reload_configs()
```

모든 설정을 다시 로드합니다.

## 디스플레이 모듈

### LCDDisplayController

LCD 디스플레이 컨트롤러 클래스입니다.

#### 메서드

```python
__init__(width: int = 128, height: int = 64, i2c_address: int = 0x3C)
```

LCD 디스플레이 컨트롤러를 초기화합니다.

```python
start_display()
```

디스플레이를 시작합니다.

```python
stop_display()
```

디스플레이를 중지합니다.

```python
clear_display()
```

디스플레이를 지웁니다.

```python
update_display(text: str, line: int = 0)
```

디스플레이를 업데이트합니다.

```python
set_display_mode(mode: str)
```

디스플레이 모드를 설정합니다.

```python
show_status(status: str)
```

상태를 표시합니다.

```python
show_error(error_message: str)
```

오류를 표시합니다.

```python
show_warning(warning_message: str)
```

경고를 표시합니다.

```python
show_info(info_message: str)
```

정보를 표시합니다.

```python
show_success(success_message: str)
```

성공 메시지를 표시합니다.

### LEDController

LED 컨트롤러 클래스입니다.

#### 메서드

```python
__init__(pin: int = 18)
```

LED 컨트롤러를 초기화합니다.

```python
set_color(color: LEDColor)
```

LED 색상을 설정합니다.

```python
set_brightness(brightness: int)
```

LED 밝기를 설정합니다.

```python
set_status_led(status: str)
```

상태 LED를 설정합니다.

```python
start_led()
```

LED를 시작합니다.

```python
stop_led()
```

LED를 중지합니다.

```python
enable_led()
```

LED를 활성화합니다.

```python
disable_led()
```

LED를 비활성화합니다.

```python
get_led_status() -> dict
```

LED 상태를 반환합니다.

### DisplayManager

디스플레이 관리자 클래스입니다.

#### 메서드

```python
__init__()
```

디스플레이 관리자를 초기화합니다.

```python
start_display()
```

디스플레이를 시작합니다.

```python
stop_display()
```

디스플레이를 중지합니다.

```python
set_display_callbacks(lcd_callback=None, led_callback=None)
```

디스플레이 콜백을 설정합니다.

```python
set_status(status: str)
```

상태를 설정합니다.

```python
update_ai_situation(situation: str)
```

AI 상황을 업데이트합니다.

```python
set_lcd_mode(mode: str)
```

LCD 모드를 설정합니다.

```python
set_led_color(color: LEDColor)
```

LED 색상을 설정합니다.

```python
set_led_brightness(brightness: int)
```

LED 밝기를 설정합니다.

```python
get_display_status() -> dict
```

디스플레이 상태를 반환합니다.

```python
enable_display()
```

디스플레이를 활성화합니다.

```python
disable_display()
```

디스플레이를 비활성화합니다.

```python
clear_display()
```

디스플레이를 지웁니다.

## 백업 모듈

### BackupManager

백업 관리자 클래스입니다.

#### 메서드

```python
__init__(backup_dir: str = "backups")
```

백업 관리자를 초기화합니다.

```python
create_backup(source_path: str, backup_name: str = None) -> str
```

백업을 생성합니다.

```python
create_zip_backup(source_path: str, backup_name: str = None) -> str
```

ZIP 백업을 생성합니다.

```python
list_backups() -> list
```

백업 목록을 반환합니다.

```python
restore_backup(backup_name: str, restore_path: str = None) -> bool
```

백업을 복원합니다.

```python
delete_backup(backup_name: str) -> bool
```

백업을 삭제합니다.

```python
cleanup_old_backups(keep_count: int = 5)
```

오래된 백업을 정리합니다.

### AutoBackup

자동 백업 클래스입니다.

#### 메서드

```python
__init__(backup_manager: BackupManager)
```

자동 백업을 초기화합니다.

```python
create_daily_backup()
```

일일 백업을 생성합니다.

```python
create_weekly_backup()
```

주간 백업을 생성합니다.

```python
create_monthly_backup()
```

월간 백업을 생성합니다.

```python
setup_schedule()
```

스케줄을 설정합니다.

```python
run()
```

자동 백업을 실행합니다.

```python
create_manual_backup(backup_name: str = None) -> str
```

수동 백업을 생성합니다.

## 데이터 구조

### 열거형 (Enums)

#### AlertSituation

```python
class AlertSituation(Enum):
    NORMAL = "normal"
    WARNING = "warning"
    ERROR = "error"
    EMERGENCY = "emergency"
```

#### AICommand

```python
class AICommand(Enum):
    STOP = "STOP"
    MOVING_FORWARD = "MOVING_FORWARD"
    MOVING_BACKWARD = "MOVING_BACKWARD"
    ROTATE_LEFT = "ROTATE_LEFT"
    ROTATE_RIGHT = "ROTATE_RIGHT"
```

#### LEDColor

```python
class LEDColor(Enum):
    RED = "red"
    GREEN = "green"
    BLUE = "blue"
    YELLOW = "yellow"
    WHITE = "white"
    OFF = "off"
```

#### LEDPattern

```python
class LEDPattern(Enum):
    SOLID = "solid"
    BLINK = "blink"
    PULSE = "pulse"
    WAVE = "wave"
```

### 데이터클래스 (Dataclasses)

#### AlertData

```python
@dataclass
class AlertData:
    robot_id: str
    situation: AlertSituation
    message: str
    timestamp: str
    data: Optional[dict] = None
```

#### AIPositionData

```python
@dataclass
class AIPositionData:
    robot_id: str
    x: float
    y: float
    angle: float
    timestamp: str
    image_data: Optional[str] = None
```

## 오류 처리

### 사용자 정의 예외

#### ConfigError

```python
class ConfigError(Exception):
    """설정 관련 오류"""
    pass
```

#### CommunicationError

```python
class CommunicationError(Exception):
    """통신 관련 오류"""
    pass
```

#### HardwareError

```python
class HardwareError(Exception):
    """하드웨어 관련 오류"""
    pass
```

#### BackupError

```python
class BackupError(Exception):
    """백업 관련 오류"""
    pass
```

## 설정 파일 예시

### 로봇 설정 (robot_config.yaml)

```yaml
robot:
  name: "AMR001"
  type: "autonomous_mobile_robot"
  frame_id: "base_link"

dimensions:
  length: 0.5
  width: 0.3
  height: 0.2

velocity_limits:
  linear_max: 1.0
  angular_max: 2.0

hardware:
  motor_enabled: true
  imu_enabled: true
  camera_enabled: true
  lidar_enabled: true
  servo_enabled: true
  led_enabled: true
  battery_monitor_enabled: true

safety:
  emergency_stop_enabled: true
  collision_detection_enabled: true
  battery_low_threshold: 20.0
```

### 모터 설정 (motor_config.yaml)

```yaml
motor:
  pwm_frequency: 50
  speed_limit: 100

pins:
  left_motor_a: 0
  left_motor_b: 1
  right_motor_a: 2
  right_motor_b: 3

servo:
  pan_servo: 4
  tilt_servo: 5

control:
  acceleration_rate: 0.1
  deceleration_rate: 0.1
  min_speed: 10
  max_speed: 100
```

### MQTT 설정 (mqtt_config.yaml)

```yaml
mqtt:
  broker: "localhost"
  port: 1883
  keepalive: 60
  client_id: "amr_client"

topics:
  robot_status: "amr/status"
  robot_command: "amr/command"
  sensor_data: "amr/sensor"
  ai_data: "amr/ai"
  alert: "amr/alert"

qos:
  status: 1
  command: 2
  sensor: 0
  ai: 1
  alert: 2
```
