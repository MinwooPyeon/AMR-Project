# 디스플레이 모듈

AMR(Autonomous Mobile Robot) 프로젝트의 디스플레이 기능

## 개요

- **LCDDisplayController**: LCD 화면 제어 및 상태 표시
- **LEDController**: LED 조명 제어 및 패턴 관리
- **DisplayManager**: LCD와 LED를 통합 관리하는 매니저
- **DisplayConfig**: 디스플레이 설정 및 구성 관리

## 설치

### 의존성

```bash
pip install -r requirements.txt
```

## 사용법

### 디스플레이 매니저 (통합 사용)

```python
from display import DisplayManager

# 디스플레이 매니저 생성
manager = DisplayManager(16, 2)

# 콜백 함수 설정
def lcd_callback(symbol, message):
    print(f"LCD: {symbol} {message}")

def led_callback(color, pattern, brightness):
    print(f"LED: {color} {pattern} {brightness}%")

manager.set_display_callbacks(lcd_callback, led_callback)

# 디스플레이 시작
manager.start_display()

# 상태 설정
manager.set_status("normal", "System Ready")
manager.set_status("warning", "Obstacle Detected")
manager.set_status("error", "Connection Lost")

# AI 상황 업데이트
manager.update_ai_situation("danger_detected")
```

### LCD 컨트롤러 (개별 사용)

```python
from display import LCDDisplayController, DisplayMode

lcd_controller = LCDDisplayController(16, 2)

def display_callback(symbol, message):
    print(f"LCD: {symbol} {message}")

lcd_controller.set_display_callback(display_callback)
lcd_controller.start_display()

lcd_controller.set_mode(DisplayMode.NORMAL, "AMR Ready")
lcd_controller.set_mode(DisplayMode.WARNING, "DANGER!")
```

### LED 컨트롤러 (개별 사용)

```python
from display import LEDController, LEDColor, LEDPattern

led_controller = LEDController()

def led_callback(color, pattern, brightness):
    print(f"LED: {color} {pattern} {brightness}%")

led_controller.set_led_callback(led_callback)
led_controller.start_led()

led_controller.set_color(LEDColor.GREEN, LEDPattern.SOLID)
led_controller.set_color(LEDColor.RED, LEDPattern.BLINK)
led_controller.set_brightness(75)
```

## 기능

### LCD 디스플레이

- **상태 표시**: 정상/경고 상태를 심볼과 메시지로 표시
- **AI 상황 연동**: AI 감지 상황에 따른 자동 모드 변경
- **실시간 업데이트**: 500ms 간격으로 디스플레이 업데이트
- **크기 지원**: 16x2, 20x4 등 다양한 LCD 크기 지원

### LED 조명

- **색상 제어**: 빨강, 초록, 파랑, 노랑, 흰색, 꺼짐
- **패턴 지원**: 고정, 깜빡임, 펄스, 웨이브 패턴
- **밝기 조절**: 0-100% 밝기 제어
- **상태 연동**: 시스템 상태에 따른 자동 색상 변경

### 통합 관리

- **동기화**: LCD와 LED 상태 동기화
- **상태 관리**: 통합된 상태 설정 및 조회
- **콜백 시스템**: 하드웨어 제어를 위한 콜백 인터페이스

## 설정

### 환경 변수

```env
# LCD 설정
LCD_WIDTH=16
LCD_HEIGHT=2
LCD_UPDATE_INTERVAL=0.5

# LED 설정
LED_UPDATE_INTERVAL=0.1
```

### 디스플레이 모드

- **normal**: 정상 상태 (초록색 LED, "OK" 심볼)
- **warning**: 경고 상태 (노랑색 LED 깜빡임, "WARN" 심볼)
- **error**: 오류 상태 (빨강색 LED 펄스, "ERR" 심볼)
- **info**: 정보 상태 (파랑색 LED, "INFO" 심볼)
- **success**: 성공 상태 (초록색 LED, "OK" 심볼)

## 테스트

```bash
# 디스플레이 매니저 테스트
python -m display.display_manager

# LCD 컨트롤러 테스트
python -m display.lcd_display_controller

# LED 컨트롤러 테스트
python -m display.led_controller
```
