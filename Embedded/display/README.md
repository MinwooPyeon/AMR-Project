# 디스플레이 모듈

AMR의 LCD 디스플레이 제어를 담당하는 모듈들입니다.

## 파일 구조

- `lcd_display_controller.py`: LCD 디스플레이 컨트롤러

## 주요 기능

- LCD 화면 제어
- 이모지 기반 상태 표시
- AI 상황 기반 자동 모드 변경
- 실시간 디스플레이 업데이트

## 사용법

```python
from display import LCDDisplayController, DisplayMode

# LCD 컨트롤러 생성
lcd_controller = LCDDisplayController(16, 2)

# 디스플레이 시작
lcd_controller.start_display()

# 모드 설정
lcd_controller.set_mode(DisplayMode.NORMAL)  # 😄
lcd_controller.set_mode(DisplayMode.WARNING)  # 🚨

# AI 상황 업데이트
lcd_controller.update_ai_situation("danger_detected")
``` 