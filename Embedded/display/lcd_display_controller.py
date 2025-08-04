#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LCD 디스플레이 컨트롤러
AMR의 LCD 화면에 이모지와 경고 메시지를 표시하는 시스템
"""

import time
import threading
import logging
from typing import Dict, Optional, Callable
from enum import Enum

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DisplayMode(Enum):
    """디스플레이 모드 열거형"""
    NORMAL = "normal"      # 정상 상태 (웃는 얼굴)
    WARNING = "warning"    # 위험 상황 (경고 표시)

class LCDDisplayController:
    """LCD 디스플레이 컨트롤러 클래스"""
    
    def __init__(self, lcd_width: int = 16, lcd_height: int = 2):
        self.lcd_width = lcd_width
        self.lcd_height = lcd_height
        
        # 디스플레이 상태
        self.current_mode = DisplayMode.NORMAL
        self.current_message = ""
        self.display_enabled = True
        
        # 이모지 정의 (더 큰 이모지 사용)
        self.emojis = {
            DisplayMode.NORMAL: "😄",    # 큰 웃는 얼굴
            DisplayMode.WARNING: "🚨",   # 큰 경고 표시
        }
        
        # 메시지 정의
        self.messages = {
            DisplayMode.NORMAL: "AMR Ready",
            DisplayMode.WARNING: "DANGER!",
        }
        
        # 디스플레이 스레드
        self.display_thread = None
        self.display_running = False
        self.display_interval = 0.5  # 500ms마다 업데이트
        
        # 콜백 함수
        self.display_callback = None
        
        # AI 상황 감지
        self.ai_situation = ""
        self.situation_lock = threading.Lock()
        
        logger.info(f"LCD Display Controller 초기화 완료 - 크기: {lcd_width}x{lcd_height}")
    
    def set_display_callback(self, callback: Callable[[str, str], None]):
        """디스플레이 콜백 함수 설정"""
        self.display_callback = callback
    
    def start_display(self):
        """디스플레이 시작"""
        if self.display_running:
            logger.warning("LCD 디스플레이가 이미 실행 중입니다")
            return
        
        self.display_running = True
        self.display_thread = threading.Thread(target=self._display_worker, daemon=True)
        self.display_thread.start()
        logger.info("LCD 디스플레이 시작")
    
    def stop_display(self):
        """디스플레이 중지"""
        self.display_running = False
        if self.display_thread:
            self.display_thread.join(timeout=1.0)
        logger.info("LCD 디스플레이 중지")
    
    def set_mode(self, mode: DisplayMode, message: str = ""):
        """디스플레이 모드 설정"""
        self.current_mode = mode
        if message:
            self.current_message = message
        else:
            self.current_message = self.messages[mode]
        
        logger.info(f"LCD 디스플레이 모드 변경: {mode.value} - {self.current_message}")
    
    def update_ai_situation(self, situation: str):
        """AI 상황 업데이트"""
        with self.situation_lock:
            self.ai_situation = situation
        
        # 상황에 따른 모드 변경 (평소/위험 2가지 모드만)
        if situation and ("danger" in situation.lower() or "warning" in situation.lower()):
            self.set_mode(DisplayMode.WARNING, f"DANGER: {situation}")
        else:
            self.set_mode(DisplayMode.NORMAL)
        
        logger.info(f"AI 상황 업데이트: {situation}")
    
    def _display_worker(self):
        """디스플레이 워커"""
        while self.display_running:
            try:
                if self.display_enabled:
                    # 현재 이모지와 메시지 가져오기
                    emoji = self.emojis[self.current_mode]
                    message = self.current_message
                    
                    # LCD 화면에 표시할 내용 생성
                    display_content = self._format_display_content(emoji, message)
                    
                    # 콜백 호출 (실제 LCD 하드웨어 제어)
                    if self.display_callback:
                        self.display_callback(emoji, message)
                    
                    logger.debug(f"LCD 디스플레이 업데이트: {emoji} {message}")
                
                time.sleep(self.display_interval)
                
            except Exception as e:
                logger.error(f"LCD 디스플레이 오류: {e}")
                time.sleep(self.display_interval)
    
    def _format_display_content(self, emoji: str, message: str) -> str:
        """디스플레이 내용 포맷팅"""
        # LCD 크기에 맞게 내용 조정
        if self.lcd_height == 1:
            # 한 줄 LCD - 이모지를 중앙에 크게 표시
            content = f"{emoji}"
            return content[:self.lcd_width]
        else:
            # 두 줄 LCD - 이모지를 첫 줄에 크게, 메시지를 두 번째 줄에
            line1 = f"{emoji}"
            line2 = message[:self.lcd_width]
            
            return {
                "line1": line1[:self.lcd_width],
                "line2": line2[:self.lcd_width]
            }
    
    def get_display_status(self) -> Dict:
        """디스플레이 상태 조회"""
        return {
            "enabled": self.display_enabled,
            "running": self.display_running,
            "current_mode": self.current_mode.value,
            "current_message": self.current_message,
            "current_emoji": self.emojis[self.current_mode],
            "ai_situation": self.ai_situation,
            "lcd_size": f"{self.lcd_width}x{self.lcd_height}"
        }
    
    def enable_display(self):
        """디스플레이 활성화"""
        self.display_enabled = True
        logger.info("LCD 디스플레이 활성화")
    
    def disable_display(self):
        """디스플레이 비활성화"""
        self.display_enabled = False
        logger.info("LCD 디스플레이 비활성화")
    
    def clear_display(self):
        """디스플레이 클리어"""
        self.current_message = ""
        logger.info("LCD 디스플레이 클리어")

def test_lcd_display_controller():
    """LCD 디스플레이 컨트롤러 테스트"""
    print("=== LCD 디스플레이 컨트롤러 테스트 ===")
    print("정상 상태 → 위험 상황 → 정상 상태 순서로 테스트합니다.")
    print("=" * 50)
    
    # LCD 컨트롤러 생성
    lcd_controller = LCDDisplayController(16, 2)
    
    # 디스플레이 콜백 설정
    def display_callback(emoji, message):
        print(f"LCD 표시: {emoji} {message}")
    
    lcd_controller.set_display_callback(display_callback)
    
    # 디스플레이 시작
    lcd_controller.start_display()
    
    # 테스트 시나리오
    print("1. 정상 상태 (웃는 얼굴)")
    lcd_controller.set_mode(DisplayMode.NORMAL)
    time.sleep(2)
    
    print("2. 위험 상황 (경고 표시)")
    lcd_controller.update_ai_situation("danger_detected")
    time.sleep(2)
    
    print("3. 정상 상태로 복귀")
    lcd_controller.update_ai_situation("")
    time.sleep(2)
    
    # 상태 출력
    status = lcd_controller.get_display_status()
    print(f"최종 상태: {status}")
    
    # 디스플레이 중지
    lcd_controller.stop_display()
    print("테스트 완료")

if __name__ == "__main__":
    test_lcd_display_controller() 