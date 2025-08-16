import time
import threading
import logging
from typing import Dict, Optional, Callable
from enum import Enum

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DisplayMode(Enum):
    NORMAL = "normal"
    WARNING = "warning"

class LCDDisplayController:
    
    def __init__(self, lcd_width: int = 16, lcd_height: int = 2):
        self.lcd_width = lcd_width
        self.lcd_height = lcd_height
        
        self.current_mode = DisplayMode.NORMAL
        self.current_message = ""
        self.display_enabled = True
        
        self.symbols = {
            DisplayMode.NORMAL: "OK",
            DisplayMode.WARNING: "WARN",
        }
        
        self.messages = {
            DisplayMode.NORMAL: "AMR Ready",
            DisplayMode.WARNING: "DANGER!",
        }
        
        self.display_thread = None
        self.display_running = False
        self.display_interval = 0.5
        
        self.display_callback = None
        
        self.ai_situation = ""
        self.situation_lock = threading.Lock()
        
        logger.info(f"LCD Display Controller initialized - size: {lcd_width}x{lcd_height}")
    
    def set_display_callback(self, callback: Callable[[str, str], None]):
        self.display_callback = callback
    
    def start_display(self):
        if self.display_running:
            logger.warning("LCD display is already running")
            return
        
        self.display_running = True
        self.display_thread = threading.Thread(target=self._display_worker, daemon=True)
        self.display_thread.start()
        logger.info("LCD display started")
    
    def stop_display(self):
        self.display_running = False
        if self.display_thread:
            self.display_thread.join(timeout=1.0)
        logger.info("LCD display stopped")
    
    def set_mode(self, mode: DisplayMode, message: str = ""):
        self.current_mode = mode
        if message:
            self.current_message = message
        else:
            self.current_message = self.messages[mode]
        
        logger.info(f"LCD display mode changed: {mode.value} - {self.current_message}")
    
    def update_ai_situation(self, situation: str):
        with self.situation_lock:
            self.ai_situation = situation
        
        if situation and ("danger" in situation.lower() or "warning" in situation.lower()):
            self.set_mode(DisplayMode.WARNING, f"DANGER: {situation}")
        else:
            self.set_mode(DisplayMode.NORMAL)
        
        logger.info(f"AI situation updated: {situation}")
    
    def _display_worker(self):
        while self.display_running:
            try:
                if self.display_enabled:
                    symbol = self.symbols[self.current_mode]
                    message = self.current_message
                    
                    display_content = self._format_display_content(symbol, message)
                    
                    if self.display_callback:
                        self.display_callback(symbol, message)
                    
                    logger.debug(f"LCD display updated: {symbol} {message}")
                
                time.sleep(self.display_interval)
                
            except Exception as e:
                logger.error(f"LCD display error: {e}")
                time.sleep(self.display_interval)
    
    def _format_display_content(self, symbol: str, message: str) -> str:
        if self.lcd_height == 1:
            content = f"{symbol}"
            return content[:self.lcd_width]
        else:
            line1 = f"{symbol}"
            line2 = message[:self.lcd_width]
            
            return {
                "line1": line1[:self.lcd_width],
                "line2": line2[:self.lcd_width]
            }
    
    def get_display_status(self) -> Dict:
        return {
            "enabled": self.display_enabled,
            "running": self.display_running,
            "current_mode": self.current_mode.value,
            "current_message": self.current_message,
            "current_symbol": self.symbols[self.current_mode],
            "ai_situation": self.ai_situation,
            "lcd_size": f"{self.lcd_width}x{self.lcd_height}"
        }
    
    def enable_display(self):
        self.display_enabled = True
        logger.info("LCD display enabled")
    
    def disable_display(self):
        self.display_enabled = False
        logger.info("LCD display disabled")
    
    def clear_display(self):
        self.current_message = ""
        logger.info("LCD display cleared")

def test_lcd_display_controller():
    print("=== LCD Display Controller Test ===")
    print("Testing normal state -> warning state -> normal state")
    print("=" * 50)
    
    lcd_controller = LCDDisplayController(16, 2)
    
    def display_callback(symbol, message):
        print(f"LCD display: {symbol} {message}")
    
    lcd_controller.set_display_callback(display_callback)
    
    lcd_controller.start_display()
    
    print("1. Normal state")
    lcd_controller.set_mode(DisplayMode.NORMAL)
    time.sleep(2)
    
    print("2. Warning state")
    lcd_controller.update_ai_situation("danger_detected")
    time.sleep(2)
    
    print("3. Return to normal state")
    lcd_controller.update_ai_situation("")
    time.sleep(2)
    
    status = lcd_controller.get_display_status()
    print(f"Final status: {status}")
    
    lcd_controller.stop_display()
    print("Test completed")

if __name__ == "__main__":
    test_lcd_display_controller() 