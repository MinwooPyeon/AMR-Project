import time
import threading
import logging
from typing import Dict, Optional, Callable
from enum import Enum

logger = logging.getLogger(__name__)

class LEDColor(Enum):
    RED = "red"
    GREEN = "green"
    BLUE = "blue"
    YELLOW = "yellow"
    WHITE = "white"
    OFF = "off"

class LEDPattern(Enum):
    SOLID = "solid"
    BLINK = "blink"
    PULSE = "pulse"
    WAVE = "wave"

class LEDController:
    def __init__(self):
        self.current_color = LEDColor.OFF
        self.current_pattern = LEDPattern.SOLID
        self.brightness = 100
        self.enabled = True
        
        self.led_thread = None
        self.led_running = False
        self.update_interval = 0.1
        
        self.led_callback = None
        self.pattern_data = {}
        
        logger.info("LED Controller initialized")
    
    def set_led_callback(self, callback: Callable[[str, str, int], None]):
        self.led_callback = callback
    
    def start_led(self):
        if self.led_running:
            logger.warning("LED controller is already running")
            return
        
        self.led_running = True
        self.led_thread = threading.Thread(target=self._led_worker, daemon=True)
        self.led_thread.start()
        logger.info("LED controller started")
    
    def stop_led(self):
        self.led_running = False
        if self.led_thread:
            self.led_thread.join(timeout=1.0)
        logger.info("LED controller stopped")
    
    def set_color(self, color: LEDColor, pattern: LEDPattern = LEDPattern.SOLID):
        self.current_color = color
        self.current_pattern = pattern
        logger.info(f"LED color set to {color.value} with pattern {pattern.value}")
    
    def set_brightness(self, brightness: int):
        self.brightness = max(0, min(100, brightness))
        logger.info(f"LED brightness set to {self.brightness}")
    
    def set_status_led(self, status: str):
        if status == "normal":
            self.set_color(LEDColor.GREEN, LEDPattern.SOLID)
        elif status == "warning":
            self.set_color(LEDColor.YELLOW, LEDPattern.BLINK)
        elif status == "error":
            self.set_color(LEDColor.RED, LEDPattern.PULSE)
        elif status == "info":
            self.set_color(LEDColor.BLUE, LEDPattern.SOLID)
        else:
            self.set_color(LEDColor.OFF)
        
        logger.info(f"LED status set to {status}")
    
    def _led_worker(self):
        while self.led_running:
            try:
                if self.enabled and self.led_callback:
                    if self.current_pattern == LEDPattern.SOLID:
                        self.led_callback(self.current_color.value, "solid", self.brightness)
                    elif self.current_pattern == LEDPattern.BLINK:
                        self.led_callback(self.current_color.value, "blink", self.brightness)
                    elif self.current_pattern == LEDPattern.PULSE:
                        self.led_callback(self.current_color.value, "pulse", self.brightness)
                    elif self.current_pattern == LEDPattern.WAVE:
                        self.led_callback(self.current_color.value, "wave", self.brightness)
                
                time.sleep(self.update_interval)
                
            except Exception as e:
                logger.error(f"LED controller error: {e}")
                time.sleep(self.update_interval)
    
    def get_led_status(self) -> Dict:
        return {
            "enabled": self.enabled,
            "running": self.led_running,
            "current_color": self.current_color.value,
            "current_pattern": self.current_pattern.value,
            "brightness": self.brightness
        }
    
    def enable_led(self):
        self.enabled = True
        logger.info("LED controller enabled")
    
    def disable_led(self):
        self.enabled = False
        logger.info("LED controller disabled")

def main():
    print("=== LED Controller Test ===")
    
    controller = LEDController()
    
    def led_callback(color, pattern, brightness):
        print(f"LED: {color} {pattern} {brightness}%")
    
    controller.set_led_callback(led_callback)
    controller.start_led()
    
    print("Testing different LED states...")
    
    controller.set_status_led("normal")
    time.sleep(2)
    
    controller.set_status_led("warning")
    time.sleep(2)
    
    controller.set_status_led("error")
    time.sleep(2)
    
    controller.set_status_led("info")
    time.sleep(2)
    
    controller.stop_led()
    print("LED controller test completed")

if __name__ == "__main__":
    main()
