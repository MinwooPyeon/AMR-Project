import logging
from typing import Dict, Any, Optional
from .lcd_display_controller import LCDDisplayController, DisplayMode
from .led_controller import LEDController, LEDColor, LEDPattern
from .display_config import DisplayConfig

logger = logging.getLogger(__name__)

class DisplayManager:
    def __init__(self, lcd_width: int = 16, lcd_height: int = 2):
        self.lcd_controller = LCDDisplayController(lcd_width, lcd_height)
        self.led_controller = LEDController()
        self.enabled = True
        
        logger.info("Display Manager initialized")
    
    def start_display(self):
        self.lcd_controller.start_display()
        self.led_controller.start_led()
        logger.info("Display Manager started")
    
    def stop_display(self):
        self.lcd_controller.stop_display()
        self.led_controller.stop_led()
        logger.info("Display Manager stopped")
    
    def set_display_callbacks(self, lcd_callback, led_callback):
        self.lcd_controller.set_display_callback(lcd_callback)
        self.led_controller.set_led_callback(led_callback)
    
    def set_status(self, status: str, message: str = ""):
        if status == "normal":
            self.lcd_controller.set_mode(DisplayMode.NORMAL, message or DisplayConfig.get_message("normal"))
            self.led_controller.set_status_led("normal")
        elif status == "warning":
            self.lcd_controller.set_mode(DisplayMode.WARNING, message or DisplayConfig.get_message("warning"))
            self.led_controller.set_status_led("warning")
        elif status == "error":
            self.lcd_controller.set_mode(DisplayMode.WARNING, message or DisplayConfig.get_message("error"))
            self.led_controller.set_status_led("error")
        elif status == "info":
            self.lcd_controller.set_mode(DisplayMode.NORMAL, message or DisplayConfig.get_message("info"))
            self.led_controller.set_status_led("info")
        elif status == "success":
            self.lcd_controller.set_mode(DisplayMode.NORMAL, message or DisplayConfig.get_message("success"))
            self.led_controller.set_status_led("normal")
        
        logger.info(f"Display status set to {status}: {message}")
    
    def update_ai_situation(self, situation: str):
        self.lcd_controller.update_ai_situation(situation)
        
        if situation and ("danger" in situation.lower() or "warning" in situation.lower()):
            self.led_controller.set_status_led("warning")
        else:
            self.led_controller.set_status_led("normal")
    
    def set_lcd_mode(self, mode: DisplayMode, message: str = ""):
        self.lcd_controller.set_mode(mode, message)
    
    def set_led_color(self, color: LEDColor, pattern: LEDPattern = LEDPattern.SOLID):
        self.led_controller.set_color(color, pattern)
    
    def set_led_brightness(self, brightness: int):
        self.led_controller.set_brightness(brightness)
    
    def get_display_status(self) -> Dict[str, Any]:
        return {
            "lcd": self.lcd_controller.get_display_status(),
            "led": self.led_controller.get_led_status(),
            "enabled": self.enabled
        }
    
    def enable_display(self):
        self.enabled = True
        self.lcd_controller.enable_display()
        self.led_controller.enable_led()
        logger.info("Display Manager enabled")
    
    def disable_display(self):
        self.enabled = False
        self.lcd_controller.disable_display()
        self.led_controller.disable_led()
        logger.info("Display Manager disabled")
    
    def clear_display(self):
        self.lcd_controller.clear_display()
        self.led_controller.set_color(LEDColor.OFF)
        logger.info("Display cleared")

def main():
    print("=== Display Manager Test ===")
    
    manager = DisplayManager(16, 2)
    
    def lcd_callback(symbol, message):
        print(f"LCD: {symbol} {message}")
    
    def led_callback(color, pattern, brightness):
        print(f"LED: {color} {pattern} {brightness}%")
    
    manager.set_display_callbacks(lcd_callback, led_callback)
    manager.start_display()
    
    print("Testing different display states...")
    
    manager.set_status("normal", "System Ready")
    import time
    time.sleep(2)
    
    manager.set_status("warning", "Obstacle Detected")
    time.sleep(2)
    
    manager.set_status("error", "Connection Lost")
    time.sleep(2)
    
    manager.set_status("info", "Processing Data")
    time.sleep(2)
    
    manager.set_status("success", "Task Completed")
    time.sleep(2)
    
    manager.stop_display()
    print("Display Manager test completed")

if __name__ == "__main__":
    main()
