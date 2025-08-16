import os
from typing import Dict, Any

class DisplayConfig:
    DEFAULT_LCD_WIDTH = 16
    DEFAULT_LCD_HEIGHT = 2
    DEFAULT_UPDATE_INTERVAL = 0.5
    
    DISPLAY_MODES = {
        "normal": "OK",
        "warning": "WARN",
        "error": "ERR",
        "info": "INFO",
        "success": "OK"
    }
    
    MESSAGES = {
        "normal": "AMR Ready",
        "warning": "DANGER!",
        "error": "ERROR!",
        "info": "INFO",
        "success": "SUCCESS"
    }
    
    COLORS = {
        "normal": "green",
        "warning": "yellow",
        "error": "red",
        "info": "blue",
        "success": "green"
    }
    
    @classmethod
    def get_lcd_config(cls) -> Dict[str, Any]:
        return {
            "width": int(os.getenv("LCD_WIDTH", cls.DEFAULT_LCD_WIDTH)),
            "height": int(os.getenv("LCD_HEIGHT", cls.DEFAULT_LCD_HEIGHT)),
            "update_interval": float(os.getenv("LCD_UPDATE_INTERVAL", cls.DEFAULT_UPDATE_INTERVAL))
        }
    
    @classmethod
    def get_display_mode(cls, mode_name: str) -> str:
        return cls.DISPLAY_MODES.get(mode_name, "INFO")
    
    @classmethod
    def get_message(cls, mode_name: str) -> str:
        return cls.MESSAGES.get(mode_name, "INFO")
    
    @classmethod
    def get_color(cls, mode_name: str) -> str:
        return cls.COLORS.get(mode_name, "white")
