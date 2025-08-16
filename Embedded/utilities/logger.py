import logging
import sys
from typing import Optional
from datetime import datetime

class AMRLogger:
    def __init__(self, name: str, level: int = logging.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)
        if not self.logger.handlers:
            self._setup_handlers()
    
    def _setup_handlers(self):
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.INFO)
        file_handler = logging.FileHandler(f'amr_{datetime.now().strftime("%Y%m%d")}.log', encoding='utf-8')
        file_handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        console_handler.setFormatter(formatter)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)
        self.logger.addHandler(file_handler)
    
    def info(self, message: str):
        self.logger.info(f"[INFO] {message}")
    
    def warn(self, message: str):
        self.logger.warning(f"[WARN] {message}")
    
    def error(self, message: str):
        self.logger.error(f"[ERROR] {message}")
    
    def debug(self, message: str):
        self.logger.debug(f"[DEBUG] {message}")
    
    def success(self, message: str):
        self.logger.info(f"[SUCCESS] {message}")
    
    def websocket_send_success(self, message: str, data: Optional[dict] = None):
        log_msg = f"[WEBSOCKET_SEND] WebSocket transmission successful: {message}"
        if data: log_msg += f" | Data: {data}"
        self.logger.info(log_msg)
    
    def mqtt_send_success(self, topic: str, data: Optional[dict] = None):
        if data:
            self.logger.info(f"{data}")
        else:
            self.logger.info(f"[MQTT_SEND] MQTT transmission successful: {topic}")
    
    def mqtt_receive_success(self, topic: str, data: Optional[dict] = None):
        log_msg = f"[MQTT_RECEIVE] MQTT reception successful: {topic}"
        if data: log_msg += f" | Data: {data}"
        self.logger.info(log_msg)

def get_logger(name: str) -> AMRLogger:
    return AMRLogger(name)

main_logger = get_logger("AMR_MAIN")
mqtt_logger = get_logger("AMR_MQTT")
ros2_logger = get_logger("AMR_ROS2")
motor_logger = get_logger("AMR_MOTOR")
sensor_logger = get_logger("AMR_SENSOR")
 