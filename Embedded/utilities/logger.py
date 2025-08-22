import logging
import sys
from typing import Optional, Dict, Any
from datetime import datetime
from pathlib import Path

class AMRLogger:
    def __init__(self, name: str, level: int = logging.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)
        if not self.logger.handlers:
            self._setup_handlers()
    
    def _setup_handlers(self):
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.INFO)
        
        log_dir = Path('logs')
        log_dir.mkdir(exist_ok=True)
        
        file_handler = logging.FileHandler(
            log_dir / f'amr_{datetime.now().strftime("%Y%m%d")}.log', 
            encoding='utf-8'
        )
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
        if data: 
            log_msg += f" | Data: {data}"
        self.logger.info(log_msg)
    
    def mqtt_send_success(self, topic: str, data: Optional[dict] = None):
        if data:
            self.logger.info(f"{data}")
        else:
            self.logger.info(f"[MQTT_SEND] MQTT transmission successful: {topic}")
    
    def mqtt_receive_success(self, topic: str, data: Optional[dict] = None):
        if data:
            self.logger.info(f"{data}")
        else:
            self.logger.info(f"[MQTT_RECEIVE] MQTT reception successful: {topic}")
    
    def connection_success(self, service: str, details: str = ""):
        self.logger.info(f"[CONNECTION] {service} connection established {details}")
    
    def connection_error(self, service: str, error: str):
        self.logger.error(f"[CONNECTION] {service} connection failed: {error}")
    
    def security_alert(self, alert_type: str, message: str):
        self.logger.warning(f"[SECURITY] {alert_type}: {message}")
    
    def performance_metric(self, metric: str, value: Any):
        self.logger.info(f"[PERFORMANCE] {metric}: {value}")


class LoggerFactory:    
    _loggers: Dict[str, AMRLogger] = {}
    
    @staticmethod
    def get_logger(name: str, level: str = "INFO") -> AMRLogger:

        if name not in LoggerFactory._loggers:
            level_map = {
                "DEBUG": logging.DEBUG,
                "INFO": logging.INFO,
                "WARNING": logging.WARNING,
                "ERROR": logging.ERROR
            }
            log_level = level_map.get(level.upper(), logging.INFO)
            LoggerFactory._loggers[name] = AMRLogger(name, log_level)
        
        return LoggerFactory._loggers[name]
    
    @staticmethod
    def get_module_logger(module_name: str) -> AMRLogger:   
        return LoggerFactory.get_logger(f"amr.{module_name}")


mqtt_logger = LoggerFactory.get_logger("amr.mqtt")
system_logger = LoggerFactory.get_logger("amr.system")
security_logger = LoggerFactory.get_logger("amr.security")
motor_logger = LoggerFactory.get_logger("amr.motor")
sensor_logger = LoggerFactory.get_logger("amr.sensor")
ai_logger = LoggerFactory.get_logger("amr.ai")
 