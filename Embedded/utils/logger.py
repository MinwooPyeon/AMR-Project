#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
공통 로거 유틸리티
AMR 시스템의 통일된 로그 스타일을 제공
"""
import logging
import sys
from typing import Optional
from datetime import datetime


class AMRLogger:
    """AMR 시스템 전용 로거 클래스"""
    
    def __init__(self, name: str, level: int = logging.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)
        
        # 이미 핸들러가 설정되어 있지 않은 경우에만 설정
        if not self.logger.handlers:
            self._setup_handlers()
    
    def _setup_handlers(self):
        """로거 핸들러 설정"""
        # 콘솔 핸들러
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.INFO)
        
        # 파일 핸들러
        file_handler = logging.FileHandler(f'amr_{datetime.now().strftime("%Y%m%d")}.log', encoding='utf-8')
        file_handler.setLevel(logging.DEBUG)
        
        # 포맷터 설정
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        console_handler.setFormatter(formatter)
        file_handler.setFormatter(formatter)
        
        self.logger.addHandler(console_handler)
        self.logger.addHandler(file_handler)
    
    def info(self, message: str):
        """INFO 레벨 로그"""
        self.logger.info(f"[INFO] {message}")
    
    def warn(self, message: str):
        """WARN 레벨 로그 (통일된 스타일)"""
        self.logger.warning(f"[WARN] {message}")
    
    def error(self, message: str):
        """ERROR 레벨 로그 (통일된 스타일)"""
        self.logger.error(f"[ERROR] {message}")
    
    def debug(self, message: str):
        """DEBUG 레벨 로그"""
        self.logger.debug(f"[DEBUG] {message}")
    
    def success(self, message: str):
        """성공 로그 (INFO 레벨)"""
        self.logger.info(f"[SUCCESS] {message}")
    
    def websocket_send_success(self, message: str, data: Optional[dict] = None):
        """WebSocket 송신 성공 로그"""
        log_msg = f"[WEBSOCKET_SEND] WebSocket 송신 성공: {message}"
        if data:
            log_msg += f" | 데이터: {data}"
        self.logger.info(log_msg)
    
    def mqtt_send_success(self, topic: str, data: Optional[dict] = None):
        """MQTT 송신 성공 로그"""
        log_msg = f"[MQTT_SEND] MQTT 송신 성공: {topic}"
        if data:
            log_msg += f" | 데이터: {data}"
        self.logger.info(log_msg)
    
    def mqtt_receive_success(self, topic: str, data: Optional[dict] = None):
        """MQTT 수신 성공 로그"""
        log_msg = f"[MQTT_RECEIVE] MQTT 수신 성공: {topic}"
        if data:
            log_msg += f" | 데이터: {data}"
        self.logger.info(log_msg)


def get_logger(name: str) -> AMRLogger:
    """로거 인스턴스 생성"""
    return AMRLogger(name)


# 전역 로거 인스턴스
main_logger = get_logger("AMR_MAIN")
mqtt_logger = get_logger("AMR_MQTT")
ros2_logger = get_logger("AMR_ROS2")
motor_logger = get_logger("AMR_MOTOR")
sensor_logger = get_logger("AMR_SENSOR")
display_logger = get_logger("AMR_DISPLAY") 