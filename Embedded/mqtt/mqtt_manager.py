#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MQTT 매니저
백엔드와 AI 통신을 모두 관리하는 통합 MQTT 매니저
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json
import time
import threading
from typing import Dict, Optional, Callable, Any
from mqtt.sensor_data_transmitter import SensorDataTransmitter
from mqtt.ai_mqtt_client import AIMQTTClient
from utils.logger import mqtt_logger

class MQTTManager:
    def __init__(self, robot_id: str = "AMR001"):
        self.robot_id = robot_id
        
        # 백엔드 통신 (192.168.100.141:1883) - 송신만
        self.backend_transmitter = SensorDataTransmitter(robot_id, "192.168.100.141", 1883)
        
        # AI 통신 (localhost:1883) - 수신만
        self.ai_client = AIMQTTClient(robot_id, "localhost", 1883)
        
        # 통합 데이터
        self.ai_command_data = {}
        self.data_lock = threading.Lock()
        
        # 콜백 함수들
        self.ai_command_callback = None
        
        # 통계
        self.stats_lock = threading.Lock()
        self.backend_sent_count = 0
        self.ai_received_count = 0
        
        mqtt_logger.success(f"MQTT Manager 초기화 완료 - Robot ID: {robot_id}")
        mqtt_logger.info("백엔드 통신: 192.168.100.141:1883 (status 토픽) - 송신만")
        mqtt_logger.info("AI 통신: localhost:1883 (position 토픽) - 수신만")
    
    def connect_all(self) -> bool:
        """모든 MQTT 연결 설정"""
        success_count = 0
        
        # 백엔드 연결 (송신만)
        if self.backend_transmitter.connect_mqtt():
            mqtt_logger.success("백엔드 전송기 연결 성공")
            success_count += 1
        else:
            mqtt_logger.error("백엔드 전송기 연결 실패")
        
        # AI 연결 (수신만)
        if self.ai_client.connect_mqtt():
            mqtt_logger.success("AI 클라이언트 연결 성공")
            success_count += 1
        else:
            mqtt_logger.error("AI 클라이언트 연결 실패")
        
        # 구독 설정
        if success_count > 0:
            self._setup_subscriptions()
        
        return success_count > 0
    
    def disconnect_all(self):
        """모든 MQTT 연결 해제"""
        self.backend_transmitter.disconnect_mqtt()
        self.ai_client.disconnect_mqtt()
        mqtt_logger.info("모든 MQTT 연결 해제 완료")
    
    def _setup_subscriptions(self):
        """구독 설정"""
        # AI 구독 (수신만)
        self.ai_client.subscribe_to_ai_commands(self.robot_id)
        
        # 콜백 설정
        self.ai_client.set_ai_command_callback(self._on_ai_command)
    
    def send_to_backend(self, data: Dict[str, Any]) -> bool:
        """백엔드로 데이터 전송 (status 토픽)"""
        # AI에서 받은 x, y 좌표 사용
        ai_data = self.get_ai_command_data()
        x = ai_data.get("x", "")
        y = ai_data.get("y", "")
        
        # 백엔드 데이터 구성
        backend_data = {
            "serial": self.robot_id,
            "state": data.get("state", "RUNNING"),
            "x": x,
            "y": y,
            "speed": "25"  # 고정값
        }
        
        success = self.backend_transmitter.send_sensor_data(backend_data)
        if success:
            with self.stats_lock:
                self.backend_sent_count += 1
        return success
    
    def set_ai_command_callback(self, callback: Callable[[Dict], None]):
        """AI 명령 콜백 설정"""
        self.ai_command_callback = callback
    
    def _on_ai_command(self, command: Dict):
        """AI 명령 수신 콜백"""
        with self.data_lock:
            self.ai_command_data = command
        with self.stats_lock:
            self.ai_received_count += 1
        
        if self.ai_command_callback:
            self.ai_command_callback(command)
    
    def get_ai_command_data(self) -> Dict:
        """AI 명령 데이터 조회"""
        with self.data_lock:
            return self.ai_command_data.copy()
    
    def get_active_ai_command(self) -> str:
        """현재 활성화된 AI 명령 조회"""
        return self.ai_client.get_active_command()
    
    def get_ai_situation(self) -> str:
        """AI 상황 조회"""
        return self.ai_client.get_ai_situation()
    
    def get_ai_position(self) -> tuple:
        """AI 위치 조회"""
        return self.ai_client.get_ai_position()
    
    def get_ai_image(self) -> str:
        """AI 이미지 조회"""
        return self.ai_client.get_ai_image()
    
    def get_stats(self) -> Dict[str, Any]:
        """통계 정보 조회"""
        with self.stats_lock:
            return {
                "backend_sent": self.backend_sent_count,
                "ai_received": self.ai_received_count,
                "backend_connected": self.backend_transmitter.connected,
                "ai_connected": self.ai_client.mqtt_connected
            }
    
    def get_connection_status(self) -> Dict[str, bool]:
        """연결 상태 조회"""
        return {
            "backend_transmitter": self.backend_transmitter.connected,
            "ai_client": self.ai_client.mqtt_connected
        }

 