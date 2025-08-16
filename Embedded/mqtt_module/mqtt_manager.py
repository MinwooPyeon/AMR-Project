"""
MQTT Manager
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
        
        self.backend_transmitter = SensorDataTransmitter(robot_id, "192.168.100.141", 1883)
        
        self.ai_client = AIMQTTClient(robot_id, "localhost", 1883)
        
        self.ai_command_data = {}
        self.data_lock = threading.Lock()
        
        self.ai_command_callback = None
        
        self.stats_lock = threading.Lock()
        self.backend_sent_count = 0
        self.ai_received_count = 0
        
        mqtt_logger.success(f"MQTT Manager initialization completed - Robot ID: {robot_id}")
        mqtt_logger.info("Backend communication: 192.168.100.141:1883 (status topic) - send only")
        mqtt_logger.info("AI communication: localhost:1883 (position topic) - receive only")
    
    def connect_all(self) -> bool:
        success_count = 0
        
        if self.backend_transmitter.connect_mqtt():
            mqtt_logger.success("Backend transmitter connection successful")
            success_count += 1
        else:
            mqtt_logger.error("Backend transmitter connection failed")
        
        if self.ai_client.connect_mqtt():
            mqtt_logger.success("AI client connection successful")
            success_count += 1
        else:
            mqtt_logger.error("AI client connection failed")
        
        if success_count > 0:
            self._setup_subscriptions()
        
        return success_count > 0
    
    def disconnect_all(self):
        self.backend_transmitter.disconnect_mqtt()
        self.ai_client.disconnect_mqtt()
        mqtt_logger.info("All MQTT connections disconnected")
    
    def _setup_subscriptions(self):
        self.ai_client.subscribe_to_ai_commands(self.robot_id)
        self.ai_client.set_ai_command_callback(self._on_ai_command)
    
    def send_to_backend(self, data: Dict[str, Any]) -> bool:
        ai_data = self.get_ai_command_data()
        x = ai_data.get("x", "")
        y = ai_data.get("y", "")
        
        backend_data = {
            "serial": self.robot_id,
            "state": data.get("state", "RUNNING"),
            "x": x,
            "y": y,
            "speed": "25" 
        }
        
        success = self.backend_transmitter.send_sensor_data(backend_data)
        if success:
            with self.stats_lock:
                self.backend_sent_count += 1
        return success
    
    def set_ai_command_callback(self, callback: Callable[[Dict], None]):
        self.ai_command_callback = callback
    
    def _on_ai_command(self, command: Dict):
        with self.data_lock:
            self.ai_command_data = command
        with self.stats_lock:
            self.ai_received_count += 1
        
        if self.ai_command_callback:
            self.ai_command_callback(command)
    
    def get_ai_command_data(self) -> Dict:
        with self.data_lock:
            return self.ai_command_data.copy()
    
    def get_active_ai_command(self) -> str:
        return self.ai_client.get_active_command()
    
    def get_ai_situation(self) -> str:
        return self.ai_client.get_ai_situation()
    
    def get_ai_position(self) -> tuple:
        return self.ai_client.get_ai_position()
    
    def get_ai_image(self) -> str:
        return self.ai_client.get_ai_image()
    
    def get_stats(self) -> Dict[str, Any]:
        with self.stats_lock:
            return {
                "backend_sent": self.backend_sent_count,
                "ai_received": self.ai_received_count,
                "backend_connected": self.backend_transmitter.connected,
                "ai_connected": self.ai_client.mqtt_connected
            }
    
    def get_connection_status(self) -> Dict[str, bool]:
        return {
            "backend_transmitter": self.backend_transmitter.connected,
            "ai_client": self.ai_client.mqtt_connected
        }
