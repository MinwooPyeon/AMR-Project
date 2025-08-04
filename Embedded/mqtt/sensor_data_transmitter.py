#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
센서 데이터 전송기
"""

import json
import time
import threading
from typing import Dict, Optional, Callable
import paho.mqtt.client as mqtt
from utils.logger import mqtt_logger

class SensorDataTransmitter:
    def __init__(self, robot_id: str = "AMR001", mqtt_broker: str = "192.168.100.141", mqtt_port: int = 1883):
        self.robot_id = robot_id
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        
        self.mqtt_client = mqtt.Client(client_id=f"sensor_transmitter_{robot_id}")
        self.connected = False
        
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_publish = self._on_mqtt_publish
        
        self.send_success_callback = None
        
        self.sensor_data = {}
        self.data_lock = threading.Lock()
        
        self.stats_lock = threading.Lock()
        self.total_sent = 0
        self.last_sent_time = 0
        
        self.sensors = {}
        
        mqtt_logger.success(f"Sensor Data Transmitter 초기화 완료 - Robot ID: {robot_id}")
    
    def connect_mqtt(self) -> bool:
        try:
            mqtt_logger.info(f"MQTT 브로커에 연결 중: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            timeout = 10
            start_time = time.time()
            while not self.connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.connected:
                mqtt_logger.success("MQTT 브로커 연결 성공")
                return True
            else:
                mqtt_logger.error("MQTT 연결 시간 초과")
                return False
                
        except Exception as e:
            mqtt_logger.error(f"MQTT 연결 실패: {e}")
            return False
    
    def disconnect_mqtt(self):
        if self.connected:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.connected = False
            mqtt_logger.info("MQTT 연결 해제")
    
    def register_sensor(self, sensor_name: str, sensor_data: Dict):
        with self.data_lock:
            self.sensors[sensor_name] = sensor_data
        mqtt_logger.info(f"센서 등록: {sensor_name}")
    
    def update_sensor_data(self, sensor_name: str, data: Dict):
        with self.data_lock:
            if sensor_name in self.sensors:
                self.sensors[sensor_name].update(data)
            else:
                self.sensors[sensor_name] = data
    
    def send_sensor_data(self, data: Dict) -> bool:
        if not self.connected:
            mqtt_logger.warn("MQTT가 연결되지 않아 데이터 전송 불가")
            return False
        
        try:
            topic = "status"
            
            json_data = json.dumps(data, ensure_ascii=False)
            
            result = self.mqtt_client.publish(topic, json_data, qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                with self.stats_lock:
                    self.total_sent += 1
                    self.last_sent_time = time.time()
                
                mqtt_logger.mqtt_send_success(topic, data)
                
                if self.send_success_callback:
                    self.send_success_callback(topic, data)
                
                return True
            else:
                mqtt_logger.error(f"센서 데이터 전송 실패: {result.rc}")
                return False
                
        except Exception as e:
            mqtt_logger.error(f"센서 데이터 전송 오류: {e}")
            return False
    
    def get_transmission_stats(self) -> Dict:
        with self.stats_lock:
            return {
                "total_sent": self.total_sent,
                "last_sent_time": self.last_sent_time,
                "connected": self.connected
            }
    
    def set_send_success_callback(self, callback: Callable[[str, Dict], None]):
        self.send_success_callback = callback
        mqtt_logger.info("송신 성공 콜백 설정 완료")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            mqtt_logger.success("MQTT 브로커에 연결됨")
        else:
            mqtt_logger.error(f"MQTT 연결 실패: {rc}")
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.connected = False
        mqtt_logger.warn("MQTT 브로커 연결 해제")
    
    def _on_mqtt_publish(self, client, userdata, mid):
        mqtt_logger.debug(f"MQTT 메시지 발행 완료: {mid}")

def test_sensor_data_transmitter():
    transmitter = SensorDataTransmitter("AMR001")
    
    if transmitter.connect_mqtt():
        test_data = {
            "serial": "AMR001",
            "state": "RUNNING",
            "x": "10.5",
            "y": "20.3",
            "speed": "25.0"
        }
        
        success = transmitter.send_sensor_data(test_data)
        if success:
            print("센서 데이터 전송 테스트 성공")
        else:
            print("센서 데이터 전송 테스트 실패")
        
        transmitter.disconnect_mqtt()

if __name__ == "__main__":
    test_sensor_data_transmitter() 