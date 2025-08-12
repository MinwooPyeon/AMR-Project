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
        
        # Embedded -> Backend 데이터 구조
        self.embedded_data = {
            "serial": robot_id,
            "state": "IDLE",
            "x": 0.0,
            "y": 0.0,
            "speed": 0.0,
            "angle": 0.0
        }
        
        self.data_lock = threading.Lock()
        
        self.stats_lock = threading.Lock()
        self.total_sent = 0
        self.last_sent_time = 0
        
        self.sensors = {}
        
        # 주기적 전송 관련 변수
        self.periodic_sending = False
        self.periodic_thread = None
        self.send_interval = 1.0  # 1초 간격
        
        # 초기화 완료
    
    def connect_mqtt(self) -> bool:
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            timeout = 10
            start_time = time.time()
            while not self.connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.connected:
                return True
            else:
                return False
                
        except Exception as e:
            return False
    
    def disconnect_mqtt(self):
        if self.connected:
            # 주기적 전송 중지
            self.stop_periodic_sending()
            
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.connected = False
    
    def update_embedded_data(self, serial: str = None, state: str = None, x: float = None, 
                           y: float = None, speed: float = None, angle: float = None):
        """Embedded 데이터 업데이트"""
        with self.data_lock:
            if serial is not None:
                self.embedded_data["serial"] = str(serial)
            if state is not None:
                self.embedded_data["state"] = str(state)
            if x is not None:
                self.embedded_data["x"] = float(x)
            if y is not None:
                self.embedded_data["y"] = float(y)
            if speed is not None:
                self.embedded_data["speed"] = float(speed)
            if angle is not None:
                self.embedded_data["angle"] = float(angle)
    
    def send_embedded_data(self) -> bool:
        """Embedded 데이터를 Backend로 전송"""
        if not self.connected:
            return False
        
        try:
            topic = "status"  
            
            with self.data_lock:
                json_data = json.dumps(self.embedded_data, ensure_ascii=False)
            
            result = self.mqtt_client.publish(topic, json_data, qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                with self.stats_lock:
                    self.total_sent += 1
                    self.last_sent_time = time.time()
                
                mqtt_logger.mqtt_send_success(topic, self.embedded_data)
                
                if self.send_success_callback:
                    self.send_success_callback(topic, self.embedded_data)
                
                return True
            else:
                return False
                
        except Exception as e:
            return False
    
    def send_sensor_data(self, data: Dict) -> bool:
        """기존 센서 데이터 전송 (하위 호환성)"""
        return self.send_embedded_data()
    
    def get_embedded_data(self) -> Dict:
        """현재 Embedded 데이터 조회"""
        with self.data_lock:
            return self.embedded_data.copy()
    
    def get_transmission_stats(self) -> Dict:
        with self.stats_lock:
            return {
                "total_sent": self.total_sent,
                "last_sent_time": self.last_sent_time,
                "connected": self.connected,
                "current_data": self.get_embedded_data()
            }
    
    def set_send_success_callback(self, callback: Callable[[str, Dict], None]):
        self.send_success_callback = callback
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
        else:
            pass
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.connected = False
    
    def start_periodic_sending(self, interval: float = 1.0) -> bool:
        """주기적 데이터 전송 시작"""
        if self.periodic_sending:
            return False
        
        if not self.connected:
            return False
        
        self.send_interval = interval
        self.periodic_sending = True
        self.periodic_thread = threading.Thread(target=self._periodic_send_loop, daemon=True)
        self.periodic_thread.start()
        
        return True
    
    def stop_periodic_sending(self):
        """주기적 데이터 전송 중지"""
        if not self.periodic_sending:
            return
        
        self.periodic_sending = False
        if self.periodic_thread and self.periodic_thread.is_alive():
            self.periodic_thread.join(timeout=2.0)
    
    def _periodic_send_loop(self):
        """주기적 전송 루프"""
        while self.periodic_sending and self.connected:
            try:
                self.send_embedded_data()
                time.sleep(self.send_interval)
            except Exception as e:
                time.sleep(self.send_interval)
    
    def _on_mqtt_publish(self, client, userdata, mid):
        pass

 