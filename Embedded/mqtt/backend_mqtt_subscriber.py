#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
백엔드 MQTT Subscriber
"""

import json
import time
import threading
from typing import Dict, Optional, Callable, Any
import paho.mqtt.client as mqtt
from utils.logger import mqtt_logger

class BackendMQTTSubscriber:
    def __init__(self, mqtt_broker: str = "192.168.100.141", mqtt_port: int = 1883):
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_client_id = f"backend_subscriber_{int(time.time())}"
        
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_connected = False
        
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_publish = self._on_mqtt_publish
        
        self.latest_amr_data = {}
        self.data_lock = threading.Lock()
        
        self.amr_data_callback: Optional[Callable[[Dict], None]] = None
        self.command_callback: Optional[Callable[[Dict], None]] = None
        
        self.stats_lock = threading.Lock()
        self.total_received = 0
        self.last_received_time = 0
        
        mqtt_logger.success(f"Backend MQTT Subscriber 초기화 완료 - Broker: {mqtt_broker}:{mqtt_port}")
    
    def connect_mqtt(self) -> bool:
        try:
            mqtt_logger.info(f"MQTT 브로커에 연결 중: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            timeout = 10
            start_time = time.time()
            while not self.mqtt_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.mqtt_connected:
                mqtt_logger.success("MQTT 브로커 연결 성공")
                return True
            else:
                mqtt_logger.error("MQTT 연결 시간 초과")
                return False
                
        except Exception as e:
            mqtt_logger.error(f"MQTT 연결 실패: {e}")
            return False
    
    def disconnect_mqtt(self):
        if self.mqtt_connected:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_connected = False
            mqtt_logger.info("MQTT 연결 해제")
    
    def subscribe_to_amr_data(self, robot_id: str = "AMR001"):
        topic = "status"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            mqtt_logger.info(f"AMR 데이터 구독 성공: {topic}")
            return True
        else:
            mqtt_logger.error(f"AMR 데이터 구독 실패: {result[0]}")
            return False
    
    def subscribe_to_commands(self, robot_id: str = "AMR001"):
        topic = f"command/{robot_id}"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            mqtt_logger.info(f"명령 구독 성공: {topic}")
            return True
        else:
            mqtt_logger.error(f"명령 구독 실패: {result[0]}")
            return False
    
    def publish_command(self, robot_id: str, command: Dict[str, Any]) -> bool:
        if not self.mqtt_connected:
            mqtt_logger.warning("MQTT 연결이 없어 명령을 전송할 수 없습니다")
            return False
        
        try:
            command["timestamp"] = time.time()
            command["source"] = "backend"
            
            json_str = json.dumps(command, ensure_ascii=False)
            topic = f"command/{robot_id}"
            
            result = self.mqtt_client.publish(topic, json_str, qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                mqtt_logger.info(f"명령 전송 성공: {command}")
                return True
            else:
                mqtt_logger.error(f"명령 전송 실패: {result.rc}")
                return False
                
        except Exception as e:
            mqtt_logger.error(f"명령 전송 중 오류: {e}")
            return False
    
    def set_amr_data_callback(self, callback: Callable[[Dict], None]):
        self.amr_data_callback = callback
        mqtt_logger.info("AMR 데이터 콜백 설정 완료")
    
    def set_command_callback(self, callback: Callable[[Dict], None]):
        self.command_callback = callback
        mqtt_logger.info("명령 콜백 설정 완료")
    
    def get_latest_amr_data(self) -> Dict:
        with self.data_lock:
            return self.latest_amr_data.copy()
    
    def get_reception_stats(self) -> Dict[str, Any]:
        with self.stats_lock:
            stats = {
                "total_received": self.total_received,
                "last_received_time": self.last_received_time,
                "mqtt_connected": self.mqtt_connected,
                "latest_data": self.get_latest_amr_data()
            }
        return stats
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            mqtt_logger.info(f"MQTT 브로커 연결 성공: {self.mqtt_broker}:{self.mqtt_port}")
        else:
            mqtt_logger.error(f"MQTT 연결 실패. 코드: {rc}")
            self.mqtt_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        if rc != 0:
            mqtt_logger.warning(f"MQTT 연결이 예기치 않게 끊어졌습니다. 코드: {rc}")
        else:
            mqtt_logger.info("MQTT 연결이 정상적으로 해제되었습니다.")
    
    def _on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            mqtt_logger.debug(f"메시지 수신 - 토픽: {topic}, 데이터: {data}")
            
            with self.stats_lock:
                self.total_received += 1
                self.last_received_time = time.time()
            
            mqtt_logger.mqtt_receive_success(topic, data)
            
            if topic == "status":
                with self.data_lock:
                    self.latest_amr_data = data
                
                if self.amr_data_callback:
                    self.amr_data_callback(data)
                    
            elif topic.startswith("command/"):
                if self.command_callback:
                    self.command_callback(data)
            
        except json.JSONDecodeError as e:
            mqtt_logger.error(f"JSON 파싱 오류: {e}")
        except Exception as e:
            mqtt_logger.error(f"메시지 처리 오류: {e}")
    
    def _on_mqtt_publish(self, client, userdata, mid):
        mqtt_logger.debug(f"MQTT 메시지 발행 완료. ID: {mid}")

 