#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI MQTT 클라이언트
AI와 localhost 통신을 위한 MQTT 클라이언트
"""

import json
import time
import threading
from typing import Dict, Optional, Callable, Any
import paho.mqtt.client as mqtt
from utils.logger import mqtt_logger

class AIMQTTClient:
    def __init__(self, robot_id: str = "AMR001", mqtt_broker: str = "localhost", mqtt_port: int = 1883):
        self.robot_id = robot_id
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_client_id = f"ai_client_{robot_id}_{int(time.time())}"
        
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_connected = False
        
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_publish = self._on_mqtt_publish
        
        # 임베디드에서 보내는 데이터 구조 (192.168.100.141:1883)
        self.embedded_data = {
            "serial": "",
            "state": "",
            "x": 0.0,
            "y": 0.0,
            "speed": 0.0,
            "angle": 0.0
        }
        
        # AI에서 받는 데이터 구조 (localhost)
        self.ai_received_data = {
            "serial": "",
            "x": 0.0,
            "y": 0.0,
            "img": "",
            "case": "",
            "timeStamp": ""
        }
        
        self.data_lock = threading.Lock()
        
        # 콜백 함수들
        self.embedded_data_callback: Optional[Callable[[Dict], None]] = None
        self.ai_data_callback: Optional[Callable[[Dict], None]] = None
        
        # 통계
        self.stats_lock = threading.Lock()
        self.total_received = 0
        self.last_received_time = 0
        
        mqtt_logger.success(f"AI MQTT Client 초기화 완료 - Robot ID: {robot_id}, Broker: {mqtt_broker}:{mqtt_port}")
    
    def connect_mqtt(self) -> bool:
        try:
            mqtt_logger.info(f"AI MQTT 브로커에 연결 중: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            timeout = 10
            start_time = time.time()
            while not self.mqtt_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.mqtt_connected:
                mqtt_logger.success("AI MQTT 브로커 연결 성공")
                return True
            else:
                mqtt_logger.error("AI MQTT 연결 시간 초과")
                return False
                
        except Exception as e:
            mqtt_logger.error(f"AI MQTT 연결 실패: {e}")
            return False
    
    def disconnect_mqtt(self):
        if self.mqtt_connected:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_connected = False
            mqtt_logger.info("AI MQTT 연결 해제")
    
    def subscribe_to_embedded_data(self, robot_id: str = "AMR001"):
        """임베디드에서 보내는 데이터 구독"""
        topic = "robot_data"  # 임베디드에서 전송하는 데이터 토픽
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            mqtt_logger.info(f"임베디드 데이터 구독 성공: {topic}")
            return True
        else:
            mqtt_logger.error(f"임베디드 데이터 구독 실패: {result[0]}")
            return False
    
    def subscribe_to_ai_data(self, robot_id: str = "AMR001"):
        """AI에서 받는 데이터 구독"""
        topic = "ai_data"  # AI에서 받는 데이터 토픽
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            mqtt_logger.info(f"AI 데이터 구독 성공: {topic}")
            return True
        else:
            mqtt_logger.error(f"AI 데이터 구독 실패: {result[0]}")
            return False
    
    def set_embedded_data_callback(self, callback: Callable[[Dict], None]):
        """임베디드 데이터 콜백 설정"""
        self.embedded_data_callback = callback
        mqtt_logger.info("임베디드 데이터 콜백 설정 완료")
    
    def set_ai_data_callback(self, callback: Callable[[Dict], None]):
        """AI 데이터 콜백 설정"""
        self.ai_data_callback = callback
        mqtt_logger.info("AI 데이터 콜백 설정 완료")
    
    def get_latest_embedded_data(self) -> Dict:
        """최신 임베디드 데이터 조회"""
        with self.data_lock:
            return self.embedded_data.copy()
    
    def get_latest_ai_data(self) -> Dict:
        """최신 AI 데이터 조회"""
        with self.data_lock:
            return self.ai_received_data.copy()
    
    def get_robot_position(self) -> tuple:
        """로봇 위치 조회"""
        with self.data_lock:
            x = self.embedded_data.get("x", 0.0)
            y = self.embedded_data.get("y", 0.0)
            return (x, y)
    
    def get_robot_state(self) -> str:
        """로봇 상태 조회"""
        with self.data_lock:
            return self.embedded_data.get("state", "")
    
    def get_robot_speed(self) -> float:
        """로봇 속도 조회"""
        with self.data_lock:
            return self.embedded_data.get("speed", 0.0)
    
    def get_robot_angle(self) -> float:
        """로봇 각도 조회"""
        with self.data_lock:
            return self.embedded_data.get("angle", 0.0)
    
    def get_ai_image(self) -> str:
        """AI 이미지 조회"""
        with self.data_lock:
            return self.ai_received_data.get("img", "")
    
    def get_ai_case(self) -> str:
        """AI 케이스 조회"""
        with self.data_lock:
            return self.ai_received_data.get("case", "")
    
    def get_reception_stats(self) -> Dict[str, Any]:
        """수신 통계 조회"""
        with self.stats_lock:
            stats = {
                "total_received": self.total_received,
                "last_received_time": self.last_received_time,
                "mqtt_connected": self.mqtt_connected,
                "latest_embedded_data": self.get_latest_embedded_data(),
                "latest_ai_data": self.get_latest_ai_data()
            }
        return stats
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            mqtt_logger.success(f"AI MQTT 브로커 연결 성공: {self.mqtt_broker}:{self.mqtt_port}")
        else:
            mqtt_logger.error(f"AI MQTT 연결 실패. 코드: {rc}")
            self.mqtt_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        if rc != 0:
            mqtt_logger.warning(f"AI MQTT 연결이 예기치 않게 끊어졌습니다. 코드: {rc}")
        else:
            mqtt_logger.info("AI MQTT 연결이 정상적으로 해제되었습니다.")
    
    def _on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            mqtt_logger.debug(f"AI 메시지 수신 - 토픽: {topic}, 데이터: {data}")
            
            with self.stats_lock:
                self.total_received += 1
                self.last_received_time = time.time()
            
            mqtt_logger.mqtt_receive_success(topic, data)
            
            if topic == "robot_data":
                # 임베디드에서 보내는 데이터 처리
                with self.data_lock:
                    # 필수 필드 확인 및 업데이트
                    required_fields = ["serial", "state", "x", "y", "speed", "angle"]
                    for field in required_fields:
                        if field in data:
                            if field in ["x", "y", "speed", "angle"]:
                                self.embedded_data[field] = float(data[field])
                            else:
                                self.embedded_data[field] = str(data[field])
                
                if self.embedded_data_callback:
                    self.embedded_data_callback(data)
                    
            elif topic == "ai_data":
                # AI에서 받는 데이터 처리
                with self.data_lock:
                    # 필수 필드 확인 및 업데이트
                    required_fields = ["serial", "x", "y", "img", "case", "timeStamp"]
                    for field in required_fields:
                        if field in data:
                            if field in ["x", "y"]:
                                self.ai_received_data[field] = float(data[field])
                            else:
                                self.ai_received_data[field] = str(data[field])
                
                if self.ai_data_callback:
                    self.ai_data_callback(data)
            
        except json.JSONDecodeError as e:
            mqtt_logger.error(f"AI JSON 파싱 오류: {e}")
        except Exception as e:
            mqtt_logger.error(f"AI 메시지 처리 오류: {e}")
    
    def _on_mqtt_publish(self, client, userdata, mid):
        mqtt_logger.debug(f"AI MQTT 메시지 발행 완료. ID: {mid}")

 