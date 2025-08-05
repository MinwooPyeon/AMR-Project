#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
import base64
from datetime import datetime
from enum import Enum
from typing import Optional, Callable
import logging
import paho.mqtt.client as mqtt

class AICommand(Enum):
    MOVING_FORWARD = "MOVING_FORWARD"
    ROTATE_LEFT = "ROTATE_LEFT"
    ROTATE_RIGHT = "ROTATE_RIGHT"
    MOVING_BACKWARD = "MOVING_BACKWARD"
    STOP = "STOP"
    UNKNOWN = "UNKNOWN"

class AIPositionData:
    def __init__(self):
        self.command: AICommand = AICommand.UNKNOWN
        self.image: str = ""
        self.situation: str = ""
        self.x: float = 0.0
        self.y: float = 0.0

class AIPositionSubscriber(Node):
    def __init__(self):
        super().__init__('ai_position_subscriber')
        
        self.subscription = self.create_subscription(
            String,
            'position',
            self.topic_callback,
            10
        )
        
        self.command_callback: Optional[Callable] = None
        self.image_callback: Optional[Callable] = None
        self.situation_callback: Optional[Callable] = None
        self.position_callback: Optional[Callable] = None
        
        self.image_save_path = "ai_images"
        os.makedirs(self.image_save_path, exist_ok=True)
        
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect("192.168.100.141", 1883, 60)
            self.mqtt_client.subscribe("position")
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT 연결 실패: {e}")
        
        self.get_logger().info("AI Position Subscriber 초기화 완료")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT 연결 성공: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            ai_data = AIPositionData()
            
            if "MOVING_FORWARD" in data and data["MOVING_FORWARD"]:
                ai_data.command = AICommand.MOVING_FORWARD
            elif "ROTATE_LEFT" in data and data["ROTATE_LEFT"]:
                ai_data.command = AICommand.ROTATE_LEFT
            elif "ROTATE_RIGHT" in data and data["ROTATE_RIGHT"]:
                ai_data.command = AICommand.ROTATE_RIGHT
            elif "MOVING_BACKWARD" in data and data["MOVING_BACKWARD"]:
                ai_data.command = AICommand.MOVING_BACKWARD
            elif "STOP" in data and data["STOP"]:
                ai_data.command = AICommand.STOP
            else:
                ai_data.command = AICommand.UNKNOWN
            
            if "img" in data and data["img"]:
                ai_data.image = data["img"]
                if self.save_image(ai_data.image):
                    self.get_logger().info("AI 이미지 저장 완료")
                    if self.image_callback:
                        self.image_callback(ai_data.image)
            
            if "situation" in data:
                ai_data.situation = data["situation"]
                if ai_data.situation and self.situation_callback:
                    self.situation_callback(ai_data.situation)
            
            if "x" in data and "y" in data:
                ai_data.x = float(data["x"])
                ai_data.y = float(data["y"])
                if self.position_callback:
                    self.position_callback(ai_data.x, ai_data.y)
            
            if self.command_callback:
                self.command_callback(ai_data)
            
            self.get_logger().info(
                f"AI Position 데이터 수신 완료 - Command: {ai_data.command.value}, "
                f"Situation: {ai_data.situation}, Position: ({ai_data.x:.2f}, {ai_data.y:.2f})"
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 파싱 실패: {e}")
        except Exception as e:
            self.get_logger().error(f"AI Position 데이터 처리 중 오류: {e}")

    def topic_callback(self, msg):
        try:
            data = json.loads(msg.data)
            ai_data = AIPositionData()
            
            if "MOVING_FORWARD" in data and data["MOVING_FORWARD"]:
                ai_data.command = AICommand.MOVING_FORWARD
            elif "ROTATE_LEFT" in data and data["ROTATE_LEFT"]:
                ai_data.command = AICommand.ROTATE_LEFT
            elif "ROTATE_RIGHT" in data and data["ROTATE_RIGHT"]:
                ai_data.command = AICommand.ROTATE_RIGHT
            elif "MOVING_BACKWARD" in data and data["MOVING_BACKWARD"]:
                ai_data.command = AICommand.MOVING_BACKWARD
            elif "STOP" in data and data["STOP"]:
                ai_data.command = AICommand.STOP
            else:
                ai_data.command = AICommand.UNKNOWN
            
            if "img" in data and data["img"]:
                ai_data.image = data["img"]
                if self.save_image(ai_data.image):
                    self.get_logger().info("AI 이미지 저장 완료")
                    if self.image_callback:
                        self.image_callback(ai_data.image)
            
            if "situation" in data:
                ai_data.situation = data["situation"]
                if ai_data.situation and self.situation_callback:
                    self.situation_callback(ai_data.situation)
            
            if "x" in data and "y" in data:
                ai_data.x = float(data["x"])
                ai_data.y = float(data["y"])
                if self.position_callback:
                    self.position_callback(ai_data.x, ai_data.y)
            
            if self.command_callback:
                self.command_callback(ai_data)
            
            self.get_logger().info(
                f"AI Position 데이터 수신 완료 - Command: {ai_data.command.value}, "
                f"Situation: {ai_data.situation}, Position: ({ai_data.x:.2f}, {ai_data.y:.2f})"
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 파싱 실패: {e}")
        except Exception as e:
            self.get_logger().error(f"AI Position 데이터 처리 중 오류: {e}")

    def save_image(self, image_data: str) -> bool:
        try:
            image_bytes = base64.b64decode(image_data)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"ai_image_{timestamp}.jpg"
            filepath = os.path.join(self.image_save_path, filename)
            
            with open(filepath, 'wb') as f:
                f.write(image_bytes)
            
            return True
        except Exception as e:
            self.get_logger().error(f"이미지 저장 실패: {e}")
            return False

    def set_command_callback(self, callback: Callable[[AIPositionData], None]):
        self.command_callback = callback

    def set_image_callback(self, callback: Callable[[str], None]):
        self.image_callback = callback

    def set_situation_callback(self, callback: Callable[[str], None]):
        self.situation_callback = callback

    def set_position_callback(self, callback: Callable[[float, float], None]):
        self.position_callback = callback 