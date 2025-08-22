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

from .config import AIConfig
from .utils import setup_logger, parse_json_message, decode_base64_to_image

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
        
        self.logger = setup_logger("AIPositionSubscriber")
        
        self.subscription = self.create_subscription(
            String,
            AIConfig.ROS_TOPIC_POSITION,
            self.topic_callback,
            10
        )
        
        self.command_callback: Optional[Callable] = None
        self.image_callback: Optional[Callable] = None
        self.situation_callback: Optional[Callable] = None
        self.position_callback: Optional[Callable] = None
        
        os.makedirs(AIConfig.AI_IMAGES_PATH, exist_ok=True)
        
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        self.mqtt_client.username_pw_set("minwoo", "minwoo")
        
        self._setup_mqtt_connection()
        
        self.logger.info("AI Position Subscriber initialized")

    def _setup_mqtt_connection(self):
        mqtt_config = AIConfig.get_mqtt_config()
        try:
            self.mqtt_client.connect(
                mqtt_config["broker"], 
                mqtt_config["port"], 
                mqtt_config["keepalive"]
            )
            self.mqtt_client.subscribe(AIConfig.ROS_TOPIC_POSITION)
            self.mqtt_client.loop_start()
            self.logger.info(f"AI -> Embedded connection: {mqtt_config['broker']}:{mqtt_config['port']}")
        except Exception as e:
            self.logger.error(f"AI MQTT connection failed: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.logger.info("AI MQTT connection successful")
        else:
            self.logger.error(f"AI MQTT connection failed - code: {rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.logger.warning(f"AI MQTT connection disconnected - code: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = parse_json_message(msg.payload.decode())
            if not data:
                return
                
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
                    self.logger.info("AI image saved successfully")
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
                self.command_callback(ai_data.command)
            
            self.logger.info(f"AI data received: {ai_data.command.value}, ({ai_data.x:.2f}, {ai_data.y:.2f})")
            
        except Exception as e:
            self.logger.error(f"MQTT message processing failed: {e}")

    def topic_callback(self, msg):
        try:
            data = parse_json_message(msg.data)
            if not data:
                return
                
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
                    self.logger.info("AI image saved successfully")
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
                self.command_callback(ai_data.command)
            
            self.logger.info(f"ROS data received: {ai_data.command.value}, ({ai_data.x:.2f}, {ai_data.y:.2f})")
            
        except Exception as e:
            self.logger.error(f"ROS topic processing failed: {e}")

    def save_image(self, base64_image: str) -> bool:
        try:
            filepath = decode_base64_to_image(base64_image)
            return filepath is not None
        except Exception as e:
            self.logger.error(f"Image save failed: {e}")
            return False

    def set_command_callback(self, callback: Callable[[AICommand], None]):
        self.command_callback = callback
        self.logger.info("Command callback function set")

    def set_image_callback(self, callback: Callable[[str], None]):
        self.image_callback = callback
        self.logger.info("Image callback function set")

    def set_situation_callback(self, callback: Callable[[str], None]):
        self.situation_callback = callback
        self.logger.info("Situation callback function set")

    def set_position_callback(self, callback: Callable[[float, float], None]):
        self.position_callback = callback
        self.logger.info("Position callback function set")

    def cleanup(self):
        try:
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            self.logger.info("AI Position Subscriber cleanup completed")
        except Exception as e:
            self.logger.error(f"Cleanup error: {e}")

def main():
    print("=== AI Position Subscriber Test ===")
    
    def on_command(command):
        print(f"Command received: {command.value}")
    
    def on_image(image_path):
        print(f"Image saved: {image_path}")
    
    def on_situation(situation):
        print(f"Situation detected: {situation}")
    
    def on_position(x, y):
        print(f"Position updated: ({x:.2f}, {y:.2f})")
    
    rclpy.init()
    subscriber = AIPositionSubscriber()
    
    subscriber.set_command_callback(on_command)
    subscriber.set_image_callback(on_image)
    subscriber.set_situation_callback(on_situation)
    subscriber.set_position_callback(on_position)
    
    try:
        print("AI Position Subscriber started... (Exit: Ctrl+C)")
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        subscriber.cleanup()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 