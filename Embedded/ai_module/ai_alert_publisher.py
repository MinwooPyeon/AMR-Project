import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import base64
from datetime import datetime
from enum import Enum
from typing import Optional
import paho.mqtt.client as mqtt

from .config import AIConfig
from .utils import setup_logger, create_json_message

class AlertSituation(Enum):
    COLLAPSE = "COLLAPSE"    
    SMOKE = "SMOKE"          
    EQUIPMENT = "EQUIPMENT"  
    UNKNOWN = "UNKNOWN"

class AlertData:
    def __init__(self):
        self.situation: AlertSituation = AlertSituation.UNKNOWN
        self.image: str = ""
        self.x: float = 0.0
        self.y: float = 0.0
        self.detail: str = ""
        self.amr_serial: str = ""

class AIAlertPublisher(Node):
    def __init__(self, amr_serial: str = None):
        super().__init__('ai_alert_publisher')
        
        self.amr_serial = amr_serial or AIConfig.DEFAULT_ROBOT_ID
        self.logger = setup_logger("AIAlertPublisher")
        
        topic_name = f"{AIConfig.ROS_TOPIC_ALERT}/{self.amr_serial}"
        
        self.publisher = self.create_publisher(
            String,
            topic_name,
            10
        )
        
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        self._setup_mqtt_connection()
        
        self.logger.info(f"AI Alert Publisher initialized - Topic: {topic_name}")

    def _setup_mqtt_connection(self):
        mqtt_config = AIConfig.get_mqtt_config()
        try:
            self.mqtt_client.connect(
                mqtt_config["broker"], 
                mqtt_config["port"], 
                mqtt_config["keepalive"]
            )
            self.mqtt_client.loop_start()
            self.logger.info(f"AI -> Embedded connection: {mqtt_config['broker']}:{mqtt_config['port']}")
        except Exception as e:
            self.logger.error(f"MQTT connection failed: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.logger.info("AI MQTT connection successful")
        else:
            self.logger.error(f"AI MQTT connection failed - code: {rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.logger.warning(f"AI MQTT connection disconnected - code: {rc}")

    def publish_alert(self, alert_data: AlertData):
        try:
            json_data = self.create_alert_json(alert_data)
            
            msg = String()
            msg.data = json_data
            
            self.publisher.publish(msg)
            
            topic_name = f"{AIConfig.ROS_TOPIC_ALERT}/{self.amr_serial}"
            self.mqtt_client.publish(topic_name, json_data)
            
            self.logger.info(
                f"Alert published successfully - Situation: {alert_data.situation.value}, "
                f"Position: ({alert_data.x:.2f}, {alert_data.y:.2f})"
            )
            
        except Exception as e:
            self.logger.error(f"Alert publishing failed: {e}")

    def publish_collapse_alert(self, image: str, x: float, y: float, detail: str = ""):
        alert_data = AlertData()
        alert_data.situation = AlertSituation.COLLAPSE
        alert_data.image = image
        alert_data.x = x
        alert_data.y = y
        alert_data.detail = detail
        alert_data.amr_serial = self.amr_serial
        
        self.publish_alert(alert_data)

    def publish_smoke_alert(self, image: str, x: float, y: float, detail: str = ""):
        alert_data = AlertData()
        alert_data.situation = AlertSituation.SMOKE
        alert_data.image = image
        alert_data.x = x
        alert_data.y = y
        alert_data.detail = detail
        alert_data.amr_serial = self.amr_serial
        
        self.publish_alert(alert_data)

    def publish_equipment_alert(self, image: str, x: float, y: float, detail: str = ""):
        alert_data = AlertData()
        alert_data.situation = AlertSituation.EQUIPMENT
        alert_data.image = image
        alert_data.x = x
        alert_data.y = y
        alert_data.detail = detail
        alert_data.amr_serial = self.amr_serial
        
        self.publish_alert(alert_data)

    def set_amr_serial(self, amr_serial: str):
        self.amr_serial = amr_serial
        
        topic_name = f"{AIConfig.ROS_TOPIC_ALERT}/{amr_serial}"
        self.publisher = self.create_publisher(
            String,
            topic_name,
            10
        )
        
        self.logger.info(f"AMR Serial changed - new Topic: {topic_name}")

    def create_alert_json(self, alert_data: AlertData) -> str:
        data = {
            "timestamp": int(datetime.now().timestamp()),
            "situation": alert_data.situation.value,
            "x": alert_data.x,
            "y": alert_data.y,
            "amr_serial": alert_data.amr_serial
        }
        
        if alert_data.image:
            data["image"] = alert_data.image
        
        if alert_data.detail:
            data["detail"] = alert_data.detail
        
        return create_json_message(data)

    def cleanup(self):
        try:
            if self.mqtt_client:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            self.logger.info("AI Alert Publisher cleanup completed")
        except Exception as e:
            self.logger.error(f"Cleanup error: {e}") 