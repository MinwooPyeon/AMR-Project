#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import base64
from datetime import datetime
from enum import Enum
from typing import Optional
import paho.mqtt.client as mqtt

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
    def __init__(self, amr_serial: str):
        super().__init__('ai_alert_publisher')
        
        self.amr_serial = amr_serial
        topic_name = f"alert/{amr_serial}"
        
        self.publisher = self.create_publisher(
            String,
            topic_name,
            10
        )
        
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        
        try:
            self.mqtt_client.connect("192.168.100.141", 1883, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT 연결 실패: {e}")
        
        self.get_logger().info(f"AI Alert Publisher 초기화 완료 - Topic: {topic_name}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT 연결 성공: {rc}")

    def publish_alert(self, alert_data: AlertData):
        try:
            json_data = self.create_alert_json(alert_data)
            
            msg = String()
            msg.data = json_data
            
            self.publisher.publish(msg)
            
            topic_name = f"alert/{self.amr_serial}"
            self.mqtt_client.publish(topic_name, json_data)
            
            self.get_logger().info(
                f"Alert 발행 완료 - Situation: {alert_data.situation.value}, "
                f"Position: ({alert_data.x:.2f}, {alert_data.y:.2f})"
            )
            
        except Exception as e:
            self.get_logger().error(f"Alert 발행 실패: {e}")

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
        
        topic_name = f"alert/{amr_serial}"
        self.publisher = self.create_publisher(
            String,
            topic_name,
            10
        )
        
        self.get_logger().info(f"AMR Serial 변경 - 새로운 Topic: {topic_name}")

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
        
        return json.dumps(data, ensure_ascii=False) 