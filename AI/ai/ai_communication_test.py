#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import time
import base64
import json
import paho.mqtt.client as mqtt
from ai_position_subscriber import AIPositionSubscriber, AICommand
from ai_alert_publisher import AIAlertPublisher

class AITestNode(Node):
    def __init__(self):
        super().__init__('ai_test_node')
        
        self.position_subscriber = AIPositionSubscriber()
        
        self.position_subscriber.set_command_callback(self.handle_ai_command)
        self.position_subscriber.set_image_callback(self.handle_ai_image)
        self.position_subscriber.set_situation_callback(self.handle_ai_situation)
        self.position_subscriber.set_position_callback(self.handle_ai_position)
        
        self.alert_publisher = AIAlertPublisher("AMR001")
        
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect("192.168.100.141", 1883, 60)
            self.mqtt_client.subscribe("position")
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT 연결 실패: {e}")
        
        self.get_logger().info("AI 통신 테스트")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT 연결 성공: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        self.get_logger().info(f"MQTT 메시지 수신: {msg.topic} - {msg.payload.decode()}")

    def run_test(self):
        self.get_logger().info("AI 통신 테스트 시작...")
        
        test_thread = threading.Thread(target=self.test_alert_publishing)
        test_thread.start()
        
        mqtt_test_thread = threading.Thread(target=self.test_mqtt_publishing)
        mqtt_test_thread.start()
        
        rclpy.spin(self.position_subscriber)

    def handle_ai_command(self, data):
        self.get_logger().info("=== AI 명령 수신 ===")
        self.get_logger().info(f"Command: {data.command.value}")
        self.get_logger().info(f"Situation: {data.situation}")
        self.get_logger().info(f"Position: ({data.x:.2f}, {data.y:.2f})")
        
        if data.command == AICommand.MOVING_FORWARD:
            self.get_logger().info("로봇 전진 명령 실행")
        elif data.command == AICommand.ROTATE_LEFT:
            self.get_logger().info("로봇 좌회전 명령 실행")
        elif data.command == AICommand.ROTATE_RIGHT:
            self.get_logger().info("로봇 우회전 명령 실행")
        elif data.command == AICommand.MOVING_BACKWARD:
            self.get_logger().info("로봇 후진 명령 실행")
        elif data.command == AICommand.STOP:
            self.get_logger().info("로봇 정지 명령 실행")
        else:
            self.get_logger().warn("알 수 없는 명령")

    def handle_ai_image(self, image_data):
        self.get_logger().info(f"AI 이미지 수신 - 크기: {len(image_data)} bytes")

    def handle_ai_situation(self, situation):
        self.get_logger().info(f"AI 상황 정보 수신: {situation}")

    def handle_ai_position(self, x, y):
        self.get_logger().info(f"AI 위치 정보 수신: ({x:.2f}, {y:.2f})")

    def test_alert_publishing(self):
        self.get_logger().info("Alert 발행 테스트")
        
        test_image = base64.b64encode(b"dummy_image_data_for_testing").decode('utf-8')
        
        time.sleep(2)
        
        self.get_logger().info("1. COLLAPSE Alert 발행")
        self.alert_publisher.publish_collapse_alert(test_image, 10.5, 20.3, "적재물 붕괴 감지")
        
        time.sleep(3)
        
        self.get_logger().info("2. SMOKE Alert 발행")
        self.alert_publisher.publish_smoke_alert(test_image, 15.2, 8.7, "흡연 행위 감지")
        
        time.sleep(3)
        
        self.get_logger().info("3. EQUIPMENT Alert 발행 테스트")
        self.alert_publisher.publish_equipment_alert(test_image, 5.1, 12.9, "안전장비 미착용")
        
        self.get_logger().info("Alert 발행 테스트 완료")

    def test_mqtt_publishing(self):
        self.get_logger().info("MQTT Position 발행 테스트 시작")
        
        time.sleep(5)
        
        test_data = {
            "MOVING_FORWARD": "",
            "img": "",
            "situation": "normal",
            "x": 10.5,
            "y": 20.3
        }
        
        json_data = json.dumps(test_data)
        self.mqtt_client.publish("position", json_data)
        
        self.get_logger().info("MQTT Position 발행")

def main(args=None):
    rclpy.init(args=args)
    
    test_node = AITestNode()
    test_node.run_test()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main() 