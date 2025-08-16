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
            self.mqtt_client.connect("localhost", 1883, 60)
                    self.mqtt_client.subscribe("robot_data")
        self.mqtt_client.subscribe("ai_data")
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT 연결 실패: {e}")
        
        self.embedded_data = {
            "serial": "",
            "state": "",
            "x": 0.0,
            "y": 0.0,
            "speed": 0.0,
            "angle": 0.0
        }
        
        self.ai_received_data = {
            "serial": "",
            "x": 0.0,
            "y": 0.0,
            "img": "",
            "case": "",
            "timeStamp": ""
        }
        
        self.get_logger().info("AI 통신 테스트")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT 연결 성공: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            self.get_logger().info(f"MQTT 메시지 수신: {topic}")
            
            if topic == "robot_data":
                self.handle_embedded_data(data)
            elif topic == "ai_data":
                self.handle_ai_received_data(data)
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON 파싱 오류: {e}")
        except Exception as e:
            self.get_logger().error(f"메시지 처리 오류: {e}")

    def handle_embedded_data(self, data):
        self.get_logger().info("=== 임베디드 데이터 수신 ===")
        self.get_logger().info(f"Serial: {data.get('serial', 'N/A')}")
        self.get_logger().info(f"State: {data.get('state', 'N/A')}")
        self.get_logger().info(f"Position: ({data.get('x', 0)}, {data.get('y', 0)})")
        self.get_logger().info(f"Speed: {data.get('speed', 0)}")
        self.get_logger().info(f"Angle: {data.get('angle', 0)}")
        
        self.embedded_data.update(data)

    def handle_ai_received_data(self, data):
        self.get_logger().info("=== AI 데이터 수신 ===")
        self.get_logger().info(f"Serial: {data.get('serial', 'N/A')}")
        self.get_logger().info(f"Position: ({data.get('x', 0)}, {data.get('y', 0)})")
        self.get_logger().info(f"Image: {len(data.get('img', ''))} bytes")
        self.get_logger().info(f"Case: {data.get('case', 'N/A')}")
        self.get_logger().info(f"Timestamp: {data.get('timeStamp', 'N/A')}")
        
        self.ai_received_data.update(data)

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
        self.get_logger().info("MQTT 데이터 발행 테스트 시작")
        
        time.sleep(5)
        
        embedded_test_data = {
            "serial": "AMR001",
            "state": "moving",
            "x": 10.5,
            "y": 20.3,
            "speed": 1.5,
            "angle": 45.0
        }
        
        json_data = json.dumps(embedded_test_data)
        self.mqtt_client.publish("robot_data", json_data)
        self.get_logger().info("임베디드 데이터 발행 완료")
        
        time.sleep(2)
        
        ai_test_data = {
            "serial": "AMR001",
            "x": 15.2,
            "y": 8.7,
            "img": base64.b64encode(b"test_image_data").decode('utf-8'),
            "case": "normal",
            "timeStamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        json_data = json.dumps(ai_test_data)
        self.mqtt_client.publish("ai_data", json_data)
        self.get_logger().info("AI 데이터 발행 완료")

def main(args=None):
    rclpy.init(args=args)
    
    test_node = AITestNode()
    test_node.run_test()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main() 