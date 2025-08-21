import cv2
import time
import json
import base64
import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from ultralytics import YOLO
import threading


robot_pose = {'x': None, 'y': None}

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            qos)

    def listener_callback(self, msg):
        robot_pose['x'] = msg.pose.pose.position.x
        robot_pose['y'] = msg.pose.pose.position.y

def start_ros2_node():
    rclpy.init()
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# 스레드로 실행
# ros_thread = threading.Thread(target=start_ros2_node, daemon=True)
# ros_thread.start()

def send_alert_to_backend(situation, image_base64="", x=None, y=None, detail=""):
    """
    AI에서 Backend로 알림 전송
    
    Args:
        situation (str): 상황 ("COLLAPSE", "SMOKE", "EQUIPMENT")
        image_base64 (str): Base64로 인코딩된 이미지
        x (str): X 좌표 또는 구역명
        y (str): Y 좌표 또는 구역명
        detail (str): 상세 정보
    """
    if x is None or y is None:
        x = robot_pose['x'] if robot_pose['x'] is not None else 0.0
        y = robot_pose['y'] if robot_pose['y'] is not None else 0.0

    try:
        print(f"[AI -> Backend] x: {x}, y: {y}")
        return True
    except Exception as e:
        print(f"좌표 특정 실패: {e}")
        return False

