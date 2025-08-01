#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI 위치 데이터 구독자
AI에서 전송하는 위치 및 명령 데이터를 수신
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Float64MultiArray, String
import json
import logging
import threading
from typing import Optional, Callable, Dict

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class AIPositionSubscriber(Node):
    """AI 위치 데이터 구독자 클래스"""
    
    def __init__(self):
        super().__init__('ai_position_subscriber')
        
        # 현재 위치 데이터
        self.current_position = {"x": 0.0, "y": 0.0}
        self.position_lock = threading.Lock()
        
        # AI 명령 데이터
        self.current_ai_data = {
            "MOVING_FORWARD": "",
            "ROTATE_LEFT": "",
            "ROTATE_RIGHT": "",
            "MOVING_BACKWARD": "",
            "STOP": "",
            "img": "",
            "situation": "",
            "x": "0.0",
            "y": "0.0"
        }
        self.ai_data_lock = threading.Lock()
        
        # 콜백 함수
        self.position_callback = None
        self.ai_data_callback = None
        
        # 구독자 설정
        self._setup_subscribers()
        
        logger.info("AI Position Subscriber 초기화 완료")
        logger.info("구독 중인 토픽:")
        logger.info("  - /ai/position (Pose2D)")
        logger.info("  - /ai/position_point (Point)")
        logger.info("  - /ai/position_array (Float64MultiArray)")
        logger.info("  - /ai/position_json (String - JSON)")
        logger.info("  - /position (String - AI 명령 데이터)")
    
    def _setup_subscribers(self):
        """구독자 설정"""
        # 기존 위치 데이터 구독자들
        self.pose_subscriber = self.create_subscription(
            Pose2D,
            '/ai/position',
            self._pose_callback,
            10
        )
        
        self.point_subscriber = self.create_subscription(
            Point,
            '/ai/position_point',
            self._point_callback,
            10
        )
        
        self.array_subscriber = self.create_subscription(
            Float64MultiArray,
            '/ai/position_array',
            self._array_callback,
            10
        )
        
        self.json_subscriber = self.create_subscription(
            String,
            '/ai/position_json',
            self._json_callback,
            10
        )
        
        # 새로운 AI 명령 데이터 구독자
        self.ai_command_subscriber = self.create_subscription(
            String,
            '/position',  # AI에서 AI 명령 데이터를 발행하는 토픽
            self._ai_command_callback,
            10
        )
    
    def _pose_callback(self, msg: Pose2D):
        """Pose2D 메시지 콜백"""
        x = msg.x
        y = msg.y
        with self.position_lock:
            self.current_position["x"] = x
            self.current_position["y"] = y
        logger.debug(f"Pose2D 수신: x={x:.2f}, y={y:.2f}")
        if self.position_callback:
            self.position_callback(x, y)
    
    def _point_callback(self, msg: Point):
        """Point 메시지 콜백"""
        x = msg.x
        y = msg.y
        with self.position_lock:
            self.current_position["x"] = x
            self.current_position["y"] = y
        logger.debug(f"Point 수신: x={x:.2f}, y={y:.2f}")
        if self.position_callback:
            self.position_callback(x, y)
    
    def _array_callback(self, msg: Float64MultiArray):
        """Float64MultiArray 메시지 콜백"""
        if len(msg.data) >= 2:
            x = msg.data[0]
            y = msg.data[1]
            with self.position_lock:
                self.current_position["x"] = x
                self.current_position["y"] = y
            logger.debug(f"Float64MultiArray 수신: x={x:.2f}, y={y:.2f}")
            if self.position_callback:
                self.position_callback(x, y)
        else:
            logger.warning(f"Float64MultiArray 데이터 부족: {msg.data}")
    
    def _json_callback(self, msg: String):
        """JSON String 메시지 콜백 - 기존 위치 데이터"""
        try:
            data = json.loads(msg.data)
            if "x" in data and "y" in data:
                x = float(data["x"])
                y = float(data["y"])
                with self.position_lock:
                    self.current_position["x"] = x
                    self.current_position["y"] = y
                logger.debug(f"JSON 수신: x={x:.2f}, y={y:.2f}")
                if "serial" in data:
                    logger.debug(f"Serial: {data['serial']}")
                if self.position_callback:
                    self.position_callback(x, y)
            else:
                logger.warning(f"JSON 데이터에 x, y 필드가 없습니다: {data}")
        except json.JSONDecodeError as e:
            logger.error(f"JSON 파싱 오류: {e}")
        except Exception as e:
            logger.error(f"JSON 처리 오류: {e}")
    
    def _ai_command_callback(self, msg: String):
        """AI 명령 데이터 콜백 - 새로운 구조"""
        try:
            data = json.loads(msg.data)
            with self.ai_data_lock:
                # AI 명령 데이터 업데이트
                self.current_ai_data.update(data)
                
                # x, y 값이 있으면 위치 데이터도 업데이트
                if "x" in data and "y" in data:
                    self.current_position["x"] = float(data["x"])
                    self.current_position["y"] = float(data["y"])
            
            logger.info(f"AI 명령 데이터 수신: {data}")
            
            # AI 데이터 콜백 호출
            if self.ai_data_callback:
                self.ai_data_callback(data)
                
        except json.JSONDecodeError as e:
            logger.error(f"AI 명령 JSON 파싱 오류: {e}")
        except Exception as e:
            logger.error(f"AI 명령 처리 오류: {e}")
    
    def set_position_callback(self, callback: Callable[[float, float], None]):
        """위치 콜백 설정"""
        self.position_callback = callback
    
    def set_ai_data_callback(self, callback: Callable[[Dict], None]):
        """AI 데이터 콜백 설정"""
        self.ai_data_callback = callback
    
    def get_current_position(self) -> tuple:
        """현재 위치 조회"""
        with self.position_lock:
            return self.current_position["x"], self.current_position["y"]
    
    def get_current_ai_data(self) -> Dict:
        """현재 AI 데이터 조회"""
        with self.ai_data_lock:
            return self.current_ai_data.copy()
    
    def get_ai_command(self) -> str:
        """현재 AI 명령 조회"""
        with self.ai_data_lock:
            # 활성화된 명령 찾기
            for command in ["MOVING_FORWARD", "ROTATE_LEFT", "ROTATE_RIGHT", "MOVING_BACKWARD", "STOP"]:
                if self.current_ai_data.get(command):
                    return command
            return "STOP"  # 기본값
    
    def get_ai_situation(self) -> str:
        """현재 상황 조회"""
        with self.ai_data_lock:
            return self.current_ai_data.get("situation", "")
    
    def get_ai_image(self) -> str:
        """현재 이미지 파일명 조회"""
        with self.ai_data_lock:
            return self.current_ai_data.get("img", "")

def main(args=None):
    rclpy.init(args=args)
    subscriber = AIPositionSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 