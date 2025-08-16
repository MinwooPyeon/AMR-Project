import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Float64MultiArray, String
import json
import logging
import threading
from typing import Optional, Callable, Dict

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class AIPositionSubscriber(Node):
    
    def __init__(self):
        super().__init__('ai_position_subscriber')
        
        self.current_position = {"x": 0.0, "y": 0.0}
        self.position_lock = threading.Lock()
        
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
        
        self.position_callback = None
        self.ai_data_callback = None
        
        self._setup_subscribers()
        
        logger.info("AI Position Subscriber initialized")
        logger.info("Subscribed topics:")
        logger.info("  - /ai/position (Pose2D)")
        logger.info("  - /ai/position_point (Point)")
        logger.info("  - /ai/position_array (Float64MultiArray)")
        logger.info("  - /ai/position_json (String - JSON)")
        logger.info("  - /position (String - AI command data)")
    
    def _setup_subscribers(self):
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
        
        self.ai_command_subscriber = self.create_subscription(
            String,
            '/position',
            self._ai_command_callback,
            10
        )
    
    def _pose_callback(self, msg: Pose2D):
        x = msg.x
        y = msg.y
        with self.position_lock:
            self.current_position["x"] = x
            self.current_position["y"] = y
        logger.debug(f"Pose2D received: x={x:.2f}, y={y:.2f}")
        if self.position_callback:
            self.position_callback(x, y)
    
    def _point_callback(self, msg: Point):
        x = msg.x
        y = msg.y
        with self.position_lock:
            self.current_position["x"] = x
            self.current_position["y"] = y
        logger.debug(f"Point received: x={x:.2f}, y={y:.2f}")
        if self.position_callback:
            self.position_callback(x, y)
    
    def _array_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 2:
            x = msg.data[0]
            y = msg.data[1]
            with self.position_lock:
                self.current_position["x"] = x
                self.current_position["y"] = y
            logger.debug(f"Float64MultiArray received: x={x:.2f}, y={y:.2f}")
            if self.position_callback:
                self.position_callback(x, y)
        else:
            logger.warning(f"Float64MultiArray data missing: {msg.data}")
    
    def _json_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if "x" in data and "y" in data:
                x = float(data["x"])
                y = float(data["y"])
                with self.position_lock:
                    self.current_position["x"] = x
                    self.current_position["y"] = y
                logger.debug(f"JSON received: x={x:.2f}, y={y:.2f}")
                if "serial" in data:
                    logger.debug(f"Serial: {data['serial']}")
                if self.position_callback:
                    self.position_callback(x, y)
            else:
                logger.warning(f"JSON data missing x, y fields: {data}")
        except json.JSONDecodeError as e:
            logger.error(f"JSON parsing error: {e}")
        except Exception as e:
            logger.error(f"JSON processing error: {e}")
    
    def _ai_command_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self.ai_data_lock:
                self.current_ai_data.update(data)
                
                if "x" in data and "y" in data:
                    self.current_position["x"] = float(data["x"])
                    self.current_position["y"] = float(data["y"])
            
            logger.info(f"AI command data received: {data}")
            
            if self.ai_data_callback:
                self.ai_data_callback(data)
                
        except json.JSONDecodeError as e:
            logger.error(f"AI command JSON parsing error: {e}")
        except Exception as e:
            logger.error(f"AI command processing error: {e}")
    
    def set_position_callback(self, callback: Callable[[float, float], None]):
        self.position_callback = callback
    
    def set_ai_data_callback(self, callback: Callable[[Dict], None]):
        self.ai_data_callback = callback
    
    def get_current_position(self) -> tuple:
        with self.position_lock:
            return self.current_position["x"], self.current_position["y"]
    
    def get_current_ai_data(self) -> Dict:
        with self.ai_data_lock:
            return self.current_ai_data.copy()
    
    def get_ai_command(self) -> str:
        with self.ai_data_lock:
            for command in ["MOVING_FORWARD", "ROTATE_LEFT", "ROTATE_RIGHT", "MOVING_BACKWARD", "STOP"]:
                if self.current_ai_data.get(command):
                    return command
            return "STOP"
    
    def get_ai_situation(self) -> str:
        with self.ai_data_lock:
            return self.current_ai_data.get("situation", "")
    
    def get_ai_image(self) -> str:
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