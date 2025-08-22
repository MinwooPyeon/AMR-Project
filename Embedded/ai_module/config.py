import os
from typing import Dict, Any
from dotenv import load_dotenv

load_dotenv()

class AIConfig:
    MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
    MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
    MQTT_KEEPALIVE = int(os.getenv("MQTT_KEEPALIVE", "60"))
    
    ROS_TOPIC_ALERT = "alert"
    ROS_TOPIC_POSITION = "position"
    ROS_TOPIC_AI_DATA = "ai_data"
    
    AI_DATA_FILE_PATH = os.getenv("AI_DATA_FILE_PATH", "/tmp/ai_data.json")
    AI_IMAGES_PATH = os.getenv("AI_IMAGES_PATH", "ai_images")
    
    DEFAULT_ROBOT_ID = os.getenv("DEFAULT_ROBOT_ID", "AMR001")
    
    LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
    LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    
    @classmethod
    def get_mqtt_config(cls) -> Dict[str, Any]:
        return {
            "broker": cls.MQTT_BROKER,
            "port": cls.MQTT_PORT,
            "keepalive": cls.MQTT_KEEPALIVE
        }
    
    @classmethod
    def get_ros_config(cls) -> Dict[str, str]:
        return {
            "alert_topic": cls.ROS_TOPIC_ALERT,
            "position_topic": cls.ROS_TOPIC_POSITION,
            "ai_data_topic": cls.ROS_TOPIC_AI_DATA
        }
