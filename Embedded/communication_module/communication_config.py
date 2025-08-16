import os
from typing import Dict, Any, List

class CommunicationConfig:
    DEFAULT_MQTT_BROKER = "localhost"
    DEFAULT_MQTT_PORT = 1883
    DEFAULT_MQTT_KEEPALIVE = 60
    
    DEFAULT_WEBSOCKET_HOST = "localhost"
    DEFAULT_WEBSOCKET_PORT = 8765
    
    DEFAULT_HTTP_TIMEOUT = 30
    DEFAULT_HTTP_RETRIES = 3
    
    MQTT_TOPICS = {
        "sensor_data": "amr/sensor/data",
        "motor_control": "amr/motor/control",
        "status": "amr/status",
        "command": "amr/command",
        "alert": "amr/alert",
        "config": "amr/config",
        "heartbeat": "amr/heartbeat"
    }
    
    WEBSOCKET_PATHS = {
        "sensor": "/sensor",
        "control": "/control",
        "status": "/status",
        "config": "/config"
    }
    
    HTTP_ENDPOINTS = {
        "sensor_data": "/api/sensor/data",
        "motor_control": "/api/motor/control",
        "status": "/api/status",
        "config": "/api/config"
    }
    
    MESSAGE_TYPES = {
        "SENSOR_DATA": "sensor_data",
        "MOTOR_CONTROL": "motor_control",
        "STATUS": "status",
        "COMMAND": "command",
        "ALERT": "alert",
        "CONFIG": "config",
        "HEARTBEAT": "heartbeat",
        "ACK": "ack",
        "ERROR": "error"
    }
    
    PROTOCOL_CONFIGS = {
        "mqtt": {
            "broker": os.getenv("MQTT_BROKER", DEFAULT_MQTT_BROKER),
            "port": int(os.getenv("MQTT_PORT", str(DEFAULT_MQTT_PORT))),
            "keepalive": int(os.getenv("MQTT_KEEPALIVE", str(DEFAULT_MQTT_KEEPALIVE))),
            "username": os.getenv("MQTT_USERNAME", ""),
            "password": os.getenv("MQTT_PASSWORD", ""),
            "client_id": os.getenv("MQTT_CLIENT_ID", "")
        },
        "websocket": {
            "host": os.getenv("WEBSOCKET_HOST", DEFAULT_WEBSOCKET_HOST),
            "port": int(os.getenv("WEBSOCKET_PORT", str(DEFAULT_WEBSOCKET_PORT))),
            "max_connections": int(os.getenv("WEBSOCKET_MAX_CONNECTIONS", "100"))
        },
        "http": {
            "base_url": os.getenv("HTTP_BASE_URL", "http://localhost:8000"),
            "timeout": int(os.getenv("HTTP_TIMEOUT", str(DEFAULT_HTTP_TIMEOUT))),
            "retries": int(os.getenv("HTTP_RETRIES", str(DEFAULT_HTTP_RETRIES))),
            "headers": {
                "Content-Type": "application/json",
                "User-Agent": "AMR-Communication/1.0"
            }
        }
    }
    
    @classmethod
    def get_mqtt_config(cls) -> Dict[str, Any]:
        return cls.PROTOCOL_CONFIGS["mqtt"].copy()
    
    @classmethod
    def get_websocket_config(cls) -> Dict[str, Any]:
        return cls.PROTOCOL_CONFIGS["websocket"].copy()
    
    @classmethod
    def get_http_config(cls) -> Dict[str, Any]:
        return cls.PROTOCOL_CONFIGS["http"].copy()
    
    @classmethod
    def get_topic(cls, topic_name: str) -> str:
        return cls.MQTT_TOPICS.get(topic_name, f"amr/{topic_name}")
    
    @classmethod
    def get_websocket_path(cls, path_name: str) -> str:
        return cls.WEBSOCKET_PATHS.get(path_name, f"/{path_name}")
    
    @classmethod
    def get_http_endpoint(cls, endpoint_name: str) -> str:
        return cls.HTTP_ENDPOINTS.get(endpoint_name, f"/api/{endpoint_name}")
    
    @classmethod
    def get_message_type(cls, type_name: str) -> str:
        return cls.MESSAGE_TYPES.get(type_name, type_name.lower())
    
    @classmethod
    def get_all_topics(cls) -> List[str]:
        return list(cls.MQTT_TOPICS.values())
    
    @classmethod
    def get_all_websocket_paths(cls) -> List[str]:
        return list(cls.WEBSOCKET_PATHS.values())
    
    @classmethod
    def get_all_http_endpoints(cls) -> List[str]:
        return list(cls.HTTP_ENDPOINTS.values())
    
    @classmethod
    def validate_config(cls, config: Dict[str, Any]) -> bool:
        required_fields = ["broker", "port"]
        
        for field in required_fields:
            if field not in config:
                return False
        
        return True
