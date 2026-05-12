import json
import time
import threading
from typing import Dict, Optional, Callable, Any

from mqtt_module.sensor_data_transmitter import SensorDataTransmitter
from mqtt_module.ai_mqtt_client import AIMQTTClient
from utilities.logger import mqtt_logger


class MQTTManager:

    def __init__(self, robot_id: str = "AMR001"):
        from config.system_config import get_config
        config = get_config()

        self.robot_id = robot_id

        self.backend_transmitter = SensorDataTransmitter(robot_id, config.MQTT_BROKER, config.MQTT_PORT)
        self.ai_client = AIMQTTClient(robot_id, config.LOCAL_MQTT_BROKER, config.LOCAL_MQTT_PORT)

        self.ai_command_data: Dict = {}
        self.data_lock = threading.Lock()

        self.ai_command_callback: Optional[Callable] = None

        self.stats_lock = threading.Lock()
        self.backend_sent_count = 0
        self.ai_received_count = 0

        mqtt_logger.success(f"MQTT Manager initialized - Robot ID: {robot_id}")
        mqtt_logger.info(f"Backend: {config.MQTT_BROKER}:{config.MQTT_PORT}")
        mqtt_logger.info(f"AI: {config.LOCAL_MQTT_BROKER}:{config.LOCAL_MQTT_PORT}")

    def connect_all(self) -> bool:
        success_count = 0

        if self.backend_transmitter.connect_mqtt():
            mqtt_logger.success("Backend transmitter connected")
            success_count += 1
        else:
            mqtt_logger.error("Backend transmitter connection failed")

        if self.ai_client.connect_mqtt():
            mqtt_logger.success("AI client connected")
            success_count += 1
        else:
            mqtt_logger.error("AI client connection failed")

        if success_count > 0:
            self._setup_subscriptions()

        return success_count > 0

    def disconnect_all(self):
        self.backend_transmitter.disconnect_mqtt()
        self.ai_client.disconnect_mqtt()
        mqtt_logger.info("All MQTT connections disconnected")

    def _setup_subscriptions(self):
        self.ai_client.subscribe_to_ai_data(self.robot_id)
        self.ai_client.set_ai_data_callback(self._on_ai_data)

    def send_to_backend(self, data: Dict[str, Any]) -> bool:
        ai_data = self.get_ai_command_data()

        self.backend_transmitter.update_embedded_data(
            state=data.get("state", "RUNNING"),
            x=ai_data.get("x", 0.0),
            y=ai_data.get("y", 0.0),
            speed=data.get("speed", 25.0)
        )

        success = self.backend_transmitter.send_embedded_data()
        if success:
            with self.stats_lock:
                self.backend_sent_count += 1
        return success

    def set_ai_command_callback(self, callback: Callable[[Dict], None]):
        self.ai_command_callback = callback

    def _on_ai_data(self, data: Dict):
        with self.data_lock:
            self.ai_command_data = data
        with self.stats_lock:
            self.ai_received_count += 1

        if self.ai_command_callback:
            self.ai_command_callback(data)

    def get_ai_command_data(self) -> Dict:
        with self.data_lock:
            return self.ai_command_data.copy()

    def get_ai_position(self) -> tuple:
        return self.ai_client.get_ai_position()

    def get_ai_image(self) -> str:
        return self.ai_client.get_ai_image()

    def get_ai_case(self) -> str:
        return self.ai_client.get_ai_case()

    def get_stats(self) -> Dict[str, Any]:
        with self.stats_lock:
            return {
                "backend_sent": self.backend_sent_count,
                "ai_received": self.ai_received_count,
                "backend_connected": self.backend_transmitter.connected,
                "ai_connected": self.ai_client.mqtt_connected
            }

    def get_connection_status(self) -> Dict[str, bool]:
        return {
            "backend_transmitter": self.backend_transmitter.connected,
            "ai_client": self.ai_client.mqtt_connected
        }
