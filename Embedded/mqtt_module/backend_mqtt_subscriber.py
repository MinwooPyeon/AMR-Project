import json
import time
import threading
from typing import Dict, Optional, Callable, Any
import paho.mqtt.client as mqtt
from utilities.logger import LoggerFactory


class BackendMQTTSubscriber:

    def __init__(self, mqtt_broker: str = None, mqtt_port: int = None):
        from config.system_config import get_config
        self.config = get_config()

        self.mqtt_broker = mqtt_broker or self.config.MQTT_BROKER
        self.mqtt_port = mqtt_port or self.config.MQTT_PORT
        self.mqtt_client_id = f"backend_subscriber_{int(time.time())}"

        self.logger = LoggerFactory.get_module_logger("mqtt.backend")

        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_connected = False

        if self.config.MQTT_USERNAME and self.config.MQTT_PASSWORD:
            self.mqtt_client.username_pw_set(self.config.MQTT_USERNAME, self.config.MQTT_PASSWORD)

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_publish = self._on_mqtt_publish

        self.latest_amr_data = {}
        self.data_lock = threading.Lock()

        self.amr_data_callback: Optional[Callable[[Dict], None]] = None
        self.command_callback: Optional[Callable[[Dict], None]] = None

        self.stats_lock = threading.Lock()
        self.total_received = 0
        self.last_received_time = 0

        self.logger.info(f"Backend MQTT Subscriber initialized - Broker: {self.mqtt_broker}:{self.mqtt_port}")

    def connect_mqtt(self) -> bool:
        try:
            self.logger.info(f"Connecting to MQTT broker: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, self.config.MQTT_KEEPALIVE)
            self.mqtt_client.loop_start()

            start_time = time.time()
            while not self.mqtt_connected and (time.time() - start_time) < self.config.COMMUNICATION_TIMEOUT:
                time.sleep(0.1)

            if self.mqtt_connected:
                self.logger.info("MQTT broker connection successful")
                return True
            else:
                self.logger.error("MQTT connection timeout")
                return False

        except Exception as e:
            self.logger.error(f"MQTT connection failed: {e}")
            return False

    def disconnect_mqtt(self):
        if self.mqtt_connected:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_connected = False
            self.logger.info("MQTT connection released")

    def subscribe_to_amr_data(self, robot_id: str = None):
        robot_id = robot_id or self.config.SYSTEM_NAME
        topic = f"status/{robot_id}"
        result = self.mqtt_client.subscribe(topic, qos=1)

        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            self.logger.info(f"AMR data subscription successful: {topic}")
            return True
        else:
            self.logger.error(f"AMR data subscription failed: {result[0]}")
            return False

    def subscribe_to_commands(self, robot_id: str = None):
        robot_id = robot_id or self.config.SYSTEM_NAME
        topic = f"command/{robot_id}"
        result = self.mqtt_client.subscribe(topic, qos=1)

        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            self.logger.info(f"Command subscription successful: {topic}")
            return True
        else:
            self.logger.error(f"Command subscription failed: {result[0]}")
            return False

    def publish_command(self, robot_id: str, command: Dict[str, Any]) -> bool:
        if not self.mqtt_connected:
            self.logger.warn("MQTT not connected, cannot send command")
            return False

        try:
            command["timestamp"] = time.time()
            command["source"] = "backend"
            json_str = json.dumps(command, ensure_ascii=False)
            topic = f"command/{robot_id}"
            result = self.mqtt_client.publish(topic, json_str, qos=1)

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.info(f"Command sent: {command}")
                return True
            else:
                self.logger.error(f"Command send failed: {result.rc}")
                return False

        except Exception as e:
            self.logger.error(f"Command send error: {e}")
            return False

    def set_amr_data_callback(self, callback: Callable[[Dict], None]):
        self.amr_data_callback = callback

    def set_command_callback(self, callback: Callable[[Dict], None]):
        self.command_callback = callback

    def get_latest_amr_data(self) -> Dict:
        with self.data_lock:
            return self.latest_amr_data.copy()

    def get_reception_stats(self) -> Dict[str, Any]:
        with self.stats_lock:
            return {
                "total_received": self.total_received,
                "last_received_time": self.last_received_time,
                "mqtt_connected": self.mqtt_connected,
                "latest_data": self.get_latest_amr_data()
            }

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            self.logger.info(f"MQTT connected: {self.mqtt_broker}:{self.mqtt_port}")
        else:
            self.logger.error(f"MQTT connection failed with code: {rc}")
            self.mqtt_connected = False

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        if rc != 0:
            self.logger.warn(f"MQTT unexpectedly disconnected: {rc}")
        else:
            self.logger.info("MQTT connection closed")

    def _on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic

            with self.stats_lock:
                self.total_received += 1
                self.last_received_time = time.time()

            if topic.startswith("status/"):
                with self.data_lock:
                    self.latest_amr_data = data
                if self.amr_data_callback:
                    self.amr_data_callback(data)

            elif topic.startswith("command/"):
                if self.command_callback:
                    self.command_callback(data)

        except json.JSONDecodeError as e:
            self.logger.error(f"JSON parsing error: {e}")
        except Exception as e:
            self.logger.error(f"Message processing error: {e}")

    def _on_mqtt_publish(self, client, userdata, mid):
        self.logger.debug(f"MQTT message published: {mid}")
