import json
import time
import threading
from typing import Dict, Optional, Callable, Any
import paho.mqtt.client as mqtt
from utils.logger import mqtt_logger

class AIMQTTClient:
    def __init__(self, robot_id: str = "AMR001", mqtt_broker: str = "localhost", mqtt_port: int = 1883):
        self.robot_id = robot_id
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_client_id = f"ai_client_{robot_id}_{int(time.time())}"
        
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_connected = False
        
        self.mqtt_client.username_pw_set("minwoo", "minwoo")
        
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_publish = self._on_mqtt_publish
        
        self.ai_received_data = {
            "serial": "",
            "x": 0.0,
            "y": 0.0,
            "img": "",
            "case": "",
            "timeStamp": ""
        }
        
        self.data_lock = threading.Lock()
        
        self.ai_data_callback: Optional[Callable[[Dict], None]] = None
        
        self.stats_lock = threading.Lock()
        self.total_received = 0
        self.last_received_time = 0
        
        mqtt_logger.success(f"AI MQTT Client initialization completed - Robot ID: {robot_id}, Broker: {mqtt_broker}:{mqtt_port}")
        mqtt_logger.info(f"AI -> Embedded connection: {mqtt_broker}:{mqtt_port}")
    
    def connect_mqtt(self) -> bool:
        try:
            mqtt_logger.info(f"Connecting to AI MQTT broker: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            timeout = 10
            start_time = time.time()
            while not self.mqtt_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.mqtt_connected:
                mqtt_logger.success("AI MQTT broker connection successful")
                return True
            else:
                mqtt_logger.error("AI MQTT connection timeout")
                return False
                
        except Exception as e:
            mqtt_logger.error(f"AI MQTT connection failed: {e}")
            return False
    
    def disconnect_mqtt(self):
        if self.mqtt_connected:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_connected = False
            mqtt_logger.info("AI MQTT connection released")
    
    def subscribe_to_ai_data(self, robot_id: str = "AMR001"):
        topic = "ai_data"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            mqtt_logger.info(f"AI data subscription successful: {topic}")
            return True
        else:
            mqtt_logger.error(f"AI data subscription failed: {result[0]}")
            return False
    
    def set_ai_data_callback(self, callback: Callable[[Dict], None]):
        self.ai_data_callback = callback
        mqtt_logger.info("AI data callback set")
    
    def get_latest_ai_data(self) -> Dict:
        with self.data_lock:
            return self.ai_received_data.copy()
    
    def get_ai_serial(self) -> str:
        with self.data_lock:
            return self.ai_received_data.get("serial", "")
    
    def get_ai_position(self) -> tuple:
        with self.data_lock:
            x = self.ai_received_data.get("x", 0.0)
            y = self.ai_received_data.get("y", 0.0)
            return (x, y)
    
    def get_ai_image(self) -> str:
        with self.data_lock:
            return self.ai_received_data.get("img", "")
    
    def get_ai_case(self) -> str:
        with self.data_lock:
            return self.ai_received_data.get("case", "")
    
    def get_ai_timestamp(self) -> str:
        with self.data_lock:
            return self.ai_received_data.get("timeStamp", "")
    
    def get_reception_stats(self) -> Dict[str, Any]:
        with self.stats_lock:
            stats = {
                "total_received": self.total_received,
                "last_received_time": self.last_received_time,
                "mqtt_connected": self.mqtt_connected,
                "latest_ai_data": self.get_latest_ai_data()
            }
        return stats
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            mqtt_logger.success(f"AI MQTT broker connection successful: {self.mqtt_broker}:{self.mqtt_port}")
        else:
            mqtt_logger.error(f"AI MQTT connection failed: {rc}")
            self.mqtt_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        if rc != 0:
            mqtt_logger.warning(f"AI MQTT connection unexpectedly disconnected: {rc}")
        else:
            mqtt_logger.info("AI MQTT connection released")
    
    def _on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            mqtt_logger.debug(f"AI message received - topic: {topic}, data: {data}")
            
            with self.stats_lock:
                self.total_received += 1
                self.last_received_time = time.time()
            
            mqtt_logger.mqtt_receive_success(topic, data)
            
            if topic == "ai_data":
                with self.data_lock:
                    required_fields = ["serial", "x", "y", "img", "case", "timeStamp"]
                    for field in required_fields:
                        if field in data:
                            if field in ["x", "y"]:
                                self.ai_received_data[field] = float(data[field])
                            else:
                                self.ai_received_data[field] = str(data[field])
                
                if self.ai_data_callback:
                    self.ai_data_callback(data)
            
        except json.JSONDecodeError as e:
            mqtt_logger.error(f"AI JSON parsing error: {e}")
        except Exception as e:
            mqtt_logger.error(f"AI message processing error: {e}")
    
    def _on_mqtt_publish(self, client, userdata, mid):
        mqtt_logger.debug(f"AI MQTT message published: {mid}")
