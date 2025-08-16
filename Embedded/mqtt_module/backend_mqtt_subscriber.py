import json
import time
import threading
import logging
from typing import Dict, Optional, Callable, Any
import paho.mqtt.client as mqtt

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class BackendMQTTSubscriber:
    
    def __init__(self, mqtt_broker: str = "192.168.100.141", mqtt_port: int = 1883):
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_client_id = f"backend_subscriber_{int(time.time())}"
        
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_connected = False
        
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
        
        logger.info(f"Backend MQTT Subscriber initialization completed - Broker: {mqtt_broker}:{mqtt_port}")
    
    def connect_mqtt(self) -> bool:
        try:
            logger.info(f"MQTT 브로커에 연결 중: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            timeout = 10
            start_time = time.time()
            while not self.mqtt_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.mqtt_connected:
                logger.info("MQTT broker connection successful")    
                return True
            else:
                logger.error("MQTT connection timeout")
                return False
                
        except Exception as e:
            logger.error(f"MQTT connection failed: {e}")
            return False
    
    def disconnect_mqtt(self):
        if self.mqtt_connected:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_connected = False
            logger.info("MQTT connection released")
    
    def subscribe_to_amr_data(self, robot_id: str = "AMR001"):
        topic = f"status/{robot_id}"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            logger.info(f"AMR data subscription successful: {topic}")
            return True
        else:
            logger.error(f"AMR data subscription failed: {result[0]}")
            return False
    
    def subscribe_to_commands(self, robot_id: str = "AMR001"):
        topic = f"command/{robot_id}"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            logger.info(f"Command subscription successful: {topic}")
            return True
        else:
            logger.error(f"Command subscription failed: {result[0]}")
            return False
    
    def publish_command(self, robot_id: str, command: Dict[str, Any]) -> bool:
        if not self.mqtt_connected:
            logger.warning("MQTT connection not established, cannot send command")
            return False
        
        try:
            command["timestamp"] = time.time()
            command["source"] = "backend"
            
            json_str = json.dumps(command, ensure_ascii=False)
            
            topic = f"command/{robot_id}"
            
            result = self.mqtt_client.publish(topic, json_str, qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.info(f"Command sent successfully: {command}")
                return True
            else:
                logger.error(f"Command sending failed: {result.rc}")
                return False
                
        except Exception as e:
            logger.error(f"Command sending error: {e}")
            return False
    
    def set_amr_data_callback(self, callback: Callable[[Dict], None]):
        self.amr_data_callback = callback
        logger.info("AMR data callback set")
    
    def set_command_callback(self, callback: Callable[[Dict], None]):
        self.command_callback = callback
        logger.info("Command callback set")
    
    def get_latest_amr_data(self) -> Dict:
        with self.data_lock:
            return self.latest_amr_data.copy()
    
    def get_reception_stats(self) -> Dict[str, Any]:    
        with self.stats_lock:
            stats = {
                "total_received": self.total_received,
                "last_received_time": self.last_received_time,
                "mqtt_connected": self.mqtt_connected,
                "latest_data": self.get_latest_amr_data()
            }
        return stats
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.mqtt_connected = True
            logger.info(f"MQTT broker connection successful: {self.mqtt_broker}:{self.mqtt_port}")
        else:
            logger.error(f"MQTT connection failed: {rc}")
            self.mqtt_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.mqtt_connected = False
        if rc != 0:
            logger.warning(f"MQTT connection unexpectedly disconnected: {rc}")
        else:
            logger.info("MQTT connection released")
    
    def _on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            logger.debug(f"Message received - topic: {topic}, data: {data}")
            
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
            logger.error(f"JSON parsing error: {e}")
        except Exception as e:
            logger.error(f"Message processing error: {e}")
    
    def _on_mqtt_publish(self, client, userdata, mid):
        logger.debug(f"MQTT message published: {mid}")

def test_backend_mqtt_subscriber():
    print("=== Backend MQTT Subscriber test ===")
    print("Receive data from AMR and send commands to AMR")
    print("MQTT broker: 192.168.100.141:1883")
    print("=" * 60)
    
    backend = BackendMQTTSubscriber("192.168.100.141", 1883)
    
    def amr_data_callback(data):
        print(f"\r AMR data received: "
              f"serial={data.get('serial', 'N/A')} | "
              f"status={data.get('status', 'N/A')} | "
              f"battery_level={data.get('battery_level', 0):.1f}% | "
              f"position=({data.get('x', 0):.1f}, {data.get('y', 0):.1f}) | "
              f"speed={data.get('speed', 0):.1f}", end="")
    
    def command_callback(data):
        print(f"\n Command received: {data}")
    
    backend.set_amr_data_callback(amr_data_callback)
    backend.set_command_callback(command_callback)
    
    print("Connecting to MQTT broker...")
    if not backend.connect_mqtt():
        print("MQTT connection failed")
        return
    
    print("MQTT connection successful")
    
    print("Subscribing to AMR data...")
    if not backend.subscribe_to_amr_data("AMR001"):
        print("AMR data subscription failed")
        return
    
    print("AMR data subscription successful")
    
    print("Subscribing to commands...")
    if not backend.subscribe_to_commands("AMR001"):
        print("Command subscription failed")
        return
    
    print("Command subscription successful")
    
    print("\nWaiting for data reception... (30 seconds)")
    print("Press Ctrl+C to exit or enter a command:")
    print("  - 'MOVE_FORWARD': Move forward")
    print("  - 'MOVE_BACKWARD': Move backward")
    print("  - 'ROTATE_LEFT': Rotate left")
    print("  - 'ROTATE_RIGHT': Rotate right")
    print("  - 'stop': Stop")
    print("  - 'custom': Custom command")
    
    start_time = time.time()
    
    try:
        while time.time() - start_time < 30:
            try:
                import select
                import sys
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    command = input().strip()
                    
                    if command == "MOVE_FORWARD":
                        backend.publish_command("AMR001", {
                            "action": "MOVE_FORWARD",
                            "speed": 50.0
                        })
                    elif command == "MOVE_BACKWARD":
                        backend.publish_command("AMR001", {
                            "action": "MOVE_BACKWARD",
                            "speed": 50.0
                        })
                    elif command == "ROTATE_LEFT":
                        backend.publish_command("AMR001", {
                            "action": "ROTATE_LEFT",
                            "speed": 50.0
                        })
                    elif command == "ROTATE_RIGHT":
                        backend.publish_command("AMR001", {
                            "action": "ROTATE_RIGHT",
                            "speed": 50.0
                        })
                    elif command == "stop":
                        backend.publish_command("AMR001", {
                            "action": "stop_motor"
                        })
                    elif command == "custom":
                        print("Enter custom command (JSON format):")
                        try:
                            custom_cmd = json.loads(input())
                            backend.publish_command("AMR001", custom_cmd)
                        except json.JSONDecodeError:
                            print("Invalid JSON format.")
                    else:
                        print(f"Unknown command: {command}")
                        
            except (EOFError, KeyboardInterrupt):
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    
    stats = backend.get_reception_stats()
    print(f"\n Reception statistics:")
    for key, value in stats.items():
        if key != "latest_data":
            print(f"  - {key}: {value}")
    
    latest_data = stats.get("latest_data", {})
    if latest_data:
        print(f"\nLatest AMR Data:")
        print(json.dumps(latest_data, indent=2, ensure_ascii=False))
    
    backend.disconnect_mqtt()
    print("\nBackend MQTT Subscriber cleanup completed")

if __name__ == "__main__":
    test_backend_mqtt_subscriber() 