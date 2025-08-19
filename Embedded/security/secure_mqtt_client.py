import os
import sys
import time
import json
import logging
import threading
from typing import Dict, Optional, Callable
import paho.mqtt.client as mqtt
import ssl
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))
from security.security_manager import get_security_manager
from config.system_config import get_config

class SecureMQTTClient:
    def __init__(self, client_id: str, username: str = None, password: str = None):
        self.config = get_config()
        self.security_manager = get_security_manager()
        self.logger = self._setup_logger()
        
        self.client_id = client_id
        self.username = username
        self.password = password
        self.client = mqtt.Client(client_id=client_id)
        
        self.connected = False
        self.connection_attempts = 0
        self.max_connection_attempts = 3
        
        self.ssl_enabled = self.config.get('MQTT_SSL_ENABLED', True)
        self.cert_verify = self.config.get('MQTT_CERT_VERIFY', True)
        
        self.message_callbacks = {}
        self.connection_callbacks = []
        
        self.lock = threading.Lock()
        
        self._setup_client()
    
    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger(f'secure_mqtt_{self.client_id}')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(name)s - %(levelname)s: %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        
        return logger
    
    def _setup_client(self):
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.on_publish = self._on_publish
        self.client.on_subscribe = self._on_subscribe
        
        if self.ssl_enabled:
            self._setup_ssl()
        
        if self.username and self.password:
            self.client.username_pw_set(self.username, self.password)
        
        self._setup_security_options()
    
    def _setup_ssl(self):
        try:
            ssl_context = ssl.create_default_context()
            
            if self.cert_verify:
                ssl_context.check_hostname = True
                ssl_context.verify_mode = ssl.CERT_REQUIRED
                
                ca_cert = Path('security/certs/ca.crt')
                if ca_cert.exists():
                    ssl_context.load_verify_locations(ca_cert)
                else:
                    self.logger.warning("CA certificate not found, using default")
            else:
                ssl_context.check_hostname = False
                ssl_context.verify_mode = ssl.CERT_NONE
            
            client_cert = Path('security/certs/client.crt')
            client_key = Path('security/certs/client.key')
            
            if client_cert.exists() and client_key.exists():
                ssl_context.load_cert_chain(client_cert, client_key)
                self.logger.info("Client certificate loaded")
            
            self.client.tls_set_context(ssl_context)
            self.client.tls_insecure_set(not self.cert_verify)
            
        except Exception as e:
            self.logger.error(f"SSL setup failed: {e}")
            raise
    
    def _setup_security_options(self):
        self.client.connect_timeout = 30
        
        self.client.keepalive = 60
        
        self.client.max_inflight_messages_set(20)
        
        self.client.reconnect_delay_set(min_delay=1, max_delay=120)
    
    def connect(self, broker: str = None, port: int = None) -> bool:
        broker = broker or self.config.MQTT_BROKER
        port = port or self.config.MQTT_PORT
        
        if self.ssl_enabled and port == 1883:
            port = 8883
        
        try:
            self.logger.info(f"Connecting to MQTT broker: {broker}:{port}")
            
            if not self.security_manager.check_rate_limit('localhost'):
                self.logger.warning("Rate limit exceeded for MQTT connection")
                return False
            
            result = self.client.connect(broker, port, 60)
            
            if result == mqtt.MQTT_ERR_SUCCESS:
                self.client.loop_start()
                
                timeout = 10
                start_time = time.time()
                
                while not self.connected and (time.time() - start_time) < timeout:
                    time.sleep(0.1)
                
                if self.connected:
                    self.logger.info("MQTT connection established successfully")
                    self.connection_attempts = 0
                    return True
                else:
                    self.logger.error("MQTT connection timeout")
                    return False
            else:
                self.logger.error(f"MQTT connection failed: {result}")
                return False
                
        except Exception as e:
            self.logger.error(f"MQTT connection error: {e}")
            self.connection_attempts += 1
            
            if self.connection_attempts < self.max_connection_attempts:
                self.logger.info(f"Retrying connection in 5 seconds... (attempt {self.connection_attempts})")
                time.sleep(5)
                return self.connect(broker, port)
            
            return False
    
    def disconnect(self):
        if self.connected:
            self.logger.info("Disconnecting from MQTT broker")
            self.client.loop_stop()
            self.client.disconnect()
            self.connected = False
    
    def subscribe(self, topic: str, qos: int = 1, callback: Callable = None) -> bool:
        if not self.connected:
            self.logger.error("Not connected to MQTT broker")
            return False
        
        try:
            if not self._validate_topic(topic):
                self.logger.error(f"Invalid topic format: {topic}")
                return False
            
            result, mid = self.client.subscribe(topic, qos)
            
            if result == mqtt.MQTT_ERR_SUCCESS:
                self.logger.info(f"Subscribed to topic: {topic} (QoS: {qos})")
                
                if callback:
                    self.message_callbacks[topic] = callback
                
                return True
            else:
                self.logger.error(f"Subscription failed: {result}")
                return False
                
        except Exception as e:
            self.logger.error(f"Subscription error: {e}")
            return False
    
    def publish(self, topic: str, payload: str, qos: int = 1, retain: bool = False) -> bool:
        if not self.connected:
            self.logger.error("Not connected to MQTT broker")
            return False
        
        try:
            if not self._validate_topic(topic):
                self.logger.error(f"Invalid topic format: {topic}")
                return False
            
            encrypted_payload = self.security_manager.encrypt_data(payload)
            
            result = self.client.publish(topic, encrypted_payload, qos, retain)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.debug(f"Message published to topic: {topic}")
                return True
            else:
                self.logger.error(f"Publish failed: {result.rc}")
                return False
                
        except Exception as e:
            self.logger.error(f"Publish error: {e}")
            return False
    
    def _validate_topic(self, topic: str) -> bool:
        if not topic or len(topic) > 65535:
            return False
        
        dangerous_chars = ['<', '>', '"', "'", '&', '|', ';', '`', '$', '(', ')']
        if any(char in topic for char in dangerous_chars):
            return False
        
        allowed_patterns = [
            r'^status/[\w\-]+$',
            r'^command/[\w\-]+$',
            r'^amr/[\w\-/]+$',
            r'^system/[\w\-/]+$'
        ]
        
        import re
        for pattern in allowed_patterns:
            if re.match(pattern, topic):
                return True
        
        return False
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            self.logger.info("MQTT connection established")
            
            for callback in self.connection_callbacks:
                try:
                    callback(True)
                except Exception as e:
                    self.logger.error(f"Connection callback error: {e}")
        else:
            self.connected = False
            self.logger.error(f"MQTT connection failed: {rc}")
            
            self.security_manager._raise_security_alert(
                'mqtt_connection_failed',
                f'MQTT connection failed with code: {rc}'
            )
    
    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        
        if rc != 0:
            self.logger.warning(f"MQTT connection lost: {rc}")
            
            self.security_manager._raise_security_alert(
                'mqtt_connection_lost',
                f'MQTT connection lost with code: {rc}'
            )
        else:
            self.logger.info("MQTT connection closed")
        
        for callback in self.connection_callbacks:
            try:
                callback(False)
            except Exception as e:
                self.logger.error(f"Disconnection callback error: {e}")
    
    def _on_message(self, client, userdata, msg):
        try:
            decrypted_payload = self.security_manager.decrypt_data(msg.payload.decode())
            
            self.logger.debug(f"Message received - Topic: {msg.topic}, QoS: {msg.qos}")
            
            if msg.topic in self.message_callbacks:
                try:
                    self.message_callbacks[msg.topic](msg.topic, decrypted_payload)
                except Exception as e:
                    self.logger.error(f"Message callback error: {e}")
            
            self._check_message_security(msg.topic, decrypted_payload)
            
        except Exception as e:
            self.logger.error(f"Message processing error: {e}")
    
    def _on_publish(self, client, userdata, mid):
        self.logger.debug(f"Message published: {mid}")
    
    def _on_subscribe(self, client, userdata, mid, granted_qos):
        self.logger.debug(f"Subscription confirmed: {mid}, QoS: {granted_qos}")
    
    def _check_message_security(self, topic: str, payload: str):
        if len(payload) > 1024 * 1024:
            self.security_manager._raise_security_alert(
                'message_size_limit',
                f'Message size exceeds limit: {len(payload)} bytes'
            )
        
        try:
            json.loads(payload)
        except json.JSONDecodeError:
            self.security_manager._raise_security_alert(
                'invalid_json',
                f'Invalid JSON format in message: {topic}'
            )
        
        if topic.startswith('command/'):
            self._validate_command_message(payload)
    
    def _validate_command_message(self, payload: str):
        try:
            command = json.loads(payload)
            
            allowed_commands = [
                'MOVE_FORWARD', 'MOVE_BACKWARD', 'ROTATE_LEFT', 'ROTATE_RIGHT',
                'STOP', 'EMERGENCY_STOP', 'GET_STATUS', 'GET_CONFIG'
            ]
            
            if 'action' in command:
                if command['action'] not in allowed_commands:
                    self.security_manager._raise_security_alert(
                        'unauthorized_command',
                        f'Unauthorized command: {command["action"]}'
                    )
            
            if 'speed' in command:
                speed = float(command['speed'])
                if speed < 0 or speed > 100:
                    self.security_manager._raise_security_alert(
                        'invalid_speed',
                        f'Invalid speed value: {speed}'
                    )
                    
        except Exception as e:
            self.security_manager._raise_security_alert(
                'command_validation_error',
                f'Command validation error: {e}'
            )
    
    def add_connection_callback(self, callback: Callable):
        self.connection_callbacks.append(callback)
    
    def get_connection_status(self) -> Dict:
        return {
            'connected': self.connected,
            'client_id': self.client_id,
            'username': self.username,
            'ssl_enabled': self.ssl_enabled,
            'connection_attempts': self.connection_attempts
        }

if __name__ == "__main__":
    client = SecureMQTTClient(
        client_id="test_client",
        username="admin",
        password="admin123!"
    )
    
    def message_callback(topic: str, payload: str):
        print(f"Received message - Topic: {topic}, Payload: {payload}")
    
    def connection_callback(connected: bool):
        print(f"Connection status changed: {connected}")
    
    client.add_connection_callback(connection_callback)
    
    if client.connect():
        client.subscribe("status/AMR001", callback=message_callback)
        
        test_message = json.dumps({
            "action": "GET_STATUS",
            "timestamp": time.time()
        })
        client.publish("command/AMR001", test_message)
        
        time.sleep(10)
        
        client.disconnect()
    
    print("Secure MQTT client test completed!")
