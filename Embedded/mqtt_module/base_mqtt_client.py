import json
import time
import threading
from typing import Dict, Optional, Callable, Any
from abc import ABC, abstractmethod
import paho.mqtt.client as mqtt

from utilities.logger import LoggerFactory


class BaseMQTTClient(ABC):
    
    def __init__(self, 
                 client_id: str,
                 broker: str = "localhost", 
                 port: int = 1883,
                 username: str = None,
                 password: str = None,
                 keepalive: int = 60):

        self.client_id = client_id
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        self.keepalive = keepalive
        
        self.connected = False
        self.connection_attempts = 0
        self.max_connection_attempts = 3
        
        self.message_callbacks: Dict[str, Callable] = {}
        self.connection_callbacks: list[Callable] = []
        
        self.data_lock = threading.Lock()
        self.stats_lock = threading.Lock()
        
        self.total_received = 0
        self.total_sent = 0
        self.last_received_time = 0
        self.last_sent_time = 0
        
        self.logger = LoggerFactory.get_module_logger("mqtt")
        
        self._setup_client()
        self._setup_callbacks()
    
    def _setup_client(self):
        self.client = mqtt.Client(client_id=self.client_id)
        
        if self.username and self.password:
            self.client.username_pw_set(self.username, self.password)
        
        self.client.connect_timeout = 30
        self.client.keepalive = self.keepalive
        self.client.max_inflight_messages_set(20)
        self.client.reconnect_delay_set(min_delay=1, max_delay=120)
    
    def _setup_callbacks(self):
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.on_publish = self._on_publish
        self.client.on_subscribe = self._on_subscribe
    
    def connect(self) -> bool:
        try:
            self.logger.info(f"Connecting to MQTT broker: {self.broker}:{self.port}")
            
            result = self.client.connect(self.broker, self.port, self.keepalive)
            
            if result == mqtt.MQTT_ERR_SUCCESS:
                self.client.loop_start()
                
                timeout = 10
                start_time = time.time()
                
                while not self.connected and (time.time() - start_time) < timeout:
                    time.sleep(0.1)
                
                if self.connected:
                    self.logger.connection_success("MQTT", f"{self.broker}:{self.port}")
                    self.connection_attempts = 0
                    return True
                else:
                    self.logger.connection_error("MQTT", "Connection timeout")
                    return False
            else:
                self.logger.connection_error("MQTT", f"Connection failed: {result}")
                return False
                
        except Exception as e:
            self.logger.connection_error("MQTT", str(e))
            self.connection_attempts += 1
            
            if self.connection_attempts < self.max_connection_attempts:
                self.logger.info(f"Retrying connection in 5 seconds... (attempt {self.connection_attempts})")
                time.sleep(5)
                return self.connect()
            
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
            result = self.client.publish(topic, payload, qos, retain)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                with self.stats_lock:
                    self.total_sent += 1
                    self.last_sent_time = time.time()
                
                self.logger.debug(f"Message published to topic: {topic}")
                return True
            else:
                self.logger.error(f"Publish failed: {result.rc}")
                return False
                
        except Exception as e:
            self.logger.error(f"Publish error: {e}")
            return False
    
    def add_connection_callback(self, callback: Callable):
        self.connection_callbacks.append(callback)
    
    def get_connection_status(self) -> Dict[str, Any]:
        return {
            'connected': self.connected,
            'client_id': self.client_id,
            'broker': self.broker,
            'port': self.port,
            'connection_attempts': self.connection_attempts
        }
    
    def get_stats(self) -> Dict[str, Any]:
        with self.stats_lock:
            return {
                'total_received': self.total_received,
                'total_sent': self.total_sent,
                'last_received_time': self.last_received_time,
                'last_sent_time': self.last_sent_time
            }
    
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
            self.logger.connection_error("MQTT", f"Connection failed with code: {rc}")
    
    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        
        if rc != 0:
            self.logger.warn(f"MQTT connection lost: {rc}")
        else:
            self.logger.info("MQTT connection closed")
        
        for callback in self.connection_callbacks:
            try:
                callback(False)
            except Exception as e:
                self.logger.error(f"Disconnection callback error: {e}")
    
    def _on_message(self, client, userdata, msg):
        try:
            with self.stats_lock:
                self.total_received += 1
                self.last_received_time = time.time()
            
            self.logger.debug(f"Message received - Topic: {msg.topic}, QoS: {msg.qos}")
            
            if msg.topic in self.message_callbacks:
                try:
                    self.message_callbacks[msg.topic](msg.topic, msg.payload.decode())
                except Exception as e:
                    self.logger.error(f"Message callback error: {e}")
            
        except Exception as e:
            self.logger.error(f"Message processing error: {e}")
    
    def _on_publish(self, client, userdata, mid):
        self.logger.debug(f"Message published: {mid}")
    
    def _on_subscribe(self, client, userdata, mid, granted_qos):
        self.logger.debug(f"Subscription confirmed: {mid}, QoS: {granted_qos}")
    
    @abstractmethod
    def setup_subscriptions(self):
        pass
    
    @abstractmethod
    def process_message(self, topic: str, payload: str):
        pass
