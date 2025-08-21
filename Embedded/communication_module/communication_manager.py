import asyncio
import json
import logging
from typing import Dict, Any, Optional, Callable
from datetime import datetime
import websockets
import paho.mqtt.client as mqtt
import requests

class CommunicationManager:
    def __init__(self):
        self.logger = logging.getLogger("CommunicationManager")
        self.mqtt_client = None
        self.websocket_server = None
        self.http_session = requests.Session()
        
        self.mqtt_callbacks = {}
        self.websocket_callbacks = {}
        self.http_callbacks = {}
        
        self.is_running = False
    
    def setup_mqtt(self, broker: str, port: int = 1883, client_id: str = None):
        if not client_id:
            client_id = f"comm_manager_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        self.mqtt_client = mqtt.Client(client_id)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        
        try:
            self.mqtt_client.connect(broker, port, 60)
            self.mqtt_client.loop_start()
            self.logger.info(f"MQTT connected to {broker}:{port}")
            return True
        except Exception as e:
            self.logger.error(f"MQTT connection failed: {e}")
            return False
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.logger.info("MQTT connection successful")
        else:
            self.logger.error(f"MQTT connection failed with code: {rc}")
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.logger.warning(f"MQTT disconnected with code: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            if topic in self.mqtt_callbacks:
                callback = self.mqtt_callbacks[topic]
                callback(topic, payload)
            else:
                self.logger.info(f"MQTT message received on {topic}: {payload}")
                
        except Exception as e:
            self.logger.error(f"MQTT message processing failed: {e}")
    
    def subscribe_mqtt(self, topic: str, callback: Callable = None):
        if self.mqtt_client:
            self.mqtt_client.subscribe(topic)
            if callback:
                self.mqtt_callbacks[topic] = callback
            self.logger.info(f"Subscribed to MQTT topic: {topic}")
    
    def publish_mqtt(self, topic: str, message: str):
        if self.mqtt_client:
            result = self.mqtt_client.publish(topic, message)
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                self.logger.info(f"MQTT message published to {topic}")
                return True
            else:
                self.logger.error(f"MQTT publish failed: {result.rc}")
                return False
        return False
    
    async def setup_websocket_server(self, host: str = "localhost", port: int = 8765):
        try:
            self.websocket_server = await websockets.serve(
                self._websocket_handler, host, port
            )
            self.logger.info(f"WebSocket server started on {host}:{port}")
            return True
        except Exception as e:
            self.logger.error(f"WebSocket server setup failed: {e}")
            return False
    
    async def _websocket_handler(self, websocket, path):
        try:
            async for message in websocket:
                if path in self.websocket_callbacks:
                    callback = self.websocket_callbacks[path]
                    response = await callback(path, message)
                    if response:
                        await websocket.send(response)
                else:
                    self.logger.info(f"WebSocket message received on {path}: {message}")
        except websockets.exceptions.ConnectionClosed:
            self.logger.info("WebSocket connection closed")
        except Exception as e:
            self.logger.error(f"WebSocket handler error: {e}")
    
    def register_websocket_callback(self, path: str, callback: Callable):
        self.websocket_callbacks[path] = callback
        self.logger.info(f"WebSocket callback registered for path: {path}")
    
    def setup_http_client(self, base_url: str, headers: Dict = None):
        if headers:
            self.http_session.headers.update(headers)
        self.http_session.headers.update({'Content-Type': 'application/json'})
        self.logger.info(f"HTTP client configured with base URL: {base_url}")
    
    def http_get(self, url: str, params: Dict = None) -> Optional[Dict]:
        try:
            response = self.http_session.get(url, params=params)
            response.raise_for_status()
            return response.json()
        except Exception as e:
            self.logger.error(f"HTTP GET failed: {e}")
            return None
    
    def http_post(self, url: str, data: Dict) -> Optional[Dict]:
        try:
            response = self.http_session.post(url, json=data)
            response.raise_for_status()
            return response.json()
        except Exception as e:
            self.logger.error(f"HTTP POST failed: {e}")
            return None
    
    def register_http_callback(self, endpoint: str, callback: Callable):
        self.http_callbacks[endpoint] = callback
        self.logger.info(f"HTTP callback registered for endpoint: {endpoint}")
    
    def send_json_message(self, protocol: str, destination: str, data: Dict) -> bool:
        message = json.dumps(data, ensure_ascii=False)
        
        if protocol.lower() == "mqtt":
            return self.publish_mqtt(destination, message)
        elif protocol.lower() == "http":
            return self.http_post(destination, data) is not None
        else:
            self.logger.error(f"Unsupported protocol: {protocol}")
            return False
    
    def broadcast_message(self, message: str, protocols: list = None):
        if not protocols:
            protocols = ["mqtt"]
        
        for protocol in protocols:
            if protocol.lower() == "mqtt" and self.mqtt_client:
                self.publish_mqtt("broadcast", message)
            elif protocol.lower() == "websocket" and self.websocket_server:
                asyncio.create_task(self._broadcast_websocket(message))
    
    async def _broadcast_websocket(self, message: str):
        if self.websocket_server:
            websockets.broadcast(self.websocket_server.websockets, message)
    
    def get_connection_status(self) -> Dict[str, Any]:
        status = {
            "mqtt": self.mqtt_client.is_connected() if self.mqtt_client else False,
            "websocket": self.websocket_server is not None,
            "http": True
        }
        return status
    
    def cleanup(self):
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        if self.websocket_server:
            self.websocket_server.close()
        
        self.http_session.close()
        self.logger.info("Communication manager cleanup completed")

def main():
    print("=== Communication Manager Test ===")
    
    manager = CommunicationManager()
    
    def mqtt_callback(topic, message):
        print(f"MQTT callback: {topic} - {message}")
    
    def websocket_callback(path, message):
        print(f"WebSocket callback: {path} - {message}")
        return f"Echo: {message}"
    
    manager.setup_mqtt("localhost", 1883)
    manager.subscribe_mqtt("test/topic", mqtt_callback)
    
    manager.publish_mqtt("test/topic", "Hello MQTT!")
    
    print("Communication manager test completed")

if __name__ == "__main__":
    main()
