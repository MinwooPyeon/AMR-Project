import json
import base64
import logging
from typing import Dict, Any, Optional, Union
from datetime import datetime
import struct

class ProtocolHandler:
    def __init__(self):
        self.logger = logging.getLogger("ProtocolHandler")
    
    def encode_json(self, data: Dict[str, Any]) -> str:
        try:
            return json.dumps(data, ensure_ascii=False, separators=(',', ':'))
        except Exception as e:
            self.logger.error(f"JSON encoding failed: {e}")
            return "{}"
    
    def decode_json(self, message: str) -> Optional[Dict[str, Any]]:
        try:
            return json.loads(message)
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decoding failed: {e}")
            return None
    
    def encode_binary(self, data: bytes) -> str:
        try:
            return base64.b64encode(data).decode('utf-8')
        except Exception as e:
            self.logger.error(f"Binary encoding failed: {e}")
            return ""
    
    def decode_binary(self, encoded_data: str) -> Optional[bytes]:
        try:
            return base64.b64decode(encoded_data)
        except Exception as e:
            self.logger.error(f"Binary decoding failed: {e}")
            return None
    
    def create_message(self, message_type: str, payload: Any, timestamp: bool = True) -> Dict[str, Any]:
        message = {
            "type": message_type,
            "payload": payload
        }
        
        if timestamp:
            message["timestamp"] = datetime.now().isoformat()
        
        return message
    
    def parse_message(self, message: Union[str, Dict]) -> Optional[Dict[str, Any]]:
        if isinstance(message, str):
            return self.decode_json(message)
        elif isinstance(message, dict):
            return message
        else:
            self.logger.error("Invalid message format")
            return None
    
    def validate_message(self, message: Dict[str, Any], required_fields: list = None) -> bool:
        if not isinstance(message, dict):
            return False
        
        if required_fields:
            for field in required_fields:
                if field not in message:
                    self.logger.error(f"Required field missing: {field}")
                    return False
        
        return True
    
    def create_sensor_data(self, sensor_id: str, value: float, unit: str = "") -> Dict[str, Any]:
        return self.create_message("sensor_data", {
            "sensor_id": sensor_id,
            "value": value,
            "unit": unit
        })
    
    def create_command(self, command: str, parameters: Dict = None) -> Dict[str, Any]:
        payload = {"command": command}
        if parameters:
            payload["parameters"] = parameters
        
        return self.create_message("command", payload)
    
    def create_status(self, status: str, details: Dict = None) -> Dict[str, Any]:
        payload = {"status": status}
        if details:
            payload["details"] = details
        
        return self.create_message("status", payload)
    
    def create_error(self, error_code: str, error_message: str) -> Dict[str, Any]:
        return self.create_message("error", {
            "error_code": error_code,
            "error_message": error_message
        })
    
    def create_ack(self, message_id: str, success: bool = True) -> Dict[str, Any]:
        return self.create_message("ack", {
            "message_id": message_id,
            "success": success
        })
    
    def pack_binary_data(self, data_format: str, *values) -> bytes:
        try:
            return struct.pack(data_format, *values)
        except Exception as e:
            self.logger.error(f"Binary packing failed: {e}")
            return b""
    
    def unpack_binary_data(self, data_format: str, data: bytes) -> Optional[tuple]:
        try:
            return struct.unpack(data_format, data)
        except Exception as e:
            self.logger.error(f"Binary unpacking failed: {e}")
            return None
    
    def create_heartbeat(self, device_id: str) -> Dict[str, Any]:
        return self.create_message("heartbeat", {
            "device_id": device_id,
            "uptime": datetime.now().timestamp()
        })
    
    def create_config_update(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return self.create_message("config_update", {
            "config": config
        })
    
    def extract_message_type(self, message: Union[str, Dict]) -> Optional[str]:
        parsed = self.parse_message(message)
        if parsed and "type" in parsed:
            return parsed["type"]
        return None
    
    def extract_payload(self, message: Union[str, Dict]) -> Optional[Any]:
        parsed = self.parse_message(message)
        if parsed and "payload" in parsed:
            return parsed["payload"]
        return None

def main():
    print("=== Protocol Handler Test ===")
    
    handler = ProtocolHandler()
    
    test_data = {
        "sensor_id": "temp_001",
        "value": 25.5,
        "unit": "celsius"
    }
    
    encoded = handler.encode_json(test_data)
    print(f"Encoded: {encoded}")
    
    decoded = handler.decode_json(encoded)
    print(f"Decoded: {decoded}")
    
    sensor_message = handler.create_sensor_data("temp_001", 25.5, "celsius")
    print(f"Sensor message: {sensor_message}")
    
    command_message = handler.create_command("move", {"direction": "forward", "speed": 0.5})
    print(f"Command message: {command_message}")
    
    print("Protocol handler test completed")

if __name__ == "__main__":
    main()
