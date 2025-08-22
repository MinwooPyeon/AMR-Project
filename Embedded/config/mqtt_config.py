from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class MQTTConfig:
    
    broker: str = "192.168.100.141"
    port: int = 1883
    timeout: int = 60
    keepalive: int = 60
    username: str = "minwoo"
    password: str = "minwoo"
    
    local_broker: str = "localhost"
    local_port: int = 1883
    
    max_connection_attempts: int = 3
    connection_timeout: int = 30
    reconnect_delay_min: int = 1
    reconnect_delay_max: int = 120
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'broker': self.broker,
            'port': self.port,
            'timeout': self.timeout,
            'keepalive': self.keepalive,
            'username': self.username,
            'password': self.password
        }
    
    @classmethod
    def from_dict(cls, config_dict: Dict[str, Any]) -> 'MQTTConfig':
        return cls(**config_dict)
    
    def get_local_config(self) -> Dict[str, Any]:
        return {
            'broker': self.local_broker,
            'port': self.local_port,
            'timeout': self.timeout,
            'keepalive': self.keepalive
        }
