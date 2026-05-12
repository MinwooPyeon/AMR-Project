import os
from dataclasses import dataclass, field
from typing import Dict, Any


@dataclass
class MQTTConfig:

    broker: str = field(default_factory=lambda: os.getenv('MQTT_BROKER', '192.168.100.141'))
    port: int = field(default_factory=lambda: int(os.getenv('MQTT_PORT', '1883')))
    timeout: int = 60
    keepalive: int = 60
    username: str = field(default_factory=lambda: os.getenv('MQTT_USERNAME', ''))
    password: str = field(default_factory=lambda: os.getenv('MQTT_PASSWORD', ''))

    local_broker: str = field(default_factory=lambda: os.getenv('MQTT_LOCAL_BROKER', 'localhost'))
    local_port: int = field(default_factory=lambda: int(os.getenv('MQTT_LOCAL_PORT', '1883')))

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
