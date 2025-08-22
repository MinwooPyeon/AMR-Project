from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class MotorConfig:
    
    i2c_address: int = 0x40
    i2c_bus: int = 1
    
    pwm_frequency: int = 50
    pwm_resolution: int = 4095
    
    max_speed: int = 100
    min_speed: int = 10
    default_speed: int = 50
    turn_speed: int = 40
    speed_multiplier: float = 40.95
    
    max_speed_limit: float = 80.0
    emergency_stop_delay: float = 0.1
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'i2c_address': self.i2c_address,
            'pwm_frequency': self.pwm_frequency,
            'max_speed': self.max_speed,
            'min_speed': self.min_speed,
            'default_speed': self.default_speed,
            'turn_speed': self.turn_speed,
            'pwm_resolution': self.pwm_resolution,
            'speed_multiplier': self.speed_multiplier
        }
    
    @classmethod
    def from_dict(cls, config_dict: Dict[str, Any]) -> 'MotorConfig':
        return cls(**config_dict)
