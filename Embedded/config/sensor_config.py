from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class SensorConfig:
    
    imu_i2c_address: int = 0x4B
    imu_sample_rate: int = 100
    imu_gyro_range: int = 250
    imu_accel_range: int = 2
    imu_mag_range: int = 1300
    
    angle_control_frequency: float = 50.0
    angle_tolerance: float = 1.0
    angle_max_error: float = 180.0
    angle_pid_kp: float = 2.0
    angle_pid_ki: float = 0.1
    angle_pid_kd: float = 0.5
    angle_pid_integral_limit: float = 100.0
    angle_pid_output_limit: float = 100.0
    
    rotation_90_degrees: float = 90.0
    rotation_180_degrees: float = 180.0
    rotation_360_degrees: float = 360.0
    
    sensor_sync_target_samples: int = 300
    sensor_sync_sleep_ms: int = 10
    sensor_sync_timeout_ms: int = 10000
    
    i2c_bus: int = 1
    gpio_base_pin: int = 18
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'imu_i2c_address': self.imu_i2c_address,
            'imu_sample_rate': self.imu_sample_rate,
            'imu_gyro_range': self.imu_gyro_range,
            'imu_accel_range': self.imu_accel_range,
            'imu_mag_range': self.imu_mag_range,
            'angle_control_frequency': self.angle_control_frequency,
            'angle_tolerance': self.angle_tolerance,
            'angle_max_error': self.angle_max_error,
            'angle_pid_kp': self.angle_pid_kp,
            'angle_pid_ki': self.angle_pid_ki,
            'angle_pid_kd': self.angle_pid_kd,
            'angle_pid_integral_limit': self.angle_pid_integral_limit,
            'angle_pid_output_limit': self.angle_pid_output_limit,
            'rotation_90_degrees': self.rotation_90_degrees,
            'rotation_180_degrees': self.rotation_180_degrees,
            'rotation_360_degrees': self.rotation_360_degrees,
            'sensor_sync_target_samples': self.sensor_sync_target_samples,
            'sensor_sync_sleep_ms': self.sensor_sync_sleep_ms,
            'sensor_sync_timeout_ms': self.sensor_sync_timeout_ms
        }
    
    @classmethod
    def from_dict(cls, config_dict: Dict[str, Any]) -> 'SensorConfig':
        return cls(**config_dict)
    
    def get_imu_config(self) -> Dict[str, Any]:
        return {
            'i2c_address': self.imu_i2c_address,
            'sample_rate': self.imu_sample_rate,
            'gyro_range': self.imu_gyro_range,
            'accel_range': self.imu_accel_range,
            'mag_range': self.imu_mag_range
        }
    
    def get_angle_control_config(self) -> Dict[str, Any]:
        return {
            'frequency': self.angle_control_frequency,
            'tolerance': self.angle_tolerance,
            'max_error': self.angle_max_error,
            'pid_kp': self.angle_pid_kp,
            'pid_ki': self.angle_pid_ki,
            'pid_kd': self.angle_pid_kd,
            'pid_integral_limit': self.angle_pid_integral_limit,
            'pid_output_limit': self.angle_pid_output_limit
        }
    
    def get_rotation_config(self) -> Dict[str, Any]:
        return {
            'rotation_90': self.rotation_90_degrees,
            'rotation_180': self.rotation_180_degrees,
            'rotation_360': self.rotation_360_degrees
        }
    
    def get_sensor_sync_config(self) -> Dict[str, Any]:
        return {
            'target_samples': self.sensor_sync_target_samples,
            'sleep_ms': self.sensor_sync_sleep_ms,
            'timeout_ms': self.sensor_sync_timeout_ms
        }
