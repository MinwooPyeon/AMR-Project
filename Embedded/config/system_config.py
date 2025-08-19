import os
from typing import Dict, Any
from dataclasses import dataclass

from .motor_config import MotorConfig
from .mqtt_config import MQTTConfig
from .sensor_config import SensorConfig


@dataclass
class SystemConfig:
    
    def __init__(self):
        self.SYSTEM_NAME = "AMR001"
        self.DEBUG_MODE = True
        
        self.motor = MotorConfig()
        self.mqtt = MQTTConfig()
        self.sensor = SensorConfig()
        
        self.WEBSOCKET_PORT = 8080
        self.HTTP_PORT = 8000
        self.COMMUNICATION_TIMEOUT = 10
        
        self.LOG_LEVEL = "INFO"
        self.LOG_FORMAT = "[%(asctime)s] %(levelname)s: %(message)s"
        self.LOG_FILE = "logs/amr_system.log"
        
        self.DEBUG_COUNTER_LIMIT = 100
        self.DEBUG_PRINT_INTERVAL = 1.0  

        self.CONTROL_LOOP_FREQUENCY = 50.0  
        self.CONTROL_LOOP_PERIOD_MS = 20    
        
        self.BATTERY_LOW_THRESHOLD = 20.0  
        
        self.TEST_DURATION = 30  
        self.TEST_SPEED = 50
        self.TEST_CURVATURE = 0.3
        
        self.BACKUP_INTERVAL = 3600  
        self.BACKUP_RETENTION_DAYS = 7
        
        self.NETWORK_TIMEOUT = 5.0
        self.NETWORK_RETRY_COUNT = 3
        self.NETWORK_RETRY_DELAY = 1.0
        
        self._setup_compatibility_attributes()
    
    def _setup_compatibility_attributes(self):
        self.MQTT_BROKER = self.mqtt.broker
        self.MQTT_PORT = self.mqtt.port
        self.MQTT_TIMEOUT = self.mqtt.timeout
        self.MQTT_KEEPALIVE = self.mqtt.keepalive
        self.MQTT_USERNAME = self.mqtt.username
        self.MQTT_PASSWORD = self.mqtt.password
        self.LOCAL_MQTT_BROKER = self.mqtt.local_broker
        self.LOCAL_MQTT_PORT = self.mqtt.local_port
        
        self.MOTOR_I2C_ADDRESS = self.motor.i2c_address
        self.MOTOR_PWM_FREQUENCY = self.motor.pwm_frequency
        self.MOTOR_MAX_SPEED = self.motor.max_speed
        self.MOTOR_MIN_SPEED = self.motor.min_speed
        self.MOTOR_DEFAULT_SPEED = self.motor.default_speed
        self.MOTOR_TURN_SPEED = self.motor.turn_speed
        self.MOTOR_PWM_RESOLUTION = self.motor.pwm_resolution
        self.MOTOR_SPEED_MULTIPLIER = self.motor.speed_multiplier
        self.MAX_SPEED_LIMIT = self.motor.max_speed_limit
        self.EMERGENCY_STOP_DELAY = self.motor.emergency_stop_delay
        
        self.IMU_I2C_ADDRESS = self.sensor.imu_i2c_address
        self.IMU_SAMPLE_RATE = self.sensor.imu_sample_rate
        self.IMU_GYRO_RANGE = self.sensor.imu_gyro_range
        self.IMU_ACCEL_RANGE = self.sensor.imu_accel_range
        self.IMU_MAG_RANGE = self.sensor.imu_mag_range
        self.ANGLE_CONTROL_FREQUENCY = self.sensor.angle_control_frequency
        self.ANGLE_TOLERANCE = self.sensor.angle_tolerance
        self.ANGLE_MAX_ERROR = self.sensor.angle_max_error
        self.ANGLE_PID_KP = self.sensor.angle_pid_kp
        self.ANGLE_PID_KI = self.sensor.angle_pid_ki
        self.ANGLE_PID_KD = self.sensor.angle_pid_kd
        self.ANGLE_PID_INTEGRAL_LIMIT = self.sensor.angle_pid_integral_limit
        self.ANGLE_PID_OUTPUT_LIMIT = self.sensor.angle_pid_output_limit
        self.ROTATION_90_DEGREES = self.sensor.rotation_90_degrees
        self.ROTATION_180_DEGREES = self.sensor.rotation_180_degrees
        self.ROTATION_360_DEGREES = self.sensor.rotation_360_degrees
        self.SENSOR_SYNC_TARGET_SAMPLES = self.sensor.sensor_sync_target_samples
        self.SENSOR_SYNC_SLEEP_MS = self.sensor.sensor_sync_sleep_ms
        self.SENSOR_SYNC_TIMEOUT_MS = self.sensor.sensor_sync_timeout_ms
        self.I2C_BUS = self.sensor.i2c_bus
        self.GPIO_BASE_PIN = self.sensor.gpio_base_pin
        
    def get_mqtt_config(self) -> Dict[str, Any]:
        return {
            'broker': self.MQTT_BROKER,
            'port': self.MQTT_PORT,
            'timeout': self.MQTT_TIMEOUT,
            'keepalive': self.MQTT_KEEPALIVE,
            'username': self.MQTT_USERNAME,
            'password': self.MQTT_PASSWORD
        }
    
    def get_local_mqtt_config(self) -> Dict[str, Any]:
        return {
            'broker': self.LOCAL_MQTT_BROKER,
            'port': self.LOCAL_MQTT_PORT,
            'timeout': self.MQTT_TIMEOUT,
            'keepalive': self.MQTT_KEEPALIVE
        }
    
    def get_motor_config(self) -> Dict[str, Any]:
        return {
            'i2c_address': self.MOTOR_I2C_ADDRESS,
            'pwm_frequency': self.MOTOR_PWM_FREQUENCY,
            'max_speed': self.MOTOR_MAX_SPEED,
            'min_speed': self.MOTOR_MIN_SPEED,
            'default_speed': self.MOTOR_DEFAULT_SPEED,
            'turn_speed': self.MOTOR_TURN_SPEED,
            'pwm_resolution': self.MOTOR_PWM_RESOLUTION,
            'speed_multiplier': self.MOTOR_SPEED_MULTIPLIER
        }
    
    def get_imu_config(self) -> Dict[str, Any]:
        return {
            'i2c_address': self.IMU_I2C_ADDRESS,
            'sample_rate': self.IMU_SAMPLE_RATE,
            'gyro_range': self.IMU_GYRO_RANGE,
            'accel_range': self.IMU_ACCEL_RANGE,
            'mag_range': self.IMU_MAG_RANGE
        }
    
    def get_angle_control_config(self) -> Dict[str, Any]:
        return {
            'frequency': self.ANGLE_CONTROL_FREQUENCY,
            'tolerance': self.ANGLE_TOLERANCE,
            'max_error': self.ANGLE_MAX_ERROR,
            'pid_kp': self.ANGLE_PID_KP,
            'pid_ki': self.ANGLE_PID_KI,
            'pid_kd': self.ANGLE_PID_KD,
            'pid_integral_limit': self.ANGLE_PID_INTEGRAL_LIMIT,
            'pid_output_limit': self.ANGLE_PID_OUTPUT_LIMIT
        }
    
    def get_rotation_config(self) -> Dict[str, Any]:
        return {
            'rotation_90': self.ROTATION_90_DEGREES,
            'rotation_180': self.ROTATION_180_DEGREES,
            'rotation_360': self.ROTATION_360_DEGREES
        }
    
    def get_sensor_sync_config(self) -> Dict[str, Any]:
        return {
            'target_samples': self.SENSOR_SYNC_TARGET_SAMPLES,
            'sleep_ms': self.SENSOR_SYNC_SLEEP_MS,
            'timeout_ms': self.SENSOR_SYNC_TIMEOUT_MS
        }
    
    def get_communication_config(self) -> Dict[str, Any]:
        return {
            'websocket_port': self.WEBSOCKET_PORT,
            'http_port': self.HTTP_PORT,
            'timeout': self.COMMUNICATION_TIMEOUT,
            'network_timeout': self.NETWORK_TIMEOUT,
            'retry_count': self.NETWORK_RETRY_COUNT,
            'retry_delay': self.NETWORK_RETRY_DELAY
        }
    
    def get_logging_config(self) -> Dict[str, Any]:
        return {
            'level': self.LOG_LEVEL,
            'format': self.LOG_FORMAT,
            'file': self.LOG_FILE
        }
    
    def get_debug_config(self) -> Dict[str, Any]:
        return {
            'enabled': self.DEBUG_MODE,
            'counter_limit': self.DEBUG_COUNTER_LIMIT,
            'print_interval': self.DEBUG_PRINT_INTERVAL
        }
    
    def get_performance_config(self) -> Dict[str, Any]:
        return {
            'control_loop_frequency': self.CONTROL_LOOP_FREQUENCY,
            'control_loop_period_ms': self.CONTROL_LOOP_PERIOD_MS
        }
    
    def get_safety_config(self) -> Dict[str, Any]:
        return {
            'max_speed_limit': self.MAX_SPEED_LIMIT,
            'emergency_stop_delay': self.EMERGENCY_STOP_DELAY,
            'battery_low_threshold': self.BATTERY_LOW_THRESHOLD
        }
    
    def get_hardware_config(self) -> Dict[str, Any]:
        return {
            'i2c_bus': self.I2C_BUS,
            'gpio_base_pin': self.GPIO_BASE_PIN
        }
    
    def get_test_config(self) -> Dict[str, Any]:
        return {
            'duration': self.TEST_DURATION,
            'speed': self.TEST_SPEED,
            'curvature': self.TEST_CURVATURE
        }
    
    def get_backup_config(self) -> Dict[str, Any]:
        return {
            'interval': self.BACKUP_INTERVAL,
            'retention_days': self.BACKUP_RETENTION_DAYS
        }
    
    def update_from_env(self):
        if os.getenv('MQTT_BROKER'):
            self.MQTT_BROKER = os.getenv('MQTT_BROKER')
        if os.getenv('MQTT_PORT'):
            self.MQTT_PORT = int(os.getenv('MQTT_PORT'))
        
        if os.getenv('SYSTEM_NAME'):
            self.SYSTEM_NAME = os.getenv('SYSTEM_NAME')
        
        if os.getenv('DEBUG_MODE'):
            self.DEBUG_MODE = os.getenv('DEBUG_MODE').lower() == 'true'
    
    def print_config(self):
        print("=== AMR System Configuration ===")
        print(f"System Name: {self.SYSTEM_NAME}")
        print(f"Debug Mode: {self.DEBUG_MODE}")
        print(f"MQTT Broker: {self.MQTT_BROKER}:{self.MQTT_PORT}")
        print(f"Motor I2C Address: 0x{self.MOTOR_I2C_ADDRESS:02X}")
        print(f"IMU I2C Address: 0x{self.IMU_I2C_ADDRESS:02X}")
        print(f"Motor Max Speed: {self.MOTOR_MAX_SPEED}")
        print(f"Control Frequency: {self.CONTROL_LOOP_FREQUENCY} Hz")
        print("================================")

config = SystemConfig()

config.update_from_env()

def get_config() -> SystemConfig:
    return config

def get_mqtt_config() -> Dict[str, Any]:
    return config.get_mqtt_config()

def get_motor_config() -> Dict[str, Any]:
    return config.get_motor_config()

def get_imu_config() -> Dict[str, Any]:
    return config.get_imu_config()

def get_angle_control_config() -> Dict[str, Any]:
    return config.get_angle_control_config()

if __name__ == "__main__":
    config.print_config()
    
    print("\nMQTT Config:", config.get_mqtt_config())
    print("Motor Config:", config.get_motor_config())
    print("IMU Config:", config.get_imu_config())
