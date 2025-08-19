# AMR 시스템 중앙 설정 파일

import os
from typing import Dict, Any

class SystemConfig:
    
    def __init__(self):
        # 기본 시스템 설정
        self.SYSTEM_NAME = "AMR001"
        self.DEBUG_MODE = True
        
        # MQTT 설정
        self.MQTT_BROKER = "192.168.100.141"
        self.MQTT_PORT = 1883
        self.MQTT_TIMEOUT = 60
        self.MQTT_KEEPALIVE = 60
        self.MQTT_USERNAME = "minwoo"
        self.MQTT_PASSWORD = "minwoo"
        
        # 로컬 MQTT 설정 (process_manager용)
        self.LOCAL_MQTT_BROKER = "localhost"
        self.LOCAL_MQTT_PORT = 1883
        
        # 모터 제어 설정
        self.MOTOR_I2C_ADDRESS = 0x40
        self.MOTOR_PWM_FREQUENCY = 50
        self.MOTOR_MAX_SPEED = 100
        self.MOTOR_MIN_SPEED = 10
        self.MOTOR_DEFAULT_SPEED = 50
        self.MOTOR_TURN_SPEED = 40
        self.MOTOR_PWM_RESOLUTION = 4095
        self.MOTOR_SPEED_MULTIPLIER = 40.95
        
        # IMU 센서 설정
        self.IMU_I2C_ADDRESS = 0x4B
        self.IMU_SAMPLE_RATE = 100  
        self.IMU_GYRO_RANGE = 250   
        self.IMU_ACCEL_RANGE = 2    
        self.IMU_MAG_RANGE = 1300   
        
        # 각도 제어 설정
        self.ANGLE_CONTROL_FREQUENCY = 50.0  
        self.ANGLE_TOLERANCE = 1.0  
        self.ANGLE_MAX_ERROR = 180.0
        self.ANGLE_PID_KP = 2.0
        self.ANGLE_PID_KI = 0.1
        self.ANGLE_PID_KD = 0.5
        self.ANGLE_PID_INTEGRAL_LIMIT = 100.0
        self.ANGLE_PID_OUTPUT_LIMIT = 100.0
        
        # 회전 각도 설정
        self.ROTATION_90_DEGREES = 90.0
        self.ROTATION_180_DEGREES = 180.0
        self.ROTATION_360_DEGREES = 360.0
        
        # 센서 동기화 설정
        self.SENSOR_SYNC_TARGET_SAMPLES = 300
        self.SENSOR_SYNC_SLEEP_MS = 10
        self.SENSOR_SYNC_TIMEOUT_MS = 10000
        
        # 통신 설정
        self.WEBSOCKET_PORT = 8080
        self.HTTP_PORT = 8000
        self.COMMUNICATION_TIMEOUT = 10
        
        # 로깅 설정
        self.LOG_LEVEL = "INFO"
        self.LOG_FORMAT = "[%(asctime)s] %(levelname)s: %(message)s"
        self.LOG_FILE = "logs/amr_system.log"
        
        # 디버깅 설정
        self.DEBUG_COUNTER_LIMIT = 100
        self.DEBUG_PRINT_INTERVAL = 1.0  

        # 성능 설정
        self.CONTROL_LOOP_FREQUENCY = 50.0  
        self.CONTROL_LOOP_PERIOD_MS = 20    
        
        # 안전 설정
        self.MAX_SPEED_LIMIT = 80.0
        self.EMERGENCY_STOP_DELAY = 0.1  
        self.BATTERY_LOW_THRESHOLD = 20.0  
        
        # 하드웨어 주소 설정
        self.I2C_BUS = 1
        self.GPIO_BASE_PIN = 18
        
        # 테스트 설정
        self.TEST_DURATION = 30  
        self.TEST_SPEED = 50
        self.TEST_CURVATURE = 0.3
        
        # 백업 설정
        self.BACKUP_INTERVAL = 3600  
        self.BACKUP_RETENTION_DAYS = 7
        
        # 네트워크 설정
        self.NETWORK_TIMEOUT = 5.0
        self.NETWORK_RETRY_COUNT = 3
        self.NETWORK_RETRY_DELAY = 1.0
        
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
