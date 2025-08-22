import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from config.system_config import SystemConfig
from config.motor_config import MotorConfig
from config.mqtt_config import MQTTConfig
from config.sensor_config import SensorConfig


class TestSystemConfig(unittest.TestCase):
    
    def setUp(self):
        self.config = SystemConfig()
    
    def test_system_config_initialization(self):
        self.assertEqual(self.config.SYSTEM_NAME, "AMR001")
        self.assertTrue(self.config.DEBUG_MODE)
        self.assertEqual(self.config.WEBSOCKET_PORT, 8080)
        self.assertEqual(self.config.HTTP_PORT, 8000)
    
    def test_mqtt_config_integration(self):
        self.assertEqual(self.config.MQTT_BROKER, "192.168.100.141")
        self.assertEqual(self.config.MQTT_PORT, 1883)
        self.assertEqual(self.config.MQTT_USERNAME, "minwoo")
        self.assertEqual(self.config.MQTT_PASSWORD, "minwoo")
    
    def test_motor_config_integration(self):
        self.assertEqual(self.config.MOTOR_I2C_ADDRESS, 0x40)
        self.assertEqual(self.config.MOTOR_MAX_SPEED, 100)
        self.assertEqual(self.config.MOTOR_DEFAULT_SPEED, 50)
    
    def test_sensor_config_integration(self):
        self.assertEqual(self.config.IMU_I2C_ADDRESS, 0x4B)
        self.assertEqual(self.config.IMU_SAMPLE_RATE, 100)
        self.assertEqual(self.config.ANGLE_CONTROL_FREQUENCY, 50.0)


class TestMotorConfig(unittest.TestCase):
    
    def setUp(self):
        self.motor_config = MotorConfig()
    
    def test_motor_config_defaults(self):
        self.assertEqual(self.motor_config.i2c_address, 0x40)
        self.assertEqual(self.motor_config.max_speed, 100)
        self.assertEqual(self.motor_config.default_speed, 50)
        self.assertEqual(self.motor_config.speed_multiplier, 40.95)
    
    def test_motor_config_to_dict(self):
        config_dict = self.motor_config.to_dict()
        self.assertIn('i2c_address', config_dict)
        self.assertIn('max_speed', config_dict)
        self.assertIn('default_speed', config_dict)
        self.assertEqual(config_dict['i2c_address'], 0x40)


class TestMQTTConfig(unittest.TestCase):
    
    def setUp(self):
        self.mqtt_config = MQTTConfig()
    
    def test_mqtt_config_defaults(self):
        self.assertEqual(self.mqtt_config.broker, "192.168.100.141")
        self.assertEqual(self.mqtt_config.port, 1883)
        self.assertEqual(self.mqtt_config.username, "minwoo")
        self.assertEqual(self.mqtt_config.password, "minwoo")
    
    def test_mqtt_config_to_dict(self):
        config_dict = self.mqtt_config.to_dict()
        self.assertIn('broker', config_dict)
        self.assertIn('port', config_dict)
        self.assertIn('username', config_dict)
        self.assertEqual(config_dict['broker'], "192.168.100.141")


class TestSensorConfig(unittest.TestCase):
    
    def setUp(self):
        self.sensor_config = SensorConfig()
    
    def test_sensor_config_defaults(self):
        self.assertEqual(self.sensor_config.imu_i2c_address, 0x4B)
        self.assertEqual(self.sensor_config.imu_sample_rate, 100)
        self.assertEqual(self.sensor_config.angle_control_frequency, 50.0)
    
    def test_sensor_config_methods(self):
        imu_config = self.sensor_config.get_imu_config()
        self.assertIn('i2c_address', imu_config)
        self.assertEqual(imu_config['i2c_address'], 0x4B)
        
        angle_config = self.sensor_config.get_angle_control_config()
        self.assertIn('frequency', angle_config)
        self.assertEqual(angle_config['frequency'], 50.0)


if __name__ == '__main__':
    unittest.main()
