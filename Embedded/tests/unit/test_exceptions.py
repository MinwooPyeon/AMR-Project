import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from utilities.exceptions import (
    AMRBaseException, AMRConnectionError, AMRConfigError, AMRSecurityError,
    AMRHardwareError, AMRCommunicationError, AMRValidationError,
    AMRTimeoutError, AMRResourceError, MQTTConnectionError, MotorControlError,
    SensorError, AuthenticationError, AuthorizationError,
    ConfigurationNotFoundError, InvalidConfigurationError,
    DataValidationError, NetworkTimeoutError, ResourceUnavailableError
)


class TestAMRBaseException(unittest.TestCase):
    
    def test_base_exception_creation(self):
        exception = AMRBaseException("Test message")
        self.assertEqual(exception.message, "Test message")
        self.assertIsNone(exception.error_code)
        self.assertEqual(exception.details, {})
    
    def test_base_exception_with_error_code(self):
        exception = AMRBaseException("Test message", "TEST_ERROR")
        self.assertEqual(exception.message, "Test message")
        self.assertEqual(exception.error_code, "TEST_ERROR")
    
    def test_base_exception_with_details(self):
        details = {"key": "value"}
        exception = AMRBaseException("Test message", details=details)
        self.assertEqual(exception.details, details)
    
    def test_base_exception_string_representation(self):
        exception = AMRBaseException("Test message", "TEST_ERROR")
        self.assertEqual(str(exception), "[TEST_ERROR] Test message")
        
        exception_no_code = AMRBaseException("Test message")
        self.assertEqual(str(exception_no_code), "Test message")


class TestConnectionExceptions(unittest.TestCase):
    
    def test_connection_error(self):
        exception = AMRConnectionError("MQTT", "Connection failed")
        self.assertEqual(exception.service, "MQTT")
        self.assertEqual(exception.message, "MQTT connection error: Connection failed")
    
    def test_mqtt_connection_error(self):
        exception = MQTTConnectionError("Connection failed", "localhost", 1883)
        self.assertEqual(exception.broker, "localhost")
        self.assertEqual(exception.port, 1883)
        self.assertEqual(exception.error_code, "MQTT_CONNECTION_ERROR")


class TestConfigExceptions(unittest.TestCase):
    
    def test_config_error(self):
        exception = AMRConfigError("system", "Invalid configuration")
        self.assertEqual(exception.config_name, "system")
        self.assertEqual(exception.message, "Configuration error in system: Invalid configuration")
    
    def test_configuration_not_found_error(self):
        exception = ConfigurationNotFoundError("config.yaml")
        self.assertEqual(exception.config_path, "config.yaml")
        self.assertEqual(exception.error_code, "CONFIG_NOT_FOUND")
    
    def test_invalid_configuration_error(self):
        exception = InvalidConfigurationError("system", "port", "invalid")
        self.assertEqual(exception.config_name, "system")
        self.assertEqual(exception.field, "port")
        self.assertEqual(exception.value, "invalid")


class TestSecurityExceptions(unittest.TestCase):
    
    def test_security_error(self):
        exception = AMRSecurityError("Authentication", "Invalid credentials")
        self.assertEqual(exception.security_type, "Authentication")
        self.assertEqual(exception.message, "Security error (Authentication): Invalid credentials")
    
    def test_authentication_error(self):
        exception = AuthenticationError("Invalid password", "user123")
        self.assertEqual(exception.username, "user123")
        self.assertEqual(exception.error_code, "AUTH_ERROR")
    
    def test_authorization_error(self):
        exception = AuthorizationError("Access denied", "user123", "read")
        self.assertEqual(exception.user, "user123")
        self.assertEqual(exception.action, "read")
        self.assertEqual(exception.error_code, "AUTHZ_ERROR")


class TestHardwareExceptions(unittest.TestCase):
    
    def test_hardware_error(self):
        exception = AMRHardwareError("Motor", "Motor not responding")
        self.assertEqual(exception.device, "Motor")
        self.assertEqual(exception.message, "Hardware error in Motor: Motor not responding")
    
    def test_motor_control_error(self):
        exception = MotorControlError("Speed control failed", "motor1")
        self.assertEqual(exception.motor_id, "motor1")
        self.assertEqual(exception.error_code, "MOTOR_CONTROL_ERROR")
    
    def test_sensor_error(self):
        exception = SensorError("IMU not responding", "IMU")
        self.assertEqual(exception.sensor_type, "IMU")
        self.assertEqual(exception.error_code, "SENSOR_ERROR")


class TestCommunicationExceptions(unittest.TestCase):
    
    def test_communication_error(self):
        exception = AMRCommunicationError("MQTT", "Message send failed")
        self.assertEqual(exception.protocol, "MQTT")
        self.assertEqual(exception.message, "Communication error (MQTT): Message send failed")


class TestValidationExceptions(unittest.TestCase):
    
    def test_validation_error(self):
        exception = AMRValidationError("speed", "Speed must be positive")
        self.assertEqual(exception.field, "speed")
        self.assertEqual(exception.message, "Validation error in speed: Speed must be positive")
    
    def test_data_validation_error(self):
        exception = DataValidationError("speed", "float", "invalid")
        self.assertEqual(exception.field, "speed")
        self.assertEqual(exception.expected_type, "float")
        self.assertEqual(exception.actual_value, "invalid")
        self.assertEqual(exception.error_code, "DATA_VALIDATION_ERROR")


class TestTimeoutExceptions(unittest.TestCase):
    
    def test_timeout_error(self):
        exception = AMRTimeoutError("connection", 30.0)
        self.assertEqual(exception.operation, "connection")
        self.assertEqual(exception.timeout, 30.0)
        self.assertEqual(exception.message, "Timeout error in connection after 30.0s")
    
    def test_network_timeout_error(self):
        exception = NetworkTimeoutError("connection", 30.0, "localhost")
        self.assertEqual(exception.host, "localhost")
        self.assertEqual(exception.error_code, "NETWORK_TIMEOUT")


class TestResourceExceptions(unittest.TestCase):
    
    def test_resource_error(self):
        exception = AMRResourceError("memory", "Out of memory")
        self.assertEqual(exception.resource, "memory")
        self.assertEqual(exception.message, "Resource error in memory: Out of memory")
    
    def test_resource_unavailable_error(self):
        exception = ResourceUnavailableError("GPIO", "Already in use")
        self.assertEqual(exception.reason, "Already in use")
        self.assertEqual(exception.error_code, "RESOURCE_UNAVAILABLE")


if __name__ == '__main__':
    unittest.main()
