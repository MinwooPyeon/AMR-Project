import unittest
import sys
import os
import time
import threading
from unittest.mock import Mock, patch
import subprocess

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from utilities.logger import LoggerFactory
from mqtt_module.base_mqtt_client import BaseMQTTClient
from config.system_config import get_config


class TestSystemIntegration(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        cls.logger = LoggerFactory.get_module_logger("test.e2e")
        cls.config = get_config()
        
    def setUp(self):
        self.logger.info("Setting up E2E test environment")
        
    def test_system_startup_sequence(self):
        self.assertIsNotNone(self.config)
        self.assertIsNotNone(self.config.LOCAL_MQTT_BROKER)
        self.assertIsNotNone(self.config.LOCAL_MQTT_PORT)
        
        self.assertIsNotNone(self.logger)
        
        self.assertTrue(hasattr(self.config, 'LOCAL_MQTT_BROKER'))
        
    def test_module_dependencies(self):
        required_modules = [
            'mqtt_module',
            'config', 
            'utilities',
            'sensors'
        ]
        
        for module in required_modules:
            try:
                __import__(module)
                self.logger.info(f"✓ {module} module available")
            except ImportError as e:
                self.fail(f"Required module {module} not available: {e}")
                
    def test_configuration_consistency(self):
        self.assertIsInstance(self.config.LOCAL_MQTT_PORT, int)
        self.assertGreater(self.config.LOCAL_MQTT_PORT, 0)
        self.assertLess(self.config.LOCAL_MQTT_PORT, 65536)
        
        self.assertIsInstance(self.config.LOCAL_MQTT_BROKER, str)
        self.assertGreater(len(self.config.LOCAL_MQTT_BROKER), 0)
        
    def test_logging_integration(self):
        test_logger = LoggerFactory.get_module_logger("test.integration")
        
        self.assertIsNotNone(test_logger)
        
        self.assertTrue(hasattr(test_logger, 'info'))
        self.assertTrue(hasattr(test_logger, 'error'))
        self.assertTrue(hasattr(test_logger, 'debug'))
        
    def test_mqtt_client_integration(self):
        class TestMQTTClient(BaseMQTTClient):
            def setup_subscriptions(self):
                pass
                
            def process_message(self, topic: str, payload: str):
                pass
        
        client = TestMQTTClient("test_e2e_client")
        
        self.assertEqual(client.client_id, "test_e2e_client")
        self.assertFalse(client.connected)
        
        stats = client.get_stats()
        self.assertIn('total_received', stats)
        self.assertIn('total_sent', stats)
        
    def test_process_manager_integration(self):
        try:
            from process_manager import ProcessManager
            manager = ProcessManager()
            
            self.assertIn('mqtt-broker', manager.modules)
            self.assertIn('sensors', manager.modules)
            self.assertIn('mqtt-module', manager.modules)
            
            self.assertIsInstance(manager.modules['mqtt-broker']['required'], bool)
            
        except ImportError as e:
            self.logger.warning(f"ProcessManager not available: {e}")
            
    def test_error_handling_integration(self):
        from utilities.exceptions import AMRException, MQTTException
        
        self.assertTrue(issubclass(MQTTException, AMRException))
        
        try:
            raise MQTTException("Test MQTT error", error_code="TEST_001")
        except MQTTException as e:
            self.assertEqual(e.error_code, "TEST_001")
            self.assertEqual(str(e), "Test MQTT error")


if __name__ == '__main__':
    unittest.main()
