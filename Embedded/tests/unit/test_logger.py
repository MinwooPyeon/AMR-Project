import unittest
import sys
import os
import tempfile
import shutil

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from utilities.logger import AMRLogger, LoggerFactory


class TestAMRLogger(unittest.TestCase):
    
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.test_dir)
    
    def tearDown(self):
        os.chdir(self.original_cwd)
        shutil.rmtree(self.test_dir)
    
    def test_logger_creation(self):
        logger = AMRLogger("test_logger")
        self.assertIsNotNone(logger)
        self.assertEqual(logger.logger.name, "test_logger")
    
    def test_logger_methods(self):
        logger = AMRLogger("test_logger")
        
        self.assertTrue(hasattr(logger, 'info'))
        self.assertTrue(hasattr(logger, 'warn'))
        self.assertTrue(hasattr(logger, 'error'))
        self.assertTrue(hasattr(logger, 'debug'))
        self.assertTrue(hasattr(logger, 'success'))
    
    def test_logger_special_methods(self):
        logger = AMRLogger("test_logger")
        
        self.assertTrue(hasattr(logger, 'connection_success'))
        self.assertTrue(hasattr(logger, 'connection_error'))
        self.assertTrue(hasattr(logger, 'security_alert'))
        self.assertTrue(hasattr(logger, 'performance_metric'))
        self.assertTrue(hasattr(logger, 'mqtt_send_success'))
        self.assertTrue(hasattr(logger, 'mqtt_receive_success'))


class TestLoggerFactory(unittest.TestCase):
    
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.test_dir)
    
    def tearDown(self):
        os.chdir(self.original_cwd)
        shutil.rmtree(self.test_dir)
    
    def test_get_logger(self):
        logger1 = LoggerFactory.get_logger("test_logger")
        logger2 = LoggerFactory.get_logger("test_logger")
        
        self.assertIs(logger1, logger2)
    
    def test_get_module_logger(self):
        logger = LoggerFactory.get_module_logger("mqtt")
        self.assertIsNotNone(logger)
        self.assertEqual(logger.logger.name, "amr.mqtt")
    
    def test_different_loggers(self):
        logger1 = LoggerFactory.get_logger("logger1")
        logger2 = LoggerFactory.get_logger("logger2")
        
        self.assertIsNot(logger1, logger2)
    
    def test_logger_levels(self):
        debug_logger = LoggerFactory.get_logger("debug_logger", "DEBUG")
        info_logger = LoggerFactory.get_logger("info_logger", "INFO")
        warning_logger = LoggerFactory.get_logger("warning_logger", "WARNING")
        error_logger = LoggerFactory.get_logger("error_logger", "ERROR")
        
        self.assertIsNotNone(debug_logger)
        self.assertIsNotNone(info_logger)
        self.assertIsNotNone(warning_logger)
        self.assertIsNotNone(error_logger)


class TestGlobalLoggers(unittest.TestCase):
    
    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.test_dir)
    
    def tearDown(self):
        os.chdir(self.original_cwd)
        shutil.rmtree(self.test_dir)
    
    def test_global_loggers_exist(self):
        from utilities.logger import (
            mqtt_logger, system_logger, security_logger,
            motor_logger, sensor_logger, ai_logger
        )
        
        self.assertIsNotNone(mqtt_logger)
        self.assertIsNotNone(system_logger)
        self.assertIsNotNone(security_logger)
        self.assertIsNotNone(motor_logger)
        self.assertIsNotNone(sensor_logger)
        self.assertIsNotNone(ai_logger)
    
    def test_global_logger_names(self):
        from utilities.logger import (
            mqtt_logger, system_logger, security_logger,
            motor_logger, sensor_logger, ai_logger
        )
        
        self.assertEqual(mqtt_logger.logger.name, "amr.mqtt")
        self.assertEqual(system_logger.logger.name, "amr.system")
        self.assertEqual(security_logger.logger.name, "amr.security")
        self.assertEqual(motor_logger.logger.name, "amr.motor")
        self.assertEqual(sensor_logger.logger.name, "amr.sensor")
        self.assertEqual(ai_logger.logger.name, "amr.ai")


if __name__ == '__main__':
    unittest.main()
