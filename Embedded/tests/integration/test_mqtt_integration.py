#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MQTT 통합 테스트
"""

import unittest
import sys
import os
import time
import threading
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from mqtt_module.base_mqtt_client import BaseMQTTClient
from mqtt_module.ai_mqtt_client import AIMQTTClient
from utilities.logger import LoggerFactory


class MockMQTTClient(BaseMQTTClient):
    """테스트용 Mock MQTT 클라이언트"""
    
    def setup_subscriptions(self):
        self.subscribe("test/topic", callback=self._test_callback)
    
    def process_message(self, topic: str, payload: str):
        self.logger.info(f"Mock client received: {topic} - {payload}")
    
    def _test_callback(self, topic: str, payload: str):
        self.logger.info(f"Test callback: {topic} - {payload}")


class TestMQTTIntegration(unittest.TestCase):
    """MQTT 통합 테스트"""
    
    def setUp(self):
        self.logger = LoggerFactory.get_module_logger("test.mqtt")
        self.mock_client = MockMQTTClient("test_client")
    
    def test_base_client_initialization(self):
        """기본 MQTT 클라이언트 초기화 테스트"""
        self.assertIsNotNone(self.mock_client)
        self.assertEqual(self.mock_client.client_id, "test_client")
        self.assertFalse(self.mock_client.connected)
    
    def test_ai_client_initialization(self):
        """AI MQTT 클라이언트 초기화 테스트"""
        ai_client = AIMQTTClient("test_robot")
        self.assertIsNotNone(ai_client)
        self.assertEqual(ai_client.robot_id, "test_robot")
    
    def test_logger_integration(self):
        """로거 통합 테스트"""
        self.assertIsNotNone(self.mock_client.logger)
        self.assertEqual(self.mock_client.logger.logger.name, "amr.mqtt")
    
    def test_client_stats(self):
        """클라이언트 통계 테스트"""
        stats = self.mock_client.get_stats()
        self.assertIn('total_received', stats)
        self.assertIn('total_sent', stats)
        self.assertIn('last_received_time', stats)
        self.assertIn('last_sent_time', stats)
    
    def test_connection_status(self):
        """연결 상태 테스트"""
        status = self.mock_client.get_connection_status()
        self.assertIn('connected', status)
        self.assertIn('client_id', status)
        self.assertIn('broker', status)
        self.assertIn('port', status)


if __name__ == '__main__':
    unittest.main()
