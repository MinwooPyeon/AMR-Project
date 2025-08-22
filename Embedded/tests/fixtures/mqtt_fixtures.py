import pytest
import json
from unittest.mock import Mock, MagicMock
from mqtt_module.base_mqtt_client import BaseMQTTClient


@pytest.fixture
def mock_mqtt_client():
    client = Mock(spec=BaseMQTTClient)
    client.client_id = "test_client"
    client.connected = False
    client.broker = "localhost"
    client.port = 1883
    
    client.connect.return_value = True
    client.disconnect.return_value = True
    client.publish.return_value = True
    client.subscribe.return_value = True
    
    client.get_stats.return_value = {
        'total_received': 0,
        'total_sent': 0,
        'last_received_time': None,
        'last_sent_time': None
    }
    
    client.get_connection_status.return_value = {
        'connected': False,
        'client_id': 'test_client',
        'broker': 'localhost',
        'port': 1883
    }
    
    return client


@pytest.fixture
def sample_mqtt_message():
    return {
        'topic': 'test/sensor/data',
        'payload': json.dumps({
            'sensor_id': 'imu_001',
            'timestamp': 1234567890,
            'data': {
                'acceleration': {'x': 0.1, 'y': 0.2, 'z': 9.8},
                'gyroscope': {'x': 0.01, 'y': 0.02, 'z': 0.03},
                'temperature': 25.5
            }
        })
    }


@pytest.fixture
def sample_robot_status():
    return {
        'robot_id': 'amr_001',
        'timestamp': 1234567890,
        'status': 'idle',
        'battery_level': 85.5,
        'position': {'x': 10.5, 'y': 20.3, 'theta': 0.785},
        'sensors': {
            'imu': 'connected',
            'lidar': 'connected',
            'camera': 'connected'
        }
    }


@pytest.fixture
def mock_config():
    config = Mock()
    config.LOCAL_MQTT_BROKER = "localhost"
    config.LOCAL_MQTT_PORT = 1883
    config.MQTT_USERNAME = "test_user"
    config.MQTT_PASSWORD = "test_password"
    config.MQTT_KEEPALIVE = 60
    config.MQTT_TIMEOUT = 10
    return config


@pytest.fixture
def mock_logger():
    logger = Mock()
    logger.info = Mock()
    logger.error = Mock()
    logger.warning = Mock()
    logger.debug = Mock()
    logger.success = Mock()
    return logger
