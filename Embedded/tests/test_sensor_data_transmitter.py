#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from mqtt.sensor_data_transmitter import SensorDataTransmitter

def test_sensor_data_transmitter():
    transmitter = SensorDataTransmitter("AMR001")
    
    if transmitter.connect_mqtt():
        test_data = {
            "serial": "AMR001",
            "state": "RUNNING",
            "x": "10.5",
            "y": "20.3",
            "speed": "25.0"
        }
        
        success = transmitter.send_sensor_data(test_data)
        if success:
            print("센서 데이터 전송 테스트 성공")
        else:
            print("센서 데이터 전송 테스트 실패")
        
        transmitter.disconnect_mqtt()

if __name__ == "__main__":
    test_sensor_data_transmitter() 