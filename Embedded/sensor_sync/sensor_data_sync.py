#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
센서 데이터 동기화
AMR의 다양한 센서 데이터를 수집하고 동기화하는 시스템
"""

import time
import threading
import logging
import json
from typing import Dict, Optional, Callable
from enum import Enum

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SensorType(Enum):
    """센서 타입 열거형"""
    LIDAR = "lidar"
    CAMERA = "camera"
    IMU = "imu"
    MOTOR_STATUS = "motor_status"

class SensorDataSync:
    """센서 데이터 동기화 클래스"""
    
    def __init__(self, robot_id: str = "AMR001"):
        self.robot_id = robot_id
        
        # 센서 데이터 저장
        self.sensor_data = {
            SensorType.LIDAR: {},
            SensorType.CAMERA: {},
            SensorType.IMU: {},
            SensorType.MOTOR_STATUS: {}
        }
        self.data_lock = threading.Lock()
        
        # 동기화 상태
        self.sync_running = False
        self.sync_thread = None
        self.sync_interval = 0.1  # 100ms마다 동기화
        
        # 콜백 함수
        self.data_callback = None
        
        # 통계 정보
        self.stats_lock = threading.Lock()
        self.total_sync_count = 0
        self.last_sync_time = 0
        
        logger.info(f"Sensor Data Sync 초기화 완료 - Robot ID: {robot_id}")
    
    def update_sensor_data(self, sensor_type: SensorType, data: Dict):
        """센서 데이터 업데이트"""
        with self.data_lock:
            self.sensor_data[sensor_type] = data
        logger.debug(f"센서 데이터 업데이트: {sensor_type.value}")
    
    def get_sensor_data(self, sensor_type: SensorType) -> Dict:
        """센서 데이터 반환"""
        with self.data_lock:
            return self.sensor_data[sensor_type].copy()
    
    def get_all_sensor_data(self) -> Dict:
        """모든 센서 데이터 반환"""
        with self.data_lock:
            return {
                sensor_type.value: data.copy()
                for sensor_type, data in self.sensor_data.items()
            }
    
    def set_data_callback(self, callback: Callable[[Dict], None]):
        """데이터 콜백 함수 설정"""
        self.data_callback = callback
    
    def start_sync(self):
        """센서 데이터 동기화 시작"""
        if self.sync_running:
            logger.warning("센서 데이터 동기화가 이미 실행 중입니다")
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._sync_worker, daemon=True)
        self.sync_thread.start()
        logger.info("센서 데이터 동기화 시작")
    
    def stop_sync(self):
        """센서 데이터 동기화 중지"""
        self.sync_running = False
        if self.sync_thread:
            self.sync_thread.join(timeout=1.0)
        logger.info("센서 데이터 동기화 중지")
    
    def _sync_worker(self):
        """센서 데이터 동기화 워커"""
        while self.sync_running:
            try:
                # 모든 센서 데이터 수집
                with self.data_lock:
                    sync_data = {
                        "robot_id": self.robot_id,
                        "timestamp": time.time(),
                        "sensors": {
                            sensor_type.value: data
                            for sensor_type, data in self.sensor_data.items()
                        }
                    }
                
                # 통계 업데이트
                with self.stats_lock:
                    self.total_sync_count += 1
                    self.last_sync_time = time.time()
                
                # 콜백 호출
                if self.data_callback:
                    self.data_callback(sync_data)
                
                time.sleep(self.sync_interval)
                
            except Exception as e:
                logger.error(f"센서 데이터 동기화 오류: {e}")
                time.sleep(self.sync_interval)
    
    def create_data_packet(self) -> Dict:
        """백엔드 전송용 데이터 패킷 생성"""
        with self.data_lock:
            # 모터 상태 데이터
            motor_data = self.sensor_data[SensorType.MOTOR_STATUS]
            left_speed = motor_data.get('left_speed', 0.0)
            right_speed = motor_data.get('right_speed', 0.0)
            average_speed = (abs(left_speed) + abs(right_speed)) / 2.0
            
            # 위치 데이터 (IMU 또는 기본값)
            imu_data = self.sensor_data[SensorType.IMU]
            x = imu_data.get('x', 0.0)
            y = imu_data.get('y', 0.0)
            
            # 백엔드 JSON 구조
            data_packet = {
                "serial": self.robot_id,
                "state": "RUNNING",
                "x": str(x),
                "y": str(y),
                "speed": str(average_speed)
            }
            
            return data_packet
    
    def get_sync_stats(self) -> Dict:
        """동기화 통계 반환"""
        with self.stats_lock:
            return {
                "total_sync_count": self.total_sync_count,
                "last_sync_time": self.last_sync_time,
                "sync_running": self.sync_running
            }

def test_sensor_data_sync():
    """센서 데이터 동기화 테스트"""
    sensor_sync = SensorDataSync("AMR001")
    
    def data_callback(sync_data):
        print(f"센서 데이터 동기화: {sync_data}")
    
    sensor_sync.set_data_callback(data_callback)
    sensor_sync.start_sync()
    
    # 테스트 데이터 업데이트
    time.sleep(1)
    sensor_sync.update_sensor_data(SensorType.MOTOR_STATUS, {
        "left_speed": 25.0,
        "right_speed": 25.0
    })
    
    time.sleep(1)
    sensor_sync.update_sensor_data(SensorType.IMU, {
        "x": 10.5,
        "y": 20.3,
        "heading": 45.0
    })
    
    time.sleep(1)
    sensor_sync.update_sensor_data(SensorType.LIDAR, {
        "distance": 150.0,
        "angle": 0.0
    })
    
    time.sleep(2)
    
    # 데이터 패킷 생성 테스트
    data_packet = sensor_sync.create_data_packet()
    print(f"데이터 패킷: {data_packet}")
    
    # 통계 출력
    stats = sensor_sync.get_sync_stats()
    print(f"동기화 통계: {stats}")
    
    sensor_sync.stop_sync()

if __name__ == "__main__":
    test_sensor_data_sync() 