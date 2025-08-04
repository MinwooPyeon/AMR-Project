#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 속도 모니터
AMR 모터의 속도를 모니터링하고 제어하는 시스템
"""

import time
import threading
import logging
from typing import Dict, Optional, Callable
from real_motor_controller import RealMotorController

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MotorController:
    """모터 컨트롤러 클래스"""
    
    def __init__(self):
        self.motor_controller = RealMotorController()
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.speed_lock = threading.Lock()
        
        logger.info("Motor Controller 초기화 완료")
    
    def set_speeds(self, left_speed: float, right_speed: float):
        """좌우 모터 속도 설정"""
        with self.speed_lock:
            self.left_speed = left_speed
            self.right_speed = right_speed
        
        # 실제 모터 제어
        self.motor_controller.set_motor_speeds(left_speed, right_speed)
        logger.debug(f"모터 속도 설정: 좌측={left_speed}, 우측={right_speed}")
    
    def get_speeds(self) -> Dict[str, float]:
        """현재 모터 속도 반환"""
        with self.speed_lock:
            return {
                "left_speed": self.left_speed,
                "right_speed": self.right_speed
            }
    
    def move_forward(self, speed: float = 50.0):
        """전진"""
        self.set_speeds(speed, speed)
        logger.info(f"전진 명령 실행: 속도={speed}")
    
    def move_backward(self, speed: float = 50.0):
        """후진"""
        self.set_speeds(-speed, -speed)
        logger.info(f"후진 명령 실행: 속도={speed}")
    
    def turn_left(self, speed: float = 50.0):
        """좌회전"""
        self.set_speeds(-speed, speed)
        logger.info(f"좌회전 명령 실행: 속도={speed}")
    
    def turn_right(self, speed: float = 50.0):
        """우회전"""
        self.set_speeds(speed, -speed)
        logger.info(f"우회전 명령 실행: 속도={speed}")
    
    def stop(self):
        """정지"""
        self.set_speeds(0.0, 0.0)
        logger.info("정지 명령 실행")

class MotorSpeedMonitor:
    """모터 속도 모니터 클래스"""
    
    def __init__(self):
        self.motor_controller = None
        self.monitoring = False
        self.monitor_thread = None
        self.monitor_interval = 0.1  # 100ms마다 모니터링
        
        # 속도 이력
        self.speed_history = []
        self.history_max_size = 100
        
        # 콜백 함수
        self.speed_callback = None
        
        logger.info("Motor Speed Monitor 초기화 완료")
    
    def set_motor_controller(self, motor_controller: MotorController):
        """모터 컨트롤러 설정"""
        self.motor_controller = motor_controller
        logger.info("모터 컨트롤러 설정 완료")
    
    def set_speed_callback(self, callback: Callable[[Dict], None]):
        """속도 콜백 함수 설정"""
        self.speed_callback = callback
    
    def start_monitoring(self):
        """모터 속도 모니터링 시작"""
        if self.monitoring:
            logger.warning("모터 속도 모니터링이 이미 실행 중입니다")
            return
        
        if not self.motor_controller:
            logger.error("모터 컨트롤러가 설정되지 않았습니다")
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_worker, daemon=True)
        self.monitor_thread.start()
        logger.info("모터 속도 모니터링 시작")
    
    def stop_monitoring(self):
        """모터 속도 모니터링 중지"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        logger.info("모터 속도 모니터링 중지")
    
    def _monitor_worker(self):
        """모터 속도 모니터링 워커"""
        while self.monitoring:
            try:
                if self.motor_controller:
                    speeds = self.motor_controller.get_speeds()
                    
                    # 속도 이력에 추가
                    timestamp = time.time()
                    speed_data = {
                        "timestamp": timestamp,
                        "left_speed": speeds["left_speed"],
                        "right_speed": speeds["right_speed"],
                        "average_speed": (abs(speeds["left_speed"]) + abs(speeds["right_speed"])) / 2.0
                    }
                    
                    self.speed_history.append(speed_data)
                    
                    # 이력 크기 제한
                    if len(self.speed_history) > self.history_max_size:
                        self.speed_history.pop(0)
                    
                    # 콜백 호출
                    if self.speed_callback:
                        self.speed_callback(speed_data)
                
                time.sleep(self.monitor_interval)
                
            except Exception as e:
                logger.error(f"모터 속도 모니터링 오류: {e}")
                time.sleep(self.monitor_interval)
    
    def get_current_speeds(self) -> Dict:
        """현재 모터 속도 반환"""
        if self.motor_controller:
            return self.motor_controller.get_speeds()
        return {"left_speed": 0.0, "right_speed": 0.0}
    
    def get_speed_history(self) -> list:
        """속도 이력 반환"""
        return self.speed_history.copy()
    
    def get_speed_stats(self) -> Dict:
        """속도 통계 반환"""
        if not self.speed_history:
            return {
                "current_speed": 0.0,
                "max_speed": 0.0,
                "avg_speed": 0.0,
                "total_samples": 0
            }
        
        current_speed = self.speed_history[-1]["average_speed"]
        max_speed = max([data["average_speed"] for data in self.speed_history])
        avg_speed = sum([data["average_speed"] for data in self.speed_history]) / len(self.speed_history)
        
        return {
            "current_speed": current_speed,
            "max_speed": max_speed,
            "avg_speed": avg_speed,
            "total_samples": len(self.speed_history)
        }

def test_motor_speed_monitor():
    """모터 속도 모니터 테스트"""
    motor_controller = MotorController()
    monitor = MotorSpeedMonitor()
    monitor.set_motor_controller(motor_controller)
    
    def speed_callback(speed_data):
        print(f"속도 업데이트: {speed_data}")
    
    monitor.set_speed_callback(speed_callback)
    monitor.start_monitoring()
    
    # 테스트 동작
    time.sleep(1)
    motor_controller.move_forward(30.0)
    time.sleep(2)
    motor_controller.turn_left(20.0)
    time.sleep(2)
    motor_controller.stop()
    time.sleep(1)
    
    # 통계 출력
    stats = monitor.get_speed_stats()
    print(f"모터 속도 통계: {stats}")
    
    monitor.stop_monitoring()

if __name__ == "__main__":
    test_motor_speed_monitor() 