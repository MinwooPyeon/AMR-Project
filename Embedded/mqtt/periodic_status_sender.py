#!/usr/bin/env python3
"""
주기적 상태 데이터 전송 예시 스크립트
1초마다 현재 로봇 상태를 백엔드로 전송합니다.
"""

import sys
import os
import time
import signal
import threading

# 프로젝트 루트를 Python 경로에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from mqtt.sensor_data_transmitter import SensorDataTransmitter
from utils.logger import mqtt_logger

class PeriodicStatusSender:
    def __init__(self, robot_id: str = "AMR001", broker: str = "192.168.100.141", port: int = 1883):
        self.robot_id = robot_id
        self.transmitter = SensorDataTransmitter(robot_id, broker, port)
        self.running = False
        
        # 시그널 핸들러 설정
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """시그널 핸들러 - 프로그램 종료 시 정리"""
        self.stop()
    
    def start(self):
        """주기적 상태 전송 시작"""
        if not self.transmitter.connect_mqtt():
            return False
        
        # 초기 상태 설정
        self.transmitter.update_embedded_data(
            serial=self.robot_id,
            state="RUNNING",
            x=0.0,
            y=0.0,
            speed=0.0,
            angle=0.0
        )
        
        # 주기적 전송 시작
        if self.transmitter.start_periodic_sending(interval=1.0):
            self.running = True
            
            # 상태 업데이트 시뮬레이션
            self._start_status_simulation()
            
            return True
        else:
            return False
    
    def stop(self):
        """주기적 상태 전송 중지"""
        if self.running:
            self.running = False
            self.transmitter.stop_periodic_sending()
            self.transmitter.disconnect_mqtt()
    
    def _start_status_simulation(self):
        """상태 시뮬레이션 (실제 구현에서는 센서 데이터로 대체)"""
        def simulate_movement():
            x, y = 0.0, 0.0
            speed = 1.0
            angle = 0.0
            
            while self.running:
                # 간단한 움직임 시뮬레이션
                x += speed * 0.1
                y += speed * 0.05
                angle += 5.0
                
                if angle >= 360.0:
                    angle = 0.0
                
                # 상태 업데이트
                self.transmitter.update_embedded_data(
                    x=x,
                    y=y,
                    speed=speed,
                    angle=angle
                )
                
                time.sleep(1.0)
        
        # 시뮬레이션 스레드 시작
        simulation_thread = threading.Thread(target=simulate_movement, daemon=True)
        simulation_thread.start()
    
    def run_forever(self):
        """무한 실행"""
        if self.start():
            try:
                while self.running:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
            finally:
                self.stop()

def main():
    """메인 함수"""
    sender = PeriodicStatusSender()
    sender.run_forever()

if __name__ == "__main__":
    main() 