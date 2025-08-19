#!/usr/bin/env python3
"""
배터리 모니터링 및 관리 시스템
실시간 배터리 상태 감시, 충전 관리, 전력 소비 분석
"""

import os
import sys
import time
import json
import logging
import threading
import math
from typing import Dict, List, Optional, Callable, Any
from pathlib import Path
from datetime import datetime, timedelta
import smbus2 as smbus

# 프로젝트 루트 추가
sys.path.append(str(Path(__file__).parent.parent))
from config.system_config import get_config
from security.security_manager import get_security_manager

class BatteryMonitor:
    """배터리 모니터링 및 관리 클래스"""
    
    def __init__(self):
        self.config = get_config()
        self.security_manager = get_security_manager()
        self.logger = self._setup_logger()
        
        # 배터리 설정
        self.battery_config = {
            'i2c_bus': 1,
            'battery_address': 0x36,  # MAX17048 배터리 게이지
            'voltage_divider_ratio': 2.0,
            'capacity_mah': 3000,  # 3000mAh
            'nominal_voltage': 3.7,  # 3.7V
            'max_voltage': 4.2,     # 4.2V
            'min_voltage': 3.0,     # 3.0V
            'low_battery_threshold': 20.0,  # 20%
            'critical_battery_threshold': 10.0,  # 10%
            'charging_current_threshold': 100,  # 100mA
            'temperature_threshold': 60.0,  # 60°C
            'monitoring_interval': 1.0,  # 1초
            'data_logging_interval': 60.0  # 60초
        }
        
        # 배터리 상태
        self.battery_status = {
            'voltage': 0.0,
            'current': 0.0,
            'percentage': 0.0,
            'temperature': 0.0,
            'is_charging': False,
            'is_full': False,
            'is_low': False,
            'is_critical': False,
            'remaining_capacity': 0.0,
            'full_capacity': 0.0,
            'time_to_empty': 0.0,
            'time_to_full': 0.0,
            'cycle_count': 0,
            'health': 100.0
        }
        
        # 전력 소비 데이터
        self.power_consumption = {
            'current_power': 0.0,
            'average_power': 0.0,
            'peak_power': 0.0,
            'total_energy': 0.0,
            'power_history': [],
            'energy_history': []
        }
        
        # 알림 설정
        self.alerts = {
            'low_battery': False,
            'critical_battery': False,
            'overheating': False,
            'overcurrent': False,
            'charging_fault': False
        }
        
        # 콜백 함수들
        self.status_callbacks = []
        self.alert_callbacks = []
        self.data_callbacks = []
        
        # 스레드 안전을 위한 락
        self.lock = threading.Lock()
        
        # I2C 통신
        self.i2c_bus = None
        
        # 모니터링 스레드
        self.monitoring_thread = None
        self.is_monitoring = False
        
        # 초기화
        self._initialize_battery_monitor()
    
    def _setup_logger(self) -> logging.Logger:
        """로거 설정"""
        logger = logging.getLogger('battery_monitor')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(name)s - %(levelname)s: %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        
        return logger
    
    def _initialize_battery_monitor(self):
        """배터리 모니터 초기화"""
        try:
            # I2C 버스 초기화
            self._setup_i2c()
            
            # 배터리 게이지 초기화
            self._initialize_battery_gauge()
            
            # 초기 배터리 상태 읽기
            self._read_battery_status()
            
            # 모니터링 시작
            self._start_monitoring()
            
            self.logger.info("Battery monitor initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Battery monitor initialization failed: {e}")
            raise
    
    def _setup_i2c(self):
        """I2C 통신 설정"""
        try:
            self.i2c_bus = smbus.SMBus(self.battery_config['i2c_bus'])
            self.logger.info(f"I2C bus {self.battery_config['i2c_bus']} initialized")
        except Exception as e:
            self.logger.error(f"I2C setup failed: {e}")
            # 시뮬레이션 모드로 전환
            self.i2c_bus = None
    
    def _initialize_battery_gauge(self):
        """배터리 게이지 초기화"""
        if self.i2c_bus is None:
            self.logger.info("Running in simulation mode")
            return
        
        try:
            # MAX17048 초기화 (실제 구현에서는 해당 칩의 레지스터 사용)
            # 여기서는 시뮬레이션
            self.logger.info("Battery gauge initialized")
        except Exception as e:
            self.logger.error(f"Battery gauge initialization failed: {e}")
    
    def _start_monitoring(self):
        """배터리 모니터링 시작"""
        self.is_monitoring = True
        self.monitoring_thread = threading.Thread(
            target=self._monitoring_loop,
            daemon=True
        )
        self.monitoring_thread.start()
        self.logger.info("Battery monitoring started")
    
    def _monitoring_loop(self):
        """배터리 모니터링 루프"""
        last_log_time = time.time()
        
        while self.is_monitoring:
            try:
                # 배터리 상태 읽기
                self._read_battery_status()
                
                # 전력 소비 계산
                self._calculate_power_consumption()
                
                # 알림 확인
                self._check_alerts()
                
                # 콜백 함수 호출
                self._notify_status_callbacks()
                
                # 데이터 로깅
                current_time = time.time()
                if current_time - last_log_time >= self.battery_config['data_logging_interval']:
                    self._log_battery_data()
                    last_log_time = current_time
                
                # 모니터링 간격 대기
                time.sleep(self.battery_config['monitoring_interval'])
                
            except Exception as e:
                self.logger.error(f"Battery monitoring error: {e}")
                time.sleep(5)  # 오류 시 5초 대기
    
    def _read_battery_status(self):
        """배터리 상태 읽기"""
        try:
            if self.i2c_bus is None:
                # 시뮬레이션 모드
                self._simulate_battery_status()
            else:
                # 실제 하드웨어에서 읽기
                self._read_battery_hardware()
            
            # 배터리 상태 업데이트
            self._update_battery_status()
            
        except Exception as e:
            self.logger.error(f"Failed to read battery status: {e}")
    
    def _simulate_battery_status(self):
        """배터리 상태 시뮬레이션"""
        import random
        
        # 시뮬레이션된 배터리 데이터
        base_voltage = 3.7
        voltage_variation = random.uniform(-0.1, 0.1)
        self.battery_status['voltage'] = base_voltage + voltage_variation
        
        # 전류 시뮬레이션 (충전 중이면 양수, 방전 중이면 음수)
        if self.battery_status['is_charging']:
            self.battery_status['current'] = random.uniform(50, 200)  # mA
        else:
            self.battery_status['current'] = random.uniform(-500, -100)  # mA
        
        # 온도 시뮬레이션
        self.battery_status['temperature'] = random.uniform(20, 35)  # °C
        
        # 용량 시뮬레이션
        self.battery_status['remaining_capacity'] = random.uniform(2000, 3000)  # mAh
        self.battery_status['full_capacity'] = self.battery_config['capacity_mah']
        
        # 배터리 퍼센트 계산
        self.battery_status['percentage'] = (
            self.battery_status['remaining_capacity'] / 
            self.battery_status['full_capacity'] * 100
        )
    
    def _read_battery_hardware(self):
        """실제 하드웨어에서 배터리 상태 읽기"""
        try:
            # MAX17048 레지스터 읽기 (실제 구현)
            # VCELL 레지스터 (0x02, 0x03)
            vcell_msb = self.i2c_bus.read_byte_data(
                self.battery_config['battery_address'], 0x02
            )
            vcell_lsb = self.i2c_bus.read_byte_data(
                self.battery_config['battery_address'], 0x03
            )
            
            # 전압 계산 (12비트 ADC)
            vcell_raw = (vcell_msb << 4) | (vcell_lsb >> 4)
            voltage = vcell_raw * 0.000078125  # 78.125μV/LSB
            
            self.battery_status['voltage'] = voltage
            
            # SOC 레지스터 (0x04, 0x05)
            soc_msb = self.i2c_bus.read_byte_data(
                self.battery_config['battery_address'], 0x04
            )
            soc_lsb = self.i2c_bus.read_byte_data(
                self.battery_config['battery_address'], 0x05
            )
            
            # SOC 계산
            soc_raw = (soc_msb << 8) | soc_lsb
            percentage = soc_raw / 256.0  # 1/256% 단위
            
            self.battery_status['percentage'] = percentage
            
        except Exception as e:
            self.logger.error(f"Hardware read failed: {e}")
            # 시뮬레이션으로 폴백
            self._simulate_battery_status()
    
    def _update_battery_status(self):
        """배터리 상태 업데이트"""
        with self.lock:
            # 충전 상태 확인
            self.battery_status['is_charging'] = (
                self.battery_status['current'] > self.battery_config['charging_current_threshold']
            )
            
            # 배터리 풀 상태 확인
            self.battery_status['is_full'] = (
                self.battery_status['percentage'] >= 95.0 and 
                self.battery_status['is_charging']
            )
            
            # 배터리 부족 상태 확인
            self.battery_status['is_low'] = (
                self.battery_status['percentage'] <= self.battery_config['low_battery_threshold']
            )
            
            # 배터리 위험 상태 확인
            self.battery_status['is_critical'] = (
                self.battery_status['percentage'] <= self.battery_config['critical_battery_threshold']
            )
            
            # 남은 시간 계산
            if self.battery_status['current'] != 0:
                if self.battery_status['is_charging']:
                    # 충전 완료까지 시간
                    remaining_capacity = (
                        self.battery_status['full_capacity'] - 
                        self.battery_status['remaining_capacity']
                    )
                    self.battery_status['time_to_full'] = (
                        remaining_capacity / abs(self.battery_status['current']) * 60  # 분
                    )
                    self.battery_status['time_to_empty'] = 0
                else:
                    # 방전 완료까지 시간
                    self.battery_status['time_to_empty'] = (
                        self.battery_status['remaining_capacity'] / 
                        abs(self.battery_status['current']) * 60  # 분
                    )
                    self.battery_status['time_to_full'] = 0
            
            # 배터리 건강도 계산
            self.battery_status['health'] = min(100.0, (
                self.battery_status['full_capacity'] / 
                self.battery_config['capacity_mah'] * 100
            ))
    
    def _calculate_power_consumption(self):
        """전력 소비 계산"""
        with self.lock:
            # 현재 전력 계산 (W)
            current_power = (
                self.battery_status['voltage'] * 
                abs(self.battery_status['current']) / 1000.0
            )
            
            self.power_consumption['current_power'] = current_power
            
            # 전력 히스토리 업데이트
            timestamp = time.time()
            power_data = {
                'timestamp': timestamp,
                'power': current_power,
                'voltage': self.battery_status['voltage'],
                'current': self.battery_status['current']
            }
            
            self.power_consumption['power_history'].append(power_data)
            
            # 히스토리 크기 제한 (최근 1000개 데이터)
            if len(self.power_consumption['power_history']) > 1000:
                self.power_consumption['power_history'] = (
                    self.power_consumption['power_history'][-1000:]
                )
            
            # 평균 전력 계산
            if self.power_consumption['power_history']:
                recent_powers = [
                    data['power'] for data in 
                    self.power_consumption['power_history'][-100:]  # 최근 100개
                ]
                self.power_consumption['average_power'] = sum(recent_powers) / len(recent_powers)
                
                # 피크 전력 업데이트
                self.power_consumption['peak_power'] = max(
                    self.power_consumption['peak_power'],
                    current_power
                )
            
            # 총 에너지 소비 계산 (Wh)
            if len(self.power_consumption['power_history']) >= 2:
                last_data = self.power_consumption['power_history'][-2]
                time_diff = timestamp - last_data['timestamp']
                energy_increment = (current_power + last_data['power']) / 2 * time_diff / 3600  # Wh
                self.power_consumption['total_energy'] += energy_increment
    
    def _check_alerts(self):
        """알림 확인"""
        with self.lock:
            # 배터리 부족 알림
            if (self.battery_status['is_low'] and 
                not self.alerts['low_battery']):
                self.alerts['low_battery'] = True
                self._raise_alert('low_battery', 
                    f"Low battery: {self.battery_status['percentage']:.1f}%")
            
            elif not self.battery_status['is_low']:
                self.alerts['low_battery'] = False
            
            # 배터리 위험 알림
            if (self.battery_status['is_critical'] and 
                not self.alerts['critical_battery']):
                self.alerts['critical_battery'] = True
                self._raise_alert('critical_battery', 
                    f"Critical battery: {self.battery_status['percentage']:.1f}%")
            
            elif not self.battery_status['is_critical']:
                self.alerts['critical_battery'] = False
            
            # 과열 알림
            if (self.battery_status['temperature'] > self.battery_config['temperature_threshold'] and 
                not self.alerts['overheating']):
                self.alerts['overheating'] = True
                self._raise_alert('overheating', 
                    f"Battery overheating: {self.battery_status['temperature']:.1f}°C")
            
            elif self.battery_status['temperature'] <= self.battery_config['temperature_threshold']:
                self.alerts['overheating'] = False
            
            # 과전류 알림
            if (abs(self.battery_status['current']) > 2000 and  # 2A 이상
                not self.alerts['overcurrent']):
                self.alerts['overcurrent'] = True
                self._raise_alert('overcurrent', 
                    f"Overcurrent detected: {self.battery_status['current']:.1f}mA")
            
            elif abs(self.battery_status['current']) <= 2000:
                self.alerts['overcurrent'] = False
    
    def _raise_alert(self, alert_type: str, message: str):
        """알림 발생"""
        alert = {
            'type': alert_type,
            'message': message,
            'timestamp': time.time(),
            'battery_status': self.battery_status.copy()
        }
        
        self.logger.warning(f"Battery alert: {alert_type} - {message}")
        
        # 알림 콜백 함수 호출
        for callback in self.alert_callbacks:
            try:
                callback(alert)
            except Exception as e:
                self.logger.error(f"Alert callback error: {e}")
    
    def _notify_status_callbacks(self):
        """상태 콜백 함수 호출"""
        status_data = {
            'battery_status': self.battery_status.copy(),
            'power_consumption': self.power_consumption.copy(),
            'alerts': self.alerts.copy(),
            'timestamp': time.time()
        }
        
        for callback in self.status_callbacks:
            try:
                callback(status_data)
            except Exception as e:
                self.logger.error(f"Status callback error: {e}")
    
    def _log_battery_data(self):
        """배터리 데이터 로깅"""
        log_data = {
            'timestamp': datetime.now().isoformat(),
            'battery_status': self.battery_status.copy(),
            'power_consumption': {
                'current_power': self.power_consumption['current_power'],
                'average_power': self.power_consumption['average_power'],
                'total_energy': self.power_consumption['total_energy']
            },
            'alerts': self.alerts.copy()
        }
        
        # 로그 파일에 저장
        log_file = Path('logs/battery_data.json')
        log_file.parent.mkdir(exist_ok=True)
        
        try:
            if log_file.exists():
                with open(log_file, 'r') as f:
                    logs = json.load(f)
            else:
                logs = []
            
            logs.append(log_data)
            
            # 로그 크기 제한 (최근 1000개)
            if len(logs) > 1000:
                logs = logs[-1000:]
            
            with open(log_file, 'w') as f:
                json.dump(logs, f, indent=2)
                
        except Exception as e:
            self.logger.error(f"Failed to log battery data: {e}")
    
    def get_battery_status(self) -> Dict[str, Any]:
        """배터리 상태 반환"""
        with self.lock:
            return self.battery_status.copy()
    
    def get_power_consumption(self) -> Dict[str, Any]:
        """전력 소비 정보 반환"""
        with self.lock:
            return self.power_consumption.copy()
    
    def get_alerts(self) -> Dict[str, bool]:
        """알림 상태 반환"""
        with self.lock:
            return self.alerts.copy()
    
    def get_battery_health(self) -> Dict[str, Any]:
        """배터리 건강도 정보 반환"""
        with self.lock:
            return {
                'health_percentage': self.battery_status['health'],
                'cycle_count': self.battery_status['cycle_count'],
                'full_capacity': self.battery_status['full_capacity'],
                'design_capacity': self.battery_config['capacity_mah'],
                'capacity_ratio': (
                    self.battery_status['full_capacity'] / 
                    self.battery_config['capacity_mah']
                )
            }
    
    def get_power_history(self, hours: int = 24) -> List[Dict]:
        """전력 소비 히스토리 반환"""
        with self.lock:
            cutoff_time = time.time() - (hours * 3600)
            return [
                data for data in self.power_consumption['power_history']
                if data['timestamp'] >= cutoff_time
            ]
    
    def estimate_remaining_time(self) -> Dict[str, float]:
        """남은 시간 추정"""
        with self.lock:
            return {
                'time_to_empty': self.battery_status['time_to_empty'],
                'time_to_full': self.battery_status['time_to_full'],
                'is_charging': self.battery_status['is_charging']
            }
    
    def add_status_callback(self, callback: Callable):
        """상태 콜백 함수 추가"""
        self.status_callbacks.append(callback)
    
    def add_alert_callback(self, callback: Callable):
        """알림 콜백 함수 추가"""
        self.alert_callbacks.append(callback)
    
    def add_data_callback(self, callback: Callable):
        """데이터 콜백 함수 추가"""
        self.data_callbacks.append(callback)
    
    def stop_monitoring(self):
        """모니터링 중지"""
        self.is_monitoring = False
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=5)
        self.logger.info("Battery monitoring stopped")
    
    def update_config(self, new_config: Dict[str, Any]):
        """배터리 설정 업데이트"""
        with self.lock:
            self.battery_config.update(new_config)
            self.logger.info("Battery configuration updated")

# 전역 배터리 모니터 인스턴스
battery_monitor = BatteryMonitor()

def get_battery_monitor() -> BatteryMonitor:
    """배터리 모니터 인스턴스 반환"""
    return battery_monitor

if __name__ == "__main__":
    # 배터리 모니터 테스트
    monitor = get_battery_monitor()
    
    print("=== Battery Monitor Test ===")
    
    # 상태 콜백 함수
    def status_callback(status_data):
        print(f"Battery Status: {status_data['battery_status']['percentage']:.1f}%")
        print(f"Voltage: {status_data['battery_status']['voltage']:.2f}V")
        print(f"Current: {status_data['battery_status']['current']:.1f}mA")
        print(f"Temperature: {status_data['battery_status']['temperature']:.1f}°C")
        print(f"Power: {status_data['power_consumption']['current_power']:.2f}W")
        print()
    
    # 알림 콜백 함수
    def alert_callback(alert):
        print(f"ALERT: {alert['type']} - {alert['message']}")
    
    # 콜백 함수 등록
    monitor.add_status_callback(status_callback)
    monitor.add_alert_callback(alert_callback)
    
    # 10초간 모니터링
    print("Monitoring battery for 10 seconds...")
    time.sleep(10)
    
    # 최종 상태 출력
    print("Final Battery Status:")
    print(f"Status: {monitor.get_battery_status()}")
    print(f"Power Consumption: {monitor.get_power_consumption()}")
    print(f"Alerts: {monitor.get_alerts()}")
    print(f"Health: {monitor.get_battery_health()}")
    
    # 모니터링 중지
    monitor.stop_monitoring()
    
    print("\nBattery monitor test completed!")
