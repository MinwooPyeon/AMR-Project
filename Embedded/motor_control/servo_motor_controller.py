#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
서보모터 컨트롤러
"""

import time
import threading
from typing import Dict, Optional
from utils.logger import motor_logger

class ServoMotorController:
    def __init__(self, i2c_address: int = 0x40, frequency: int = 50):
        self.i2c_address = i2c_address
        self.frequency = frequency
        
        self.servo_channels = {
            "servo1": 0,
            "servo2": 1,  
            "servo3": 2,  
            "servo4": 3   
        }
        
        self.angle_min = 0
        self.angle_max = 180
        self.default_angle = 90
        
        self.pulse_min = 500   
        self.pulse_max = 2500  
        
        self.current_angles = {
            "servo1": self.default_angle,
            "servo2": self.default_angle,
            "servo3": self.default_angle,
            "servo4": self.default_angle
        }
        
        self.angle_lock = threading.Lock()
        
        self.pca9685 = None
        self._initialize_pca9685()
        
        motor_logger.success("서보모터 컨트롤러 초기화 완료")
    
    def _initialize_pca9685(self):
        try:
            import smbus2 as smbus
            
            self.bus = smbus.SMBus(1)
            self._pca9685_init()
            
            motor_logger.success(f"PCA9685 드라이버 초기화 성공 - 주소: 0x{self.i2c_address:02X}")
            
        except ImportError:
            motor_logger.warn("smbus2 모듈이 없어 시뮬레이션 모드로 동작")
            self.bus = None
        except Exception as e:
            motor_logger.error(f"PCA9685 초기화 실패: {e}")
            self.bus = None
    
    def _pca9685_init(self):
        if not self.bus:
            return
        
        try:
            self.bus.write_byte_data(self.i2c_address, 0x00, 0x20)
            time.sleep(0.01)
            
            self.bus.write_byte_data(self.i2c_address, 0x01, 0x04)
            time.sleep(0.01)
            
            prescale = int(25000000 / (4096 * self.frequency) - 1)
            self.bus.write_byte_data(self.i2c_address, 0xFE, prescale)
            time.sleep(0.01)
            
            self.bus.write_byte_data(self.i2c_address, 0x00, 0xA0)
            time.sleep(0.01)
            
            motor_logger.info(f"PCA9685 초기화 완료 - 주파수: {self.frequency}Hz")
            
        except Exception as e:
            motor_logger.error(f"PCA9685 초기화 오류: {e}")
    
    def _set_pwm(self, channel: int, on: int, off: int):
        if not self.bus:
            motor_logger.debug(f"시뮬레이션 모드 - 채널 {channel}: ON={on}, OFF={off}")
            return
        
        try:
            self.bus.write_byte_data(self.i2c_address, 0x06 + 4 * channel, on & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x07 + 4 * channel, (on >> 8) & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x08 + 4 * channel, off & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x09 + 4 * channel, (off >> 8) & 0xFF)
            
        except Exception as e:
            motor_logger.error(f"PWM 설정 오류 - 채널 {channel}: {e}")
    
    def _angle_to_pulse(self, angle: float) -> int:
        pulse_width = int(self.pulse_min + (angle / 180.0) * (self.pulse_max - self.pulse_min))
        counter_value = int(pulse_width * 4096 / 20000)
        return counter_value
    
    def set_servo_angle(self, servo_name: str, angle: float) -> bool:
        if servo_name not in self.servo_channels:
            motor_logger.error(f"알 수 없는 서보모터: {servo_name}")
            return False
        
        if angle < self.angle_min or angle > self.angle_max:
            motor_logger.error(f"각도 범위 오류: {angle}도 (범위: {self.angle_min}-{self.angle_max}도)")
            return False
        
        try:
            channel = self.servo_channels[servo_name]
            pulse_counter = self._angle_to_pulse(angle)
            
            self._set_pwm(channel, 0, pulse_counter)
            
            with self.angle_lock:
                self.current_angles[servo_name] = angle
            
            motor_logger.info(f"{servo_name} 각도 설정: {angle}도 (채널: {channel})")
            return True
            
        except Exception as e:
            motor_logger.error(f"서보모터 각도 설정 오류: {e}")
            return False
    
    def set_all_servos(self, angle: float) -> bool:
        success = True
        for servo_name in self.servo_channels.keys():
            if not self.set_servo_angle(servo_name, angle):
                success = False
        
        if success:
            motor_logger.info(f"모든 서보모터 각도 설정: {angle}도")
        
        return success
    
    def set_servo_angles(self, angles: Dict[str, float]) -> bool:
        success = True
        for servo_name, angle in angles.items():
            if not self.set_servo_angle(servo_name, angle):
                success = False
        
        if success:
            motor_logger.info(f"서보모터 각도 설정: {angles}")
        
        return success
    
    def get_servo_angle(self, servo_name: str) -> Optional[float]:
        if servo_name not in self.current_angles:
            motor_logger.error(f"알 수 없는 서보모터: {servo_name}")
            return None
        
        with self.angle_lock:
            return self.current_angles[servo_name]
    
    def get_all_angles(self) -> Dict[str, float]:
        with self.angle_lock:
            return self.current_angles.copy()
    
    def reset_all_servos(self) -> bool:
        return self.set_all_servos(self.default_angle)
    
    def sweep_servo(self, servo_name: str, start_angle: float = 0, end_angle: float = 180, 
                   step: float = 5, delay: float = 0.1) -> bool:
        if servo_name not in self.servo_channels:
            motor_logger.error(f"알 수 없는 서보모터: {servo_name}")
            return False
        
        try:
            motor_logger.info(f"{servo_name} 스윕 시작: {start_angle}도 → {end_angle}도")
            
            for angle in range(int(start_angle), int(end_angle) + 1, int(step)):
                self.set_servo_angle(servo_name, angle)
                time.sleep(delay)
            
            for angle in range(int(end_angle), int(start_angle) - 1, -int(step)):
                self.set_servo_angle(servo_name, angle)
                time.sleep(delay)
            
            motor_logger.info(f"{servo_name} 스윕 완료")
            return True
            
        except Exception as e:
            motor_logger.error(f"서보모터 스윕 오류: {e}")
            return False
    
    def get_status(self) -> Dict:
        return {
            "initialized": self.bus is not None,
            "i2c_address": f"0x{self.i2c_address:02X}",
            "frequency": self.frequency,
            "current_angles": self.get_all_angles(),
            "servo_channels": self.servo_channels,
            "angle_range": {"min": self.angle_min, "max": self.angle_max},
            "pulse_range": {"min": self.pulse_min, "max": self.pulse_max}
        }
    
    def cleanup(self):
        if self.bus:
            try:
                self.bus.close()
                motor_logger.info("I2C 버스 연결 해제")
            except Exception as e:
                motor_logger.error(f"I2C 버스 해제 오류: {e}")

def test_servo_motor_controller():
    print("=== 서보모터 컨트롤러 테스트 ===")
    print("PCA9685 드라이버를 사용하여 서보모터 4개 제어")
    print("=" * 50)
    
    controller = ServoMotorController()
    
    status = controller.get_status()
    print(f"\n📊 서보모터 컨트롤러 상태:")
    print(f"  - 초기화됨: {'예' if status['initialized'] else '아니오'}")
    print(f"  - I2C 주소: {status['i2c_address']}")
    print(f"  - PWM 주파수: {status['frequency']}Hz")
    print(f"  - 각도 범위: {status['angle_range']['min']}° - {status['angle_range']['max']}°")
    
    print(f"\n🔧 서보모터 채널:")
    for servo_name, channel in status['servo_channels'].items():
        print(f"  - {servo_name}: 채널 {channel}")
    
    try:
        print(f"\n🧪 테스트 시작...")
        
        print("\n1. 모든 서보모터 90도 설정")
        controller.set_all_servos(90)
        time.sleep(2)
        
        print("\n2. 개별 서보모터 테스트")
        test_angles = [0, 45, 90, 135, 180]
        
        for servo_name in controller.servo_channels.keys():
            print(f"\n   {servo_name} 테스트:")
            for angle in test_angles:
                controller.set_servo_angle(servo_name, angle)
                print(f"     → {angle}도")
                time.sleep(0.5)
        
        print("\n3. 모든 서보모터 90도로 리셋")
        controller.reset_all_servos()
        time.sleep(2)
        
        print("\n4. servo1 스윕 테스트")
        controller.sweep_servo("servo1", 0, 180, 10, 0.1)
        
        print("\n5. 최종 상태 확인")
        final_angles = controller.get_all_angles()
        for servo_name, angle in final_angles.items():
            print(f"   {servo_name}: {angle}도")
        
        print("\n✅ 서보모터 컨트롤러 테스트 완료")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    test_servo_motor_controller() 