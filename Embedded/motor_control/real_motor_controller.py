#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading

try:
    import smbus2 as smbus
    SMBUS2_AVAILABLE = True
except ImportError:
    SMBUS2_AVAILABLE = False

class RealMotorController:
    
    def __init__(self, i2c_address: int = 0x40, i2c_bus: int = 0):
        self.i2c_address = i2c_address
        self.i2c_bus = i2c_bus
        
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4
        
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.left_direction = 'forward'
        self.right_direction = 'forward'
        
        self.motor_lock = threading.Lock()
        
        self.bus = None
        self.is_initialized = False
        
        self._initialize_motor()
    
    def _initialize_motor(self):
        if not SMBUS2_AVAILABLE:
            print("smbus2 모듈이 설치되어 있지 않습니다")
            return
        
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
            
            self._pca9685_init()
            
            self._set_motor_pins(0, 0, 0, 0, 0, 0)
            
            self.is_initialized = True
            print(f"실제 모터 컨트롤러 초기화 완료 - 버스: {self.i2c_bus}, 주소: 0x{self.i2c_address:02X}")
            
        except Exception as e:
            print(f"모터 초기화 실패: {e}")
            self.is_initialized = False
    
    def _pca9685_init(self):
        if not self.bus:
            return
        
        try:
            self.bus.write_byte_data(self.i2c_address, 0x00, 0x20)
            time.sleep(0.01)
            
            self.bus.write_byte_data(self.i2c_address, 0x01, 0x04)
            time.sleep(0.01)
            
            prescale = int(25000000 / (4096 * 50) - 1)
            self.bus.write_byte_data(self.i2c_address, 0xFE, prescale)
            time.sleep(0.01)
            
            self.bus.write_byte_data(self.i2c_address, 0x00, 0xA0)
            time.sleep(0.01)
            
            print(f"PCA9685 초기화 완료 - 주파수: 50Hz")
            
        except Exception as e:
            print(f"PCA9685 초기화 오류: {e}")
    
    def _set_pwm(self, channel: int, on: int, off: int):
        """PWM 채널 설정"""
        if not self.bus:
            return
        
        try:
            self.bus.write_byte_data(self.i2c_address, 0x06 + 4 * channel, on & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x07 + 4 * channel, (on >> 8) & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x08 + 4 * channel, off & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x09 + 4 * channel, (off >> 8) & 0xFF)
            
        except Exception as e:
            print(f"PWM 설정 오류 - 채널 {channel}: {e}")
    
    def _set_motor_pins(self, pwma_on, pwma_off, ain1_on, ain1_off, ain2_on, ain2_off):
        """모터 핀 설정"""
        self._set_pwm(self.PWMA, pwma_on, pwma_off)
        self._set_pwm(self.AIN1, ain1_on, ain1_off)
        self._set_pwm(self.AIN2, ain2_on, ain2_off)
    
    def set_speed(self, left_speed: float, right_speed: float) -> bool:
        """모터 속도 설정"""
        with self.motor_lock:
            if not self.is_initialized:
                self._simulate_set_speed(left_speed, right_speed)
                return True
            
            try:
                # 속도 범위 제한 (-100 ~ 100)
                left_speed = max(-100, min(100, left_speed))
                right_speed = max(-100, min(100, right_speed))
                
                # 왼쪽 모터 제어
                if left_speed > 0:
                    # 전진
                    pwm_value = int((left_speed / 100.0) * 4095)
                    self._set_motor_pins(0, pwm_value, 4095, 0, 0, 0)
                elif left_speed < 0:
                    # 후진
                    pwm_value = int((abs(left_speed) / 100.0) * 4095)
                    self._set_motor_pins(0, pwm_value, 0, 0, 4095, 0)
                else:
                    # 정지
                    self._set_motor_pins(0, 0, 0, 0, 0, 0)
                
                # 오른쪽 모터 제어
                if right_speed > 0:
                    # 전진
                    pwm_value = int((right_speed / 100.0) * 4095)
                    self._set_pwm(self.PWMB, 0, pwm_value)
                    self._set_pwm(self.BIN1, 4095, 0)
                    self._set_pwm(self.BIN2, 0, 0)
                elif right_speed < 0:
                    # 후진
                    pwm_value = int((abs(right_speed) / 100.0) * 4095)
                    self._set_pwm(self.PWMB, 0, pwm_value)
                    self._set_pwm(self.BIN1, 0, 0)
                    self._set_pwm(self.BIN2, 4095, 0)
                else:
                    # 정지
                    self._set_pwm(self.PWMB, 0, 0)
                    self._set_pwm(self.BIN1, 0, 0)
                    self._set_pwm(self.BIN2, 0, 0)
                
                self.left_speed = left_speed
                self.right_speed = right_speed
                self.left_direction = 'forward' if left_speed >= 0 else 'backward'
                self.right_direction = 'forward' if right_speed >= 0 else 'backward'
                
                print(f"모터 속도 설정: L={left_speed:.1f}, R={right_speed:.1f}")
                return True
                
            except Exception as e:
                print(f"모터 속도 설정 오류: {e}")
                return False
    
    def _simulate_set_speed(self, left_speed: float, right_speed: float):
        """시뮬레이션 모드에서 속도 설정"""
        self.left_speed = left_speed
        self.right_speed = right_speed
        print(f"시뮬레이션 모터 속도 설정: L={left_speed:.1f}, R={right_speed:.1f}")
    
    def stop(self) -> bool:
        """모터 정지"""
        return self.set_speed(0, 0)
    
    def emergency_stop(self) -> bool:
        """비상 정지 - 모든 모터 핀을 0으로 설정"""
        try:
            if self.bus:
                # 모든 채널을 0으로 설정
                for channel in range(16):
                    self._set_pwm(channel, 0, 0)
                print("🚨 비상 정지 실행")
                return True
        except Exception as e:
            print(f"비상 정지 오류: {e}")
        return False
    
    def test_motor_connection(self) -> bool:
        """모터 연결 테스트"""
        try:
            if not self.bus:
                print("❌ I2C 버스 연결 없음")
                return False
            
            # PCA9685 통신 테스트
            mode1 = self.bus.read_byte_data(self.i2c_address, 0x00)
            print(f"✅ PCA9685 통신 성공 (MODE1: 0x{mode1:02X})")
            
            # 간단한 PWM 테스트
            self._set_pwm(0, 0, 1000)  # 채널 0에 PWM 신호
            time.sleep(0.1)
            self._set_pwm(0, 0, 0)     # 정지
            
            print("✅ 모터 연결 테스트 성공")
            return True
            
        except Exception as e:
            print(f"❌ 모터 연결 테스트 실패: {e}")
            return False
    
    def get_speeds(self) -> dict:
        """현재 속도 조회"""
        return {
            "left_speed": self.left_speed,
            "right_speed": self.right_speed,
            "left_direction": self.left_direction,
            "right_direction": self.right_direction,
            "is_running": (self.left_speed != 0 or self.right_speed != 0),
            "is_initialized": self.is_initialized
        }
    
    def cleanup(self):
        """정리"""
        if self.bus:
            try:
                self.stop()
                self.bus.close()
                print("I2C 버스 연결 해제")
            except Exception as e:
                print(f"I2C 버스 해제 오류: {e}") 