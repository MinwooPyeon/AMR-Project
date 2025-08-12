#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
실제 모터 컨트롤러
Motor_Driver_HAT_Code의 실제 모터 제어 코드를 기반으로 함
"""

import time
import logging
import sys
import os

# PCA9685 모듈 경로 추가
sys.path.append(os.path.join(os.path.dirname(__file__), 'Motor_Driver_HAT_Code/Motor_Driver_HAT_Code/Jetson Nano/python3'))

try:
    from PCA9685 import PCA9685
    PCA9685_AVAILABLE = True
except ImportError:
    PCA9685_AVAILABLE = False
    logging.warning("PCA9685 모듈을 찾을 수 없습니다. 시뮬레이션 모드로 실행됩니다.")

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RealMotorController:
    """실제 모터 컨트롤러 (Motor_Driver_HAT_Code 기반)"""
    
    def __init__(self):
        self.Dir = ['forward', 'backward']
        self.pwm = None
        self.motor_driver = None
        self.is_initialized = False
        
        # 모터 핀 설정
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4
        
        # 현재 모터 상태
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.left_direction = 'forward'
        self.right_direction = 'forward'
        
        self._initialize_motor()
    
    def _initialize_motor(self):
        """모터 초기화"""
        if not PCA9685_AVAILABLE:
            logger.warning("PCA9685 모듈이 없어 시뮬레이션 모드로 실행됩니다.")
            return
        
        try:
            # PCA9685 초기화
            self.pwm = PCA9685(0x40, debug=False)
            self.pwm.setPWMFreq(50)
            
            # 모터 드라이버 초기화
            self.motor_driver = MotorDriver(self.pwm)
            self.is_initialized = True
            
            logger.info("실제 모터 컨트롤러 초기화 완료")
            
        except Exception as e:
            logger.error(f"모터 초기화 실패: {e}")
            self.is_initialized = False
    
    def set_speed(self, left_speed: float, right_speed: float) -> bool:
        """모터 속도 설정"""
        if not self.is_initialized:
            logger.warning("모터가 초기화되지 않았습니다. 시뮬레이션 모드로 실행됩니다.")
            self._simulate_set_speed(left_speed, right_speed)
            return True
        
        try:
            # 속도 제한 (0-100)
            left_speed = max(0, min(100, abs(left_speed)))
            right_speed = max(0, min(100, abs(right_speed)))
            
            # 방향 결정
            left_direction = 'forward' if left_speed >= 0 else 'backward'
            right_direction = 'forward' if right_speed >= 0 else 'backward'
            
            # 실제 모터 제어
            if left_speed > 0:
                self.motor_driver.MotorRun(0, left_direction, int(left_speed))
            else:
                self.motor_driver.MotorStop(0)
            
            if right_speed > 0:
                self.motor_driver.MotorRun(1, right_direction, int(right_speed))
            else:
                self.motor_driver.MotorStop(1)
            
            # 상태 업데이트
            self.left_speed = left_speed
            self.right_speed = right_speed
            self.left_direction = left_direction
            self.right_direction = right_direction
            
            logger.info(f"실제 모터 속도 설정: L={left_speed:.1f}({left_direction}), R={right_speed:.1f}({right_direction})")
            return True
            
        except Exception as e:
            logger.error(f"모터 속도 설정 실패: {e}")
            return False
    
    def _simulate_set_speed(self, left_speed: float, right_speed: float):
        """시뮬레이션 모드에서 속도 설정"""
        self.left_speed = left_speed
        self.right_speed = right_speed
        logger.info(f"시뮬레이션 모터 속도 설정: L={left_speed:.1f}, R={right_speed:.1f}")
    
    def stop(self) -> bool:
        """모터 정지"""
        if not self.is_initialized:
            logger.warning("모터가 초기화되지 않았습니다.")
            self._simulate_set_speed(0, 0)
            return True
        
        try:
            self.motor_driver.MotorStop(0)
            self.motor_driver.MotorStop(1)
            
            self.left_speed = 0.0
            self.right_speed = 0.0
            
            logger.info("실제 모터 정지")
            return True
            
        except Exception as e:
            logger.error(f"모터 정지 실패: {e}")
            return False
    
    def get_speeds(self) -> dict:
        """현재 속도 조회"""
        return {
            "left_speed": self.left_speed,
            "right_speed": self.right_speed,
            "left_direction": self.left_direction,
            "right_direction": self.right_direction,
            "is_running": (self.left_speed > 0 or self.right_speed > 0),
            "is_initialized": self.is_initialized
        }
    
    def cleanup(self):
        """모터 정리"""
        if self.is_initialized:
            self.stop()

class MotorDriver:
    """모터 드라이버 클래스 (원본 코드 기반)"""
    
    def __init__(self, pwm):
        self.pwm = pwm
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4
    
    def MotorRun(self, motor, index, speed):
        """모터 실행"""
        if speed > 100:
            return
        
        if motor == 0:  # 좌측 모터
            self.pwm.setDutycycle(self.PWMA, speed)
            if index == 'forward':
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:  # backward
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
        else:  # 우측 모터
            self.pwm.setDutycycle(self.PWMB, speed)
            if index == 'forward':
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
            else:  # backward
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)
    
    def MotorStop(self, motor):
        """모터 정지"""
        if motor == 0:
            self.pwm.setDutycycle(self.PWMA, 0)
        else:
            self.pwm.setDutycycle(self.PWMB, 0)

def test_real_motor():
    """실제 모터 테스트"""
    print("=== 실제 모터 컨트롤러 테스트 ===")
    
    motor = RealMotorController()
    
    try:
        print("모터 테스트 시작...")
        
        # 전진 테스트
        print("\n1. 전진 테스트 (3초)")
        motor.set_speed(50, 50)
        time.sleep(10)
        
        # 정지
        print("\n2. 정지 테스트 (2초)")
        motor.stop()
        time.sleep(2)
        
        # 좌회전 테스트
        print("\n3. 좌회전 테스트 (3초)")
        motor.set_speed(30, 50)
        time.sleep(10)
        
        # 우회전 테스트
        print("\n4. 우회전 테스트 (3초)")
        motor.set_speed(50, 30)
        time.sleep(10)
        
        # 후진 테스트
        print("\n5. 후진 테스트 (3초)")
        motor.set_speed(-50, -50)
        time.sleep(10)
        
        # 최종 정지
        print("\n6. 최종 정지")
        motor.stop()
        
        print("\n=== 테스트 완료 ===")
        
        # 최종 상태 출력
        speeds = motor.get_speeds()
        print(f"최종 상태:")
        print(f"  - 좌측 모터: {speeds['left_speed']:.1f} ({speeds['left_direction']})")
        print(f"  - 우측 모터: {speeds['right_speed']:.1f} ({speeds['right_direction']})")
        print(f"  - 동작 중: {speeds['is_running']}")
        print(f"  - 초기화됨: {speeds['is_initialized']}")
        
    except KeyboardInterrupt:
        print("\n테스트 중단")
        motor.stop()
    finally:
        motor.cleanup()

if __name__ == "__main__":
    test_real_motor() 