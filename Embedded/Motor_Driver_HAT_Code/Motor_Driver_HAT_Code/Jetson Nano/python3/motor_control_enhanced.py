#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Jetson Nano용 PCA9685 모터 제어 코드
Motor Driver HAT을 사용한 DC 모터 제어
"""

import time
import math
import smbus
from PCA9685 import PCA9685

class MotorDriverHAT:
    """Motor Driver HAT 클래스"""
    
    # 모터 방향 정의
    FORWARD = 'forward'
    BACKWARD = 'backward'
    
    def __init__(self, i2c_address=0x40, i2c_bus=7, debug=True):
        """
        모터 드라이버 초기화
        
        Args:
            i2c_address (int): I2C 주소 (기본값: 0x40)
            i2c_bus (int): I2C 버스 번호 (기본값: 7)
            debug (bool): 디버그 모드 (기본값: True)
        """
        self.debug = debug
        self.i2c_address = i2c_address
        self.i2c_bus = i2c_bus
        
        # 모터 핀 정의
        self.PWMA = 0  # 모터 A PWM
        self.AIN1 = 1  # 모터 A 방향 1
        self.AIN2 = 2  # 모터 A 방향 2
        self.PWMB = 5  # 모터 B PWM
        self.BIN1 = 3  # 모터 B 방향 1
        self.BIN2 = 4  # 모터 B 방향 2
        
        # 모터 상태
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        self.motor_a_direction = self.FORWARD
        self.motor_b_direction = self.FORWARD
        
        try:
            # PCA9685 초기화
            self.pwm = PCA9685(i2c_address, debug=debug)
            self.pwm.setPWMFreq(50)  # 50Hz PWM 주파수
            
            # 모든 모터 정지
            self.stop_all()
            
            if self.debug:
                print(f"Motor Driver HAT 초기화 완료")
                print(f"   I2C 주소: 0x{i2c_address:02X}")
                print(f"   I2C 버스: {i2c_bus}")
                print(f"   PWM 주파수: 50Hz")
                
        except Exception as e:
            print(f"Motor Driver HAT 초기화 실패: {e}")
            raise
    
    def set_motor_speed(self, motor, direction, speed):
        """
        모터 속도 설정
        
        Args:
            motor (int): 모터 번호 (0: 모터A, 1: 모터B)
            direction (str): 방향 ('forward' 또는 'backward')
            speed (int): 속도 (0-100)
        """
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
            
        if motor == 0:  # 모터 A
            self.pwm.setDutycycle(self.PWMA, speed)
            if direction == self.FORWARD:
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:  # backward
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
            
            self.motor_a_speed = speed
            self.motor_a_direction = direction
            
            if self.debug:
                print(f"모터 A: {direction}, 속도: {speed}%")
                
        elif motor == 1:  # 모터 B
            self.pwm.setDutycycle(self.PWMB, speed)
            if direction == self.FORWARD:
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
            else:  # backward
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)
            
            self.motor_b_speed = speed
            self.motor_b_direction = direction
            
            if self.debug:
                print(f"모터 B: {direction}, 속도: {speed}%")
    
    def stop_motor(self, motor):
        """
        개별 모터 정지
        
        Args:
            motor (int): 모터 번호 (0: 모터A, 1: 모터B)
        """
        if motor == 0:
            self.pwm.setDutycycle(self.PWMA, 0)
            self.motor_a_speed = 0
            if self.debug:
                print("모터 A 정지")
        elif motor == 1:
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
            if self.debug:
                print("모터 B 정지")
    
    def stop_all(self):
        """모든 모터 정지"""
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        if self.debug:
            print("모든 모터 정지")
    
    def get_motor_status(self):
        """모터 상태 조회"""
        return {
            'motor_a': {
                'speed': self.motor_a_speed,
                'direction': self.motor_a_direction
            },
            'motor_b': {
                'speed': self.motor_b_speed,
                'direction': self.motor_b_direction
            }
        }
    
    def test_motors(self):
        """모터 테스트"""
        print("\n모터 테스트 시작")
        
        # 모터 A 테스트
        print("\n모터 A 테스트")
        self.set_motor_speed(0, self.FORWARD, 40)
        time.sleep(2)
        self.set_motor_speed(0, self.BACKWARD, 40)
        time.sleep(2)
        self.stop_motor(0)
        
        # 모터 B 테스트
        print("\n모터 B 테스트")
        self.set_motor_speed(1, self.FORWARD, 40)
        time.sleep(2)
        self.set_motor_speed(1, self.BACKWARD, 40)
        time.sleep(2)
        self.stop_motor(1)
        
        print("모터 테스트 완료")
    
    def differential_drive(self, left_speed, right_speed):
        """
        차동 구동 (로봇 이동)
        
        Args:
            left_speed (int): 왼쪽 모터 속도 (-100 ~ 100)
            right_speed (int): 오른쪽 모터 속도 (-100 ~ 100)
        """
        # 왼쪽 모터 (모터 A)
        if left_speed > 0:
            self.set_motor_speed(0, self.FORWARD, abs(left_speed))
        elif left_speed < 0:
            self.set_motor_speed(0, self.BACKWARD, abs(left_speed))
        else:
            self.stop_motor(0)
        
        # 오른쪽 모터 (모터 B)
        if right_speed > 0:
            self.set_motor_speed(1, self.FORWARD, abs(right_speed))
        elif right_speed < 0:
            self.set_motor_speed(1, self.BACKWARD, abs(right_speed))
        else:
            self.stop_motor(1)
        
        if self.debug:
            print(f"차동 구동: L={left_speed}, R={right_speed}")

def main():
    """메인 함수"""
    print("=" * 60)
    print("Jetson Nano Motor Driver HAT 테스트")
    print("=" * 60)
    
    try:
        # 모터 드라이버 초기화
        motor = MotorDriverHAT(debug=True)
        
        # 모터 테스트
        motor.test_motors()
        
        # 차동 구동 테스트
        print("\n차동 구동 테스트")
        print("전진")
        motor.differential_drive(50, 50)
        time.sleep(3)
        
        print("후진")
        motor.differential_drive(-50, -50)
        time.sleep(3)
        
        print("좌회전")
        motor.differential_drive(-30, 30)
        time.sleep(3)
        
        print("우회전")
        motor.differential_drive(30, -30)
        time.sleep(3)
        
        # 정지
        motor.stop_all()
        print("\n테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단됨")
        motor.stop_all()
    except Exception as e:
        print(f"\n오류 발생: {e}")
        motor.stop_all()
    finally:
        print("정리 완료")

if __name__ == "__main__":
    main() 