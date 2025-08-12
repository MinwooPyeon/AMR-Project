#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 드라이버 테스트 (PCA9685 기반)
"""

import time
from PCA9685 import PCA9685

class MotorDriver:
    """모터 드라이버 클래스"""
    
    def __init__(self, pwm):
        self.pwm = pwm
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4
        
        # 모터 초기화
        self.pwm.setPWMFreq(50)
        self.pwm.setPWM(self.PWMA, 0, 0)
        self.pwm.setPWM(self.AIN1, 0, 0)
        self.pwm.setPWM(self.AIN2, 0, 0)
        self.pwm.setPWM(self.PWMB, 0, 0)
        self.pwm.setPWM(self.BIN1, 0, 0)
        self.pwm.setPWM(self.BIN2, 0, 0)
        
        print("모터 드라이버 초기화 완료")
    
    def MotorRun(self, motor, index, speed):
        """모터 구동"""
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
            
        if motor == 0:
            if index == 1:
                self.pwm.setPWM(self.AIN1, 0, 4095)
                self.pwm.setPWM(self.AIN2, 0, 0)
                self.pwm.setPWM(self.PWMA, 0, int(speed * 40.95))
            else:
                self.pwm.setPWM(self.AIN1, 0, 0)
                self.pwm.setPWM(self.AIN2, 0, 4095)
                self.pwm.setPWM(self.PWMA, 0, int(speed * 40.95))
        else:
            if index == 1:
                self.pwm.setPWM(self.BIN1, 0, 4095)
                self.pwm.setPWM(self.BIN2, 0, 0)
                self.pwm.setPWM(self.PWMB, 0, int(speed * 40.95))
            else:
                self.pwm.setPWM(self.BIN1, 0, 0)
                self.pwm.setPWM(self.BIN2, 0, 4095)
                self.pwm.setPWM(self.PWMB, 0, int(speed * 40.95))
    
    def MotorStop(self, motor):
        """모터 정지"""
        if motor == 0:
            self.pwm.setPWM(self.PWMA, 0, 0)
        else:
            self.pwm.setPWM(self.PWMB, 0, 0)

def main():
    """메인 함수"""
    print("=== 모터 드라이버 테스트 ===")
    
    # PCA9685 초기화
    pwm = PCA9685(0x40, debug=False)
    
    # 모터 드라이버 초기화
    motor_driver = MotorDriver(pwm)
    
    try:
        print("모터 테스트 시작...")
        
        # 전진 테스트
        print("전진 테스트 (3초)")
        motor_driver.MotorRun(0, 1, 50)  # 왼쪽 모터 전진
        motor_driver.MotorRun(1, 1, 50)  # 오른쪽 모터 전진
        time.sleep(3)
        
        # 정지
        print("정지")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        # 후진 테스트
        print("후진 테스트 (3초)")
        motor_driver.MotorRun(0, 0, 50)  # 왼쪽 모터 후진
        motor_driver.MotorRun(1, 0, 50)  # 오른쪽 모터 후진
        time.sleep(3)
        
        # 정지
        print("정지")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        # 좌회전 테스트
        print("좌회전 테스트 (3초)")
        motor_driver.MotorRun(0, 0, 50)  # 왼쪽 모터 후진
        motor_driver.MotorRun(1, 1, 50)  # 오른쪽 모터 전진
        time.sleep(3)
        
        # 정지
        print("정지")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        # 우회전 테스트
        print("우회전 테스트 (3초)")
        motor_driver.MotorRun(0, 1, 50)  # 왼쪽 모터 전진
        motor_driver.MotorRun(1, 0, 50)  # 오른쪽 모터 후진
        time.sleep(3)
        
        # 정지
        print("정지")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        
        print("모터 테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n테스트 중단됨")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
    except Exception as e:
        print(f"오류 발생: {e}")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)

if __name__ == "__main__":
    main() 