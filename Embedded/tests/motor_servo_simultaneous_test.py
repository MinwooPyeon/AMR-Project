#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 드라이버와 서보 모터 드라이버 동시 구동 테스트
사용자 제공 코드 기반으로 작성
"""

import time
import threading
import math
import smbus
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class MotorDriver:
    """모터 드라이버 클래스 (사용자 제공 코드 기반)"""
    
    # Registers/etc.
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address=0x40, debug=True, bus_number=0):
        self.bus = smbus.SMBus(bus_number)
        self.address = address
        self.debug = debug
        
        if self.debug:
            print(f"모터 드라이버 초기화 - I2C 버스: {bus_number}, 주소: 0x{address:02X}")
        
        try:
            self.write(self.__MODE1, 0x00)
            time.sleep(0.01)
            print("모터 드라이버 초기화 성공")
        except Exception as e:
            print(f"모터 드라이버 초기화 실패: {e}")
            raise

    def write(self, reg, value):
        """레지스터에 8비트 값 쓰기"""
        try:
            self.bus.write_byte_data(self.address, reg, value)
            if self.debug:
                print(f"모터 I2C: 레지스터 0x{reg:02X}에 0x{value:02X} 쓰기")
        except Exception as e:
            print(f"모터 I2C 쓰기 오류: {e}")
            raise

    def read(self, reg):
        """I2C 디바이스에서 바이트 읽기"""
        try:
            result = self.bus.read_byte_data(self.address, reg)
            if self.debug:
                print(f"모터 I2C: 디바이스 0x{self.address:X}에서 레지스터 0x{reg:X}로부터 0x{result:X} 읽기")
            return result
        except Exception as e:
            print(f"모터 I2C 읽기 오류: {e}")
            raise

    def setPWMFreq(self, freq):
        """PWM 주파수 설정"""
        prescaleval = 25000000.0  # 25MHz
        prescaleval //= 4096.0     # 12-bit
        prescaleval //= float(freq)
        prescaleval -= 1.0
        
        if self.debug:
            print(f"모터 PWM 주파수를 {freq} Hz로 설정")
        
        prescale = math.floor(prescaleval + 0.5)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.write(self.__MODE1, newmode)   # sleep 모드로
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)
        self.read(self.__MODE1)

    def setPWM(self, channel, on, off):
        """단일 PWM 채널 설정"""
        self.write(self.__LED0_ON_L + 4*channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4*channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4*channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4*channel, off >> 8)
        
        if self.debug:
            print(f"모터 채널: {channel}  LED_ON: {on} LED_OFF: {off}")

    def setDutycycle(self, channel, pulse):
        """듀티 사이클 설정"""
        self.setPWM(channel, 0, int(pulse * (4096 // 100)))

    def setLevel(self, channel, value):
        """레벨 설정 (0 또는 1)"""
        if value == 1:
            self.setPWM(channel, 0, 4095)
        else:
            self.setPWM(channel, 0, 0)

class ServoDriver:
    """서보 모터 드라이버 클래스 (사용자 제공 코드 기반)"""
    
    def __init__(self, address=0x60, debug=True):
        self.debug = debug
        self.address = address
        
        if self.debug:
            print(f"서보 드라이버 초기화 중... (주소: 0x{address:02X})")
        
        try:
            # I2C 초기화
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            # PCA9685 객체 생성 (서보 주소 0x60 사용)
            self.pca = PCA9685(self.i2c, address=address)
            self.pca.frequency = 50  # 서보에 맞는 주파수 설정
            
            # 서보 객체 생성 (채널 7번)
            self.servo0 = servo.Servo(self.pca.channels[7])
            
            print("서보 드라이버 초기화 성공")
        except Exception as e:
            print(f"서보 드라이버 초기화 실패: {e}")
            raise
    
    def set_servo_angle(self, angle):
        """서보모터 각도 설정"""
        try:
            self.servo0.angle = angle
            if self.debug:
                print(f"서보 각도 설정: {angle}° (채널 7)")
        except Exception as e:
            print(f"서보 각도 설정 오류: {e}")

class MotorServoSimultaneousTest:
    """모터와 서보모터 동시 구동 테스트"""
    
    def __init__(self):
        # 모터 드라이버 초기화 (주소 0x40)
        self.motor_driver = MotorDriver(address=0x40, debug=True, bus_number=7)
        self.motor_driver.setPWMFreq(50)
        
        # 서보 드라이버 초기화 (주소 0x60)
        self.servo_driver = ServoDriver(address=0x60, debug=True)
        
        print("모터-서보 동시 구동 테스트 시스템 초기화 완료")
        print("모터 주소: 0x40, 서보 주소: 0x60, 서보 채널: 7번")
    
    def motor_test_sequence(self):
        """모터 테스트 시퀀스"""
        print("🔧 모터 테스트 시작")
        
        # 모터 제어 테스트
        print("1. 모터 듀티 사이클 50% 설정")
        self.motor_driver.setDutycycle(0, 50)  # 채널 0, 50% 듀티 사이클
        time.sleep(2)
        
        print("2. 모터 레벨 HIGH 설정")
        self.motor_driver.setLevel(1, 1)  # 채널 1 HIGH
        time.sleep(2)
        
        print("3. 모터 레벨 LOW 설정")
        self.motor_driver.setLevel(1, 0)  # 채널 1 LOW
        time.sleep(2)
        
        print("4. 모터 듀티 사이클 0% 설정 (정지)")
        self.motor_driver.setDutycycle(0, 0)
        
        print("✅ 모터 테스트 완료")
    
    def servo_test_sequence(self):
        """서보모터 테스트 시퀀스"""
        print("⚙️ 서보모터 테스트 시작")
        
        # 서보 테스트 동작 (사용자 제공 코드 기반)
        print("서보모터 0도에서 180도까지 30도씩 회전")
        for angle in range(0, 180, 30):
            self.servo_driver.set_servo_angle(angle)
            time.sleep(0.5)
        
        print("서보모터 180도에서 0도까지 30도씩 회전")
        for angle in range(180, -1, -30):
            self.servo_driver.set_servo_angle(angle)
            time.sleep(0.5)
        
        # 종료 전 각도 초기화
        self.servo_driver.set_servo_angle(90)
        print("✅ 서보모터 테스트 완료")
    
    def simultaneous_test(self):
        """모터와 서보모터 동시 구동 테스트"""
        print("🚀 동시 구동 테스트 시작")
        
        def motor_task():
            """모터 제어 태스크"""
            print("🔧 모터 태스크 시작")
            
            # 모터 동작 시퀀스
            for i in range(3):
                print(f"모터 사이클 {i+1}/3")
                
                # 모터 켜기
                self.motor_driver.setDutycycle(0, 30)  # 30% 듀티 사이클
                self.motor_driver.setLevel(1, 1)  # 방향 제어 HIGH
                time.sleep(1)
                
                # 모터 끄기
                self.motor_driver.setDutycycle(0, 0)
                self.motor_driver.setLevel(1, 0)
                time.sleep(0.5)
            
            print("✅ 모터 태스크 완료")
        
        def servo_task():
            """서보 제어 태스크"""
            print("⚙️ 서보 태스크 시작")
            
            # 서보 동작 시퀀스
            for i in range(3):
                print(f"서보 사이클 {i+1}/3")
                
                # 0도에서 180도까지
                for angle in range(0, 181, 45):
                    self.servo_driver.set_servo_angle(angle)
                    time.sleep(0.3)
                
                # 180도에서 0도까지
                for angle in range(180, -1, -45):
                    self.servo_driver.set_servo_angle(angle)
                    time.sleep(0.3)
                
                # 90도로 초기화
                self.servo_driver.set_servo_angle(90)
                time.sleep(0.5)
            
            print("✅ 서보 태스크 완료")
        
        # 스레드로 동시 실행
        motor_thread = threading.Thread(target=motor_task)
        servo_thread = threading.Thread(target=servo_task)
        
        print("🔄 모터와 서보 동시 실행 시작")
        motor_thread.start()
        servo_thread.start()
        
        # 스레드 완료 대기
        motor_thread.join()
        servo_thread.join()
        
        print("✅ 동시 구동 테스트 완료")
    
    def run_all_tests(self):
        """모든 테스트 실행"""
        print("=" * 60)
        print("🚀 모터-서보 동시 구동 테스트 시작")
        print("=" * 60)
        
        try:
            # 1. 개별 테스트
            print("\n📋 1단계: 개별 테스트")
            print("-" * 40)
            
            print("\n🔧 모터 드라이버 테스트")
            self.motor_test_sequence()
            
            print("\n⚙️ 서보모터 드라이버 테스트")
            self.servo_test_sequence()
            
            # 2. 동시 구동 테스트
            print("\n📋 2단계: 동시 구동 테스트")
            print("-" * 40)
            self.simultaneous_test()
            
            print("\n🎉 모든 테스트 완료!")
            
        except KeyboardInterrupt:
            print("\n⚠️ 테스트 중단됨")
        except Exception as e:
            print(f"\n❌ 테스트 오류: {e}")
        finally:
            # 정리
            print("\n🧹 정리 작업")
            self.motor_driver.setDutycycle(0, 0)  # 모터 정지
            self.motor_driver.setLevel(1, 0)
            self.servo_driver.set_servo_angle(90)  # 서보 중립 위치
            print("정리 완료")

def main():
    """메인 함수"""
    test = MotorServoSimultaneousTest()
    test.run_all_tests()

if __name__ == "__main__":
    main() 