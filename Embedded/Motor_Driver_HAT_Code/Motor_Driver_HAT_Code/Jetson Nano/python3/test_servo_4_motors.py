#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
4개 서보모터 동시 제어 테스트
"""

import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def test_4_servos():
    """4개 서보모터 동시 테스트"""
    print("=" * 60)
    print("4개 서보모터 동시 제어 테스트")
    print("=" * 60)
    
    try:
        # I2C 초기화
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x60)  # 서보모터 드라이버 주소
        pca.frequency = 50
        
        # 4개 서보모터 생성 (채널 0, 1, 2, 3)
        servo0 = servo.Servo(pca.channels[0])
        servo1 = servo.Servo(pca.channels[1])
        servo2 = servo.Servo(pca.channels[2])
        servo3 = servo.Servo(pca.channels[3])
        
        print("4개 서보모터 초기화 완료")
        print("각도 테스트 시작...")
        
        # 모든 서보모터를 90도로 초기화
        print("모든 서보모터를 90도로 초기화")
        servo0.angle = 90
        servo1.angle = 90
        servo2.angle = 90
        servo3.angle = 90
        time.sleep(2)
        
        # 테스트 1: 모든 서보모터를 0도로
        print("테스트 1: 모든 서보모터를 0도로")
        servo0.angle = 0
        servo1.angle = 0
        servo2.angle = 0
        servo3.angle = 0
        time.sleep(2)
        
        # 테스트 2: 모든 서보모터를 180도로
        print("테스트 2: 모든 서보모터를 180도로")
        servo0.angle = 180
        servo1.angle = 180
        servo2.angle = 180
        servo3.angle = 180
        time.sleep(2)
        
        # 테스트 3: 모든 서보모터를 90도로 복원
        print("테스트 3: 모든 서보모터를 90도로 복원")
        servo0.angle = 90
        servo1.angle = 90
        servo2.angle = 90
        servo3.angle = 90
        time.sleep(2)
        
        print("✅ 4개 서보모터 테스트 완료")
        
    except Exception as e:
        print(f"❌ 4개 서보모터 테스트 실패: {e}")

def test_servo_sequence():
    """서보모터 순차 동작 테스트"""
    print("\n" + "=" * 60)
    print("서보모터 순차 동작 테스트")
    print("=" * 60)
    
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x60)
        pca.frequency = 50
        
        # 4개 서보모터 생성
        servos = [
            servo.Servo(pca.channels[0]),
            servo.Servo(pca.channels[1]),
            servo.Servo(pca.channels[2]),
            servo.Servo(pca.channels[3])
        ]
        
        print("서보모터 순차 동작 테스트 시작")
        
        # 각도 시퀀스 테스트
        angles = [0, 45, 90, 135, 180]
        
        for angle in angles:
            print(f"모든 서보모터를 {angle}도로 설정")
            for i, servo_motor in enumerate(servos):
                servo_motor.angle = angle
                print(f"  서보모터 {i}: {angle}도")
            time.sleep(2)
        
        # 90도로 복원
        print("모든 서보모터를 90도로 복원")
        for i, servo_motor in enumerate(servos):
            servo_motor.angle = 90
            print(f"  서보모터 {i}: 90도")
        time.sleep(2)
        
        print("✅ 서보모터 순차 동작 테스트 완료")
        
    except Exception as e:
        print(f"❌ 서보모터 순차 동작 테스트 실패: {e}")

def test_servo_wave():
    """서보모터 웨이브 동작 테스트"""
    print("\n" + "=" * 60)
    print("서보모터 웨이브 동작 테스트")
    print("=" * 60)
    
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=0x60)
        pca.frequency = 50
        
        # 4개 서보모터 생성
        servos = [
            servo.Servo(pca.channels[0]),
            servo.Servo(pca.channels[1]),
            servo.Servo(pca.channels[2]),
            servo.Servo(pca.channels[3])
        ]
        
        print("서보모터 웨이브 동작 테스트 시작")
        
        # 웨이브 동작 (각 서보모터가 순차적으로 움직임)
        for cycle in range(3):  # 3번 반복
            print(f"웨이브 사이클 {cycle + 1}")
            
            # 각 서보모터를 순차적으로 0도에서 180도로
            for i, servo_motor in enumerate(servos):
                print(f"  서보모터 {i}: 0도 → 180도")
                servo_motor.angle = 0
                time.sleep(0.5)
                servo_motor.angle = 180
                time.sleep(0.5)
            
            # 각 서보모터를 순차적으로 180도에서 0도로
            for i, servo_motor in enumerate(servos):
                print(f"  서보모터 {i}: 180도 → 0도")
                servo_motor.angle = 180
                time.sleep(0.5)
                servo_motor.angle = 0
                time.sleep(0.5)
        
        # 모든 서보모터를 90도로 복원
        print("모든 서보모터를 90도로 복원")
        for i, servo_motor in enumerate(servos):
            servo_motor.angle = 90
            print(f"  서보모터 {i}: 90도")
        time.sleep(2)
        
        print("✅ 서보모터 웨이브 동작 테스트 완료")
        
    except Exception as e:
        print(f"❌ 서보모터 웨이브 동작 테스트 실패: {e}")

if __name__ == "__main__":
    test_4_servos()
    test_servo_sequence()
    test_servo_wave()
