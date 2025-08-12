#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
서보모터 간단 테스트 (사용자 제공 코드)
"""

import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

print("🚀 서보모터 테스트 시작")

try:
    # I2C 초기화
    i2c = busio.I2C(board.SCL, board.SDA)
    print("✅ I2C 초기화 성공")

    # PCA9685 객체 생성
    pca = PCA9685(i2c)
    pca.frequency = 50  # 서보에 맞는 주파수 설정
    print("✅ PCA9685 초기화 성공")

    # 서보 객체 생성 (채널 0번) 채널 변경해서 테스트
    servo0 = servo.Servo(pca.channels[0])
    print("✅ 서보 객체 생성 성공")

    print("\n⚙️ 서보 테스트 동작 시작")
    # 서보 테스트 동작
    for angle in range(0, 180, 30):
        print(f"서보 각도 설정: {angle}°")
        servo0.angle = angle
        time.sleep(0.5)

    # 종료 전 각도 초기화
    print("서보 각도 초기화: 90°")
    servo0.angle = 90
    
    print("✅ 서보모터 테스트 완료!")

except Exception as e:
    print(f"❌ 오류 발생: {e}")
    print("하드웨어 연결 상태를 확인해주세요.") 