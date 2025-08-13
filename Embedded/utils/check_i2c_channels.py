#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson Orin Nano I2C 채널 확인 스크립트
"""

import os
import subprocess
import time

def check_i2c_channels():
    print("=== Jetson Orin Nano I2C 채널 확인 ===")
    
    # 사용 가능한 I2C 버스 확인
    i2c_buses = []
    for i in range(10):  # 0-9번까지 확인
        bus_path = f"/dev/i2c-{i}"
        if os.path.exists(bus_path):
            i2c_buses.append(i)
            print(f"✓ I2C 버스 {i} 발견: {bus_path}")
        else:
            print(f"✗ I2C 버스 {i} 없음")
    
    print(f"\n총 {len(i2c_buses)}개의 I2C 버스 발견")
    
    # 각 버스에서 장치 스캔
    for bus in i2c_buses:
        print(f"\n--- I2C 버스 {bus} 스캔 ---")
        try:
            result = subprocess.run(['i2cdetect', '-y', str(bus)], 
                                  capture_output=True, text=True, timeout=10)
            print(result.stdout)
        except subprocess.TimeoutExpired:
            print(f"버스 {bus} 스캔 타임아웃")
        except Exception as e:
            print(f"버스 {bus} 스캔 실패: {e}")
    
    # 권한 확인
    print("\n--- I2C 권한 확인 ---")
    for bus in i2c_buses:
        bus_path = f"/dev/i2c-{bus}"
        try:
            stat = os.stat(bus_path)
            print(f"버스 {bus}: {oct(stat.st_mode)[-3:]} (소유자: {stat.st_uid})")
        except Exception as e:
            print(f"버스 {bus} 권한 확인 실패: {e}")

def test_i2c_access():
    print("\n=== I2C 접근 테스트 ===")
    
    import smbus
    
    for bus_num in range(10):
        try:
            print(f"\n버스 {bus_num} 접근 테스트...")
            bus = smbus.SMBus(bus_num)
            
            # 간단한 읽기 테스트 (MPU6050 주소 0x68)
            try:
                bus.write_byte_data(0x68, 0x6B, 0)
                print(f"✓ 버스 {bus_num}: MPU6050 (0x68) 접근 성공")
            except Exception as e:
                print(f"✗ 버스 {bus_num}: MPU6050 접근 실패 - {e}")
            
            # PCA9685 주소 0x40 테스트 (모터 드라이버)
            try:
                bus.write_byte_data(0x40, 0x00, 0x20)
                print(f"✓ 버스 {bus_num}: PCA9685 (0x40) 모터 드라이버 접근 성공")
            except Exception as e:
                print(f"✗ 버스 {bus_num}: PCA9685 (0x40) 모터 드라이버 접근 실패 - {e}")
            
            # PCA9685 주소 0x60 테스트 (서보 모터 드라이버)
            try:
                bus.write_byte_data(0x60, 0x00, 0x20)
                print(f"✓ 버스 {bus_num}: PCA9685 (0x60) 서보 모터 드라이버 접근 성공")
            except Exception as e:
                print(f"✗ 버스 {bus_num}: PCA9685 (0x60) 서보 모터 드라이버 접근 실패 - {e}")

            # BNO08x 주소 0x4A/0x4B 테스트 (간단 접근 시도)
            for bno_addr in [0x4A, 0x4B]:
                try:
                    # SHTP 프로토콜 장치이므로 단순 레지스터 없음, 바이트 읽기 시도
                    bus.read_byte_data(bno_addr, 0x00)
                    print(f"✓ 버스 {bus_num}: BNO08x 추정 (0x{bno_addr:02X}) 응답 확인")
                except Exception as e:
                    print(f"✗ 버스 {bus_num}: BNO08x 추정 (0x{bno_addr:02X}) 접근 실패 - {e}")
            
            bus.close()
            
        except Exception as e:
            print(f"✗ 버스 {bus_num} 열기 실패: {e}")

if __name__ == "__main__":
    check_i2c_channels()
    test_i2c_access()
