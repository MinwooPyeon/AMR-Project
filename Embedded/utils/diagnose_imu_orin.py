#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson Orin Nano IMU 센서 인식 진단 스크립트
"""

import os
import subprocess
import time
import smbus

def check_system_info():
    """시스템 정보 확인"""
    print("=== 시스템 정보 ===")
    
    # Jetson 모델 확인
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read().strip()
            print(f"모델: {model}")
    except:
        print("모델 정보를 읽을 수 없습니다.")
    
    # I2C 장치 확인
    i2c_devices = []
    for i in range(10):
        if os.path.exists(f"/dev/i2c-{i}"):
            i2c_devices.append(i)
    
    print(f"사용 가능한 I2C 버스: {i2c_devices}")
    
    # I2C 권한 확인
    for bus in i2c_devices:
        try:
            stat = os.stat(f"/dev/i2c-{bus}")
            print(f"버스 {bus} 권한: {oct(stat.st_mode)[-3:]} (소유자: {stat.st_uid})")
        except Exception as e:
            print(f"버스 {bus} 권한 확인 실패: {e}")

def scan_all_i2c_buses():
    """모든 I2C 버스 스캔"""
    print("\n=== I2C 버스 스캔 ===")
    
    for bus_num in range(10):
        if not os.path.exists(f"/dev/i2c-{bus_num}"):
            continue
            
        print(f"\n--- 버스 {bus_num} 스캔 ---")
        try:
            result = subprocess.run(['i2cdetect', '-y', str(bus_num)], 
                                  capture_output=True, text=True, timeout=5)
            print(result.stdout)
        except Exception as e:
            print(f"스캔 실패: {e}")

def test_imu_detection():
    """IMU 센서 감지 테스트"""
    print("\n=== IMU 센서 감지 테스트 ===")
    
    imu_found = False
    
    for bus_num in range(10):
        if not os.path.exists(f"/dev/i2c-{bus_num}"):
            continue
            
        print(f"\n버스 {bus_num}에서 IMU 검색 중...")
        
        try:
            bus = smbus.SMBus(bus_num)
            
            # MPU6050 WHO_AM_I 레지스터 읽기 (0x75)
            try:
                # PWR_MGMT_1 레지스터 초기화
                bus.write_byte_data(0x68, 0x6B, 0)
                time.sleep(0.1)
                
                # WHO_AM_I 레지스터 읽기
                who_am_i = bus.read_byte_data(0x68, 0x75)
                print(f"  주소 0x68 WHO_AM_I: 0x{who_am_i:02X}")
                
                if who_am_i == 0x68:
                    print(f"  ✅ IMU 발견! - 버스 {bus_num}, 주소 0x68")
                    imu_found = True
                else:
                    print(f"  ❌ 잘못된 WHO_AM_I 값: 0x{who_am_i:02X} (예상: 0x68)")
                    
            except Exception as e:
                print(f"  ❌ 주소 0x68 접근 실패: {e}")
            
            # 주소 0x69도 확인 (AD0 핀이 HIGH인 경우)
            try:
                bus.write_byte_data(0x69, 0x6B, 0)
                time.sleep(0.1)
                
                who_am_i = bus.read_byte_data(0x69, 0x75)
                print(f"  주소 0x69 WHO_AM_I: 0x{who_am_i:02X}")
                
                if who_am_i == 0x68:
                    print(f"  ✅ IMU 발견! - 버스 {bus_num}, 주소 0x69")
                    imu_found = True
                else:
                    print(f"  ❌ 잘못된 WHO_AM_I 값: 0x{who_am_i:02X} (예상: 0x68)")
                    
            except Exception as e:
                print(f"  ❌ 주소 0x69 접근 실패: {e}")
            
            bus.close()
            
        except Exception as e:
            print(f"  ❌ 버스 {bus_num} 열기 실패: {e}")
    
    if not imu_found:
        print("\n❌ IMU 센서를 찾을 수 없습니다!")
        print("\n가능한 원인:")
        print("1. 하드웨어 연결 문제")
        print("2. 전원 공급 문제")
        print("3. I2C 버스 활성화 문제")
        print("4. 권한 문제")
    else:
        print("\n✅ IMU 센서가 정상적으로 감지되었습니다!")

def test_pca9685_detection():
    """PCA9685 드라이버 감지 테스트"""
    print("\n=== PCA9685 드라이버 감지 테스트 ===")
    
    addresses = [0x40, 0x60]  # 모터 드라이버, 서보 드라이버
    
    for bus_num in range(10):
        if not os.path.exists(f"/dev/i2c-{bus_num}"):
            continue
            
        print(f"\n버스 {bus_num}에서 PCA9685 검색 중...")
        
        try:
            bus = smbus.SMBus(bus_num)
            
            for addr in addresses:
                try:
                    # MODE1 레지스터 읽기
                    mode1 = bus.read_byte_data(addr, 0x00)
                    print(f"  주소 0x{addr:02X} MODE1: 0x{mode1:02X}")
                    print(f"  ✅ PCA9685 발견! - 버스 {bus_num}, 주소 0x{addr:02X}")
                    
                except Exception as e:
                    print(f"  ❌ 주소 0x{addr:02X} 접근 실패: {e}")
            
            bus.close()
            
        except Exception as e:
            print(f"  ❌ 버스 {bus_num} 열기 실패: {e}")

def suggest_solutions():
    """해결 방법 제안"""
    print("\n=== 해결 방법 제안 ===")
    
    print("\n1. I2C 활성화:")
    print("   sudo modprobe i2c-dev")
    print("   sudo usermod -a -G i2c $USER")
    print("   sudo chmod 666 /dev/i2c-*")
    print("   sudo reboot")
    
    print("\n2. 하드웨어 연결 확인:")
    print("   - VCC → 3.3V")
    print("   - GND → GND")
    print("   - SDA → I2C SDA 핀")
    print("   - SCL → I2C SCL 핀")
    print("   - AD0 → GND (주소 0x68) 또는 3.3V (주소 0x69)")
    
    print("\n3. 전원 공급 확인:")
    print("   - 안정적인 3.3V 전원")
    print("   - 전류 공급 능력 확인")
    
    print("\n4. 풀업 저항 확인:")
    print("   - SDA, SCL에 4.7kΩ 풀업 저항")
    
    print("\n5. 배선 확인:")
    print("   - 접촉 불량 확인")
    print("   - 배선 길이 (너무 길면 안됨)")
    print("   - 노이즈 간섭 확인")

def main():
    print("=== Jetson Orin Nano IMU 센서 인식 진단 ===")
    
    check_system_info()
    scan_all_i2c_buses()
    test_imu_detection()
    test_pca9685_detection()
    suggest_solutions()
    
    print("\n=== 진단 완료 ===")

if __name__ == "__main__":
    main()
