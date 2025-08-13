#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
다른 I2C 장치로 연결 테스트
PCA9685가 감지되지 않을 때 다른 장치로 I2C 연결을 확인합니다.
"""

import smbus
import time

def test_common_i2c_devices():
    """일반적인 I2C 장치들 테스트"""
    print("=== 일반적인 I2C 장치 테스트 ===")
    
    # 일반적인 I2C 장치 주소들
    common_devices = {
        0x20: "PCF8574 (I/O 확장)",
        0x21: "PCF8574A (I/O 확장)",
        0x27: "PCF8574 (LCD)",
        0x3C: "OLED Display",
        0x3D: "OLED Display",
        0x40: "PCA9685 (PWM)",
        0x41: "PCA9685 (PWM)",
        0x42: "PCA9685 (PWM)",
        0x43: "PCA9685 (PWM)",
        0x48: "ADS1115 (ADC)",
        0x49: "ADS1115 (ADC)",
        0x4A: "ADS1115 (ADC)",
        0x4B: "ADS1115 (ADC)",
        0x50: "EEPROM",
        0x51: "EEPROM",
        0x52: "EEPROM",
        0x53: "EEPROM",
        0x68: "DS3231 (RTC)",
        0x69: "MPU6050 (IMU)",
        0x4A: "BNO08x (IMU)",
        0x4B: "BNO08x (IMU)",
        0x76: "BME280 (센서)",
        0x77: "BME280 (센서)",
    }
    
    for bus in [0, 1, 2]:
        print(f"\n📡 I2C 버스 {bus} 스캔:")
        
        try:
            bus_obj = smbus.SMBus(bus)
            found_devices = []
            
            for addr, device_name in common_devices.items():
                try:
                    # Quick command로 장치 존재 확인
                    bus_obj.write_quick(addr)
                    found_devices.append((addr, device_name))
                    print(f"  ✅ 0x{addr:02X}: {device_name}")
                except Exception as e:
                    # 오류 메시지 숨김 (너무 많은 출력 방지)
                    pass
            
            bus_obj.close()
            
            if found_devices:
                print(f"  🎯 발견된 장치: {len(found_devices)}개")
                for addr, name in found_devices:
                    print(f"    - 0x{addr:02X}: {name}")
            else:
                print(f"  ❌ 발견된 장치 없음")
                
        except Exception as e:
            print(f"  ❌ 버스 {bus} 접근 실패: {e}")

def test_simple_i2c_communication():
    """간단한 I2C 통신 테스트"""
    print("\n=== 간단한 I2C 통신 테스트 ===")
    
    for bus in [0, 1, 2]:
        print(f"\n🔧 버스 {bus} 통신 테스트:")
        
        try:
            bus_obj = smbus.SMBus(bus)
            
            # 간단한 읽기/쓰기 테스트
            test_address = 0x40  # PCA9685 기본 주소
            
            try:
                # 읽기 테스트
                data = bus_obj.read_byte_data(test_address, 0x00)
                print(f"  ✅ 읽기 성공: 0x{data:02X}")
            except Exception as e:
                print(f"  ❌ 읽기 실패: {e}")
            
            try:
                # 쓰기 테스트
                bus_obj.write_byte_data(test_address, 0x00, 0x00)
                print(f"  ✅ 쓰기 성공")
            except Exception as e:
                print(f"  ❌ 쓰기 실패: {e}")
            
            bus_obj.close()
            
        except Exception as e:
            print(f"  ❌ 버스 {bus} 접근 실패: {e}")

def test_pca9685_specific_registers():
    """PCA9685 특정 레지스터 테스트"""
    print("\n=== PCA9685 특정 레지스터 테스트 ===")
    
    for bus in [0, 1, 2]:
        print(f"\n🎯 버스 {bus} PCA9685 레지스터 테스트:")
        
        try:
            bus_obj = smbus.SMBus(bus)
            
            # PCA9685 레지스터 주소들
            registers = {
                0x00: "MODE1",
                0x01: "MODE2", 
                0xFE: "PRE_SCALE",
                0x06: "LED0_ON_L",
                0x07: "LED0_ON_H",
                0x08: "LED0_OFF_L",
                0x09: "LED0_OFF_H"
            }
            
            for addr in [0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47]:
                print(f"  주소 0x{addr:02X} 테스트:")
                
                try:
                    # MODE1 레지스터 읽기
                    mode1 = bus_obj.read_byte_data(addr, 0x00)
                    print(f"    ✅ MODE1: 0x{mode1:02X}")
                    
                    # 다른 레지스터들도 읽기 시도
                    for reg_addr, reg_name in registers.items():
                        try:
                            value = bus_obj.read_byte_data(addr, reg_addr)
                            print(f"    ✅ {reg_name}: 0x{value:02X}")
                        except:
                            pass
                            
                except Exception as e:
                    print(f"    ❌ 실패: {e}")
            
            bus_obj.close()
            
        except Exception as e:
            print(f"  ❌ 버스 {bus} 접근 실패: {e}")

def provide_troubleshooting_tips():
    """문제 해결 팁 제공"""
    print("\n=== 문제 해결 팁 ===")
    print("1. 하드웨어 연결 확인:")
    print("   - 전원 공급 (3.3V)")
    print("   - GND 연결")
    print("   - SDA, SCL 핀 연결")
    print("   - 케이블 상태")
    
    print("\n2. 전압 측정:")
    print("   - VCC: 3.3V")
    print("   - GND: 0V")
    print("   - SDA: 약 3.3V (풀업)")
    print("   - SCL: 약 3.3V (풀업)")
    
    print("\n3. 대안 테스트:")
    print("   - 다른 I2C 장치 연결")
    print("   - 다른 PCA9685 모듈 시도")
    print("   - 다른 케이블 사용")
    
    print("\n4. 소프트웨어 확인:")
    print("   - I2C 모듈 로드: sudo modprobe i2c-dev")
    print("   - 권한 확인: sudo usermod -a -G i2c $USER")
    print("   - 재부팅 후 테스트")

def main():
    print("🔧 다른 I2C 장치 테스트")
    print("=" * 50)
    
    # 1. 일반적인 I2C 장치 스캔
    test_common_i2c_devices()
    
    # 2. 간단한 I2C 통신 테스트
    test_simple_i2c_communication()
    
    # 3. PCA9685 특정 레지스터 테스트
    test_pca9685_specific_registers()
    
    # 4. 문제 해결 팁
    provide_troubleshooting_tips()
    
    print("\n=== 결론 ===")
    print("모든 I2C 버스에서 장치가 감지되지 않는다면:")
    print("1. 하드웨어 연결 문제일 가능성이 높습니다")
    print("2. 전압 측정으로 연결 상태 확인하세요")
    print("3. 다른 I2C 장치로 테스트해보세요")
    print("4. PCA9685 모듈 자체에 문제가 있을 수 있습니다")

if __name__ == "__main__":
    main() 