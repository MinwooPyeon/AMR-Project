#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
서보모터 4개 테스트
"""

import time
import threading
from main.amr_real_data_sync import AMRRealDataSync
from utils.logger import motor_logger

def test_servo_motors():
    motor_logger.info("=== 서보모터 4개 테스트 ===")
    motor_logger.info("PCA9685 드라이버를 사용하여 서보모터 4개 제어")
    motor_logger.info("각 서보모터는 0-180도 범위에서 동작")
    motor_logger.info("=" * 60)
    
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=False, enable_backup=False)
    
    servo_status = amr_sync.get_servo_status()
    print(f"\n📊 서보모터 컨트롤러 상태:")
    print(f"  - 초기화됨: {'예' if servo_status.get('initialized', False) else '아니오'}")
    print(f"  - I2C 주소: {servo_status.get('i2c_address', 'N/A')}")
    print(f"  - PWM 주파수: {servo_status.get('frequency', 'N/A')}Hz")
    
    if not servo_status.get('initialized', False):
        print("❌ 서보모터 컨트롤러 초기화 실패")
        return
    
    print(f"\n🔧 서보모터 채널:")
    servo_channels = servo_status.get('servo_channels', {})
    for servo_name, channel in servo_channels.items():
        print(f"  - {servo_name}: 채널 {channel}")
    
    try:
        print(f"\n🧪 서보모터 테스트 시작...")
        
        print("\n1. 모든 서보모터 90도 초기화")
        amr_sync.set_all_servos(90)
        time.sleep(2)
        
        print("\n2. 개별 서보모터 테스트")
        test_angles = [0, 45, 90, 135, 180]
        
        for servo_name in servo_channels.keys():
            print(f"\n   {servo_name} 테스트:")
            for angle in test_angles:
                amr_sync.set_servo_angle(servo_name, angle)
                print(f"     → {angle}도")
                time.sleep(0.5)
        
        print("\n3. 모든 서보모터 90도로 리셋")
        amr_sync.reset_all_servos()
        time.sleep(2)
        
        print("\n4. 동시에 여러 서보모터 제어")
        test_sequences = [
            {"servo1": 0, "servo2": 45, "servo3": 90, "servo4": 135},
            {"servo1": 45, "servo2": 90, "servo3": 135, "servo4": 180},
            {"servo1": 90, "servo2": 135, "servo3": 180, "servo4": 0},
            {"servo1": 135, "servo2": 180, "servo3": 0, "servo4": 45},
            {"servo1": 180, "servo2": 0, "servo3": 45, "servo4": 90}
        ]
        
        for i, angles in enumerate(test_sequences, 1):
            print(f"   시퀀스 {i}: {angles}")
            amr_sync.set_servo_angles(angles)
            time.sleep(1)
        
        print("\n5. servo1 스윕 테스트")
        amr_sync.sweep_servo("servo1", 0, 180, 10, 0.1)
        
        print("\n6. 최종 상태 확인")
        final_angles = amr_sync.get_all_servo_angles()
        for servo_name, angle in final_angles.items():
            print(f"   {servo_name}: {angle}도")
        
        print("\n7. 모든 서보모터 90도로 최종 리셋")
        amr_sync.reset_all_servos()
        time.sleep(2)
        
        print("\n✅ 서보모터 4개 테스트 완료")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
        amr_sync.reset_all_servos()
    
    print("\n" + "=" * 60)
    print("=== 최종 통계 ===")
    print("=" * 60)
    
    servo_status = amr_sync.get_servo_status()
    print(f"\n🔧 서보모터 컨트롤러 상태:")
    print(f"  - 초기화됨: {'예' if servo_status.get('initialized', False) else '아니오'}")
    print(f"  - I2C 주소: {servo_status.get('i2c_address', 'N/A')}")
    print(f"  - PWM 주파수: {servo_status.get('frequency', 'N/A')}Hz")
    
    current_angles = servo_status.get('current_angles', {})
    if current_angles:
        print(f"\n📐 현재 서보모터 각도:")
        for servo_name, angle in current_angles.items():
            print(f"  - {servo_name}: {angle}도")
    
    angle_range = servo_status.get('angle_range', {})
    if angle_range:
        print(f"\n📏 서보모터 각도 범위:")
        print(f"  - 최소: {angle_range.get('min', 'N/A')}도")
        print(f"  - 최대: {angle_range.get('max', 'N/A')}도")
    
    pulse_range = servo_status.get('pulse_range', {})
    if pulse_range:
        print(f"\n⚡ PWM 펄스 범위:")
        print(f"  - 최소: {pulse_range.get('min', 'N/A')}μs")
        print(f"  - 최대: {pulse_range.get('max', 'N/A')}μs")
    
    print(f"\n💡 서보모터 제어 방식:")
    print(f"  - 드라이버: PCA9685 PWM 컨트롤러")
    print(f"  - 통신: I2C (주소: 0x40)")
    print(f"  - PWM 주파수: 50Hz")
    print(f"  - 서보모터 수: 4개 (채널 0-3)")
    print(f"  - 각도 범위: 0-180도")
    print(f"  - 기본 각도: 90도")
    
    print("\n✅ 서보모터 4개 테스트 완료")

if __name__ == "__main__":
    test_servo_motors() 