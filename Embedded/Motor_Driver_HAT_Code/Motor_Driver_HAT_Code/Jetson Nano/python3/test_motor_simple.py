#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
간단한 모터 동작 테스트
"""

import time
from ai_motor_controller import AIMotorController

def test_motor_simple():
    """간단한 모터 동작 테스트"""
    print("=" * 60)
    print("간단한 모터 동작 테스트")
    print("=" * 60)
    
    try:
        # AI 모터 컨트롤러 초기화 (백엔드 연결 없이)
        motor = AIMotorController(
            debug=True, 
            api_url=None,
            backend_broker=None,
            backend_port=1883
        )
        
        print("모터 컨트롤러 초기화 완료")
        print(f"PWM 주파수: 20Hz")
        print(f"모터 속도 설정: {motor.motor_speeds}")
        
        # 1. 모터 A만 테스트
        print("\n1. 모터 A 전진 테스트 (5초)")
        motor.set_motor_speed(0, motor.FORWARD, 40)
        time.sleep(5)
        
        # 2. 모터 A 정지
        print("\n2. 모터 A 정지")
        motor.stop_motor(0)
        time.sleep(2)
        
        # 3. 모터 B만 테스트
        print("\n3. 모터 B 전진 테스트 (5초)")
        motor.set_motor_speed(1, motor.FORWARD, 40)
        time.sleep(5)
        
        # 4. 모터 B 정지
        print("\n4. 모터 B 정지")
        motor.stop_motor(1)
        time.sleep(2)
        
        # 5. 양쪽 모터 동시 테스트
        print("\n5. 양쪽 모터 동시 전진 테스트 (5초)")
        motor.differential_drive(40, 40)
        time.sleep(5)
        
        # 6. 양쪽 모터 정지
        print("\n6. 양쪽 모터 정지")
        motor.stop_all()
        
        print("\n모터 테스트 완료!")
        
    except Exception as e:
        print(f"모터 테스트 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'motor' in locals():
            motor.stop_all()

def test_motor_status():
    """모터 상태 확인 테스트"""
    print("\n" + "=" * 60)
    print("모터 상태 확인 테스트")
    print("=" * 60)
    
    try:
        motor = AIMotorController(
            debug=True, 
            api_url=None,
            backend_broker=None,
            backend_port=1883
        )
        
        # 초기 상태
        status = motor.get_motor_status()
        print(f"초기 상태: {status}")
        
        # 모터 A 동작
        motor.set_motor_speed(0, motor.FORWARD, 40)
        time.sleep(1)
        status = motor.get_motor_status()
        print(f"모터 A 동작 후: {status}")
        
        # 모터 B 동작
        motor.set_motor_speed(1, motor.BACKWARD, 40)
        time.sleep(1)
        status = motor.get_motor_status()
        print(f"모터 B 동작 후: {status}")
        
        # 정지
        motor.stop_all()
        status = motor.get_motor_status()
        print(f"정지 후: {status}")
        
    except Exception as e:
        print(f"상태 확인 테스트 오류: {e}")
    finally:
        if 'motor' in locals():
            motor.stop_all()

if __name__ == "__main__":
    test_motor_simple()
    test_motor_status()

