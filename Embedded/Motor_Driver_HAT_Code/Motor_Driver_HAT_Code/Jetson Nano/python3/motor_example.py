#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Jetson Nano Motor Driver HAT 사용 예제
간단한 모터 제어 방법
"""

from motor_control_enhanced import MotorDriverHAT
import time

def simple_motor_control():
    """간단한 모터 제어 예제"""
    print("🚀 간단한 모터 제어 예제")
    
    # 모터 드라이버 초기화
    motor = MotorDriverHAT(debug=True)
    
    try:
        # 1. 개별 모터 제어
        print("\n1️⃣ 개별 모터 제어")
        
        # 모터 A 전진
        motor.set_motor_speed(0, 'forward', 50)
        time.sleep(2)
        motor.stop_motor(0)
        
        # 모터 B 전진
        motor.set_motor_speed(1, 'forward', 50)
        time.sleep(2)
        motor.stop_motor(1)
        
        # 2. 동시 제어
        print("\n2️⃣ 동시 제어")
        motor.set_motor_speed(0, 'forward', 30)
        motor.set_motor_speed(1, 'forward', 30)
        time.sleep(3)
        motor.stop_all()
        
        # 3. 차동 구동
        print("\n3️⃣ 차동 구동")
        motor.differential_drive(50, 50)  # 전진
        time.sleep(2)
        motor.differential_drive(-50, -50)  # 후진
        time.sleep(2)
        motor.differential_drive(-30, 30)  # 좌회전
        time.sleep(2)
        motor.differential_drive(30, -30)  # 우회전
        time.sleep(2)
        motor.stop_all()
        
        print("✅ 예제 완료!")
        
    except KeyboardInterrupt:
        print("\n⚠️ 중단됨")
        motor.stop_all()
    except Exception as e:
        print(f"❌ 오류: {e}")
        motor.stop_all()

def interactive_control():
    """대화형 모터 제어"""
    print("🎮 대화형 모터 제어")
    print("명령어:")
    print("  a: 모터 A 전진")
    print("  b: 모터 B 전진")
    print("  s: 정지")
    print("  f: 전진")
    print("  r: 후진")
    print("  l: 좌회전")
    print("  t: 우회전")
    print("  q: 종료")
    
    motor = MotorDriverHAT(debug=False)
    
    try:
        while True:
            cmd = input("\n명령어 입력: ").lower()
            
            if cmd == 'q':
                break
            elif cmd == 'a':
                motor.set_motor_speed(0, 'forward', 50)
            elif cmd == 'b':
                motor.set_motor_speed(1, 'forward', 50)
            elif cmd == 's':
                motor.stop_all()
            elif cmd == 'f':
                motor.differential_drive(50, 50)
            elif cmd == 'r':
                motor.differential_drive(-50, -50)
            elif cmd == 'l':
                motor.differential_drive(-30, 30)
            elif cmd == 't':
                motor.differential_drive(30, -30)
            else:
                print("❌ 잘못된 명령어")
                
    except KeyboardInterrupt:
        print("\n⚠️ 종료")
    finally:
        motor.stop_all()

if __name__ == "__main__":
    print("=" * 60)
    print("🎯 Jetson Nano Motor Driver HAT 예제")
    print("=" * 60)
    
    choice = input("예제 선택 (1: 간단한 제어, 2: 대화형 제어): ")
    
    if choice == "1":
        simple_motor_control()
    elif choice == "2":
        interactive_control()
    else:
        print("❌ 잘못된 선택") 