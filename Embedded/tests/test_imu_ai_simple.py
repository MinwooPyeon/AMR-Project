#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU AI 모터 컨트롤러 간단 테스트
"""

import time
import sys
import os

# 현재 디렉토리에서 IMU AI 모터 컨트롤러 임포트
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from imu_ai_motor_controller import IMUAIMotorController

def test_basic_movements():
    """기본 동작 테스트"""
    print("=== 기본 동작 테스트 ===")
    
    try:
        # 컨트롤러 초기화 (AI API 없이)
        controller = IMUAIMotorController(
            debug=True,
            api_url=None,  # AI API 비활성화
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        controller.set_serial_number("AMR001")
        
        print("1. 전진 테스트 (3초)")
        controller.differential_drive(30, 30)
        time.sleep(3)
        controller.stop_all()
        
        print("2. 후진 테스트 (3초)")
        controller.differential_drive(-30, -30)
        time.sleep(3)
        controller.stop_all()
        
        print("3. 좌회전 테스트 (3초)")
        controller.differential_drive(-30, 30)
        time.sleep(3)
        controller.stop_all()
        
        print("4. 우회전 테스트 (3초)")
        controller.differential_drive(30, -30)
        time.sleep(3)
        controller.stop_all()
        
        print("기본 동작 테스트 완료!")
        
    except Exception as e:
        print(f"기본 동작 테스트 오류: {e}")

def test_imu_control():
    """IMU 제어 테스트"""
    print("\n=== IMU 제어 테스트 ===")
    
    try:
        # 컨트롤러 초기화
        controller = IMUAIMotorController(
            debug=True,
            api_url=None,
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        controller.set_serial_number("AMR001")
        
        # IMU 모터 통합 테스트 실행
        controller.test_imu_motors()
        
        print("IMU 제어 테스트 완료!")
        
    except Exception as e:
        print(f"IMU 제어 테스트 오류: {e}")

def test_ai_integration():
    """AI 통합 테스트"""
    print("\n=== AI 통합 테스트 ===")
    
    try:
        # 컨트롤러 초기화 (AI API 포함)
        controller = IMUAIMotorController(
            debug=True,
            api_url="http://localhost:5001/command",  # AI 명령 API 활성화
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        controller.set_serial_number("AMR001")
        
        if controller.connect_backend():
            print("Backend 연결 성공")
        else:
            print("Backend 연결 실패")
        
        # AI 제어 루프 실행 (10초만)
        print("AI 제어 루프 시작 (10초)")
        controller.start_control_loop()
        
        start_time = time.time()
        while time.time() - start_time < 10:
            ai_data = controller.get_ai_data()
            if ai_data:
                controller.process_ai_command(ai_data)
            time.sleep(1)
        
        controller.stop_control_loop()
        controller.stop_all()
        controller.disconnect_backend()
        
        print("AI 통합 테스트 완료!")
        
    except Exception as e:
        print(f"AI 통합 테스트 오류: {e}")

def test_center_alignment():
    """정중앙 맞추기 테스트"""
    print("\n=== 정중앙 맞추기 테스트 ===")
    
    try:
        # 컨트롤러 초기화
        controller = IMUAIMotorController(
            debug=True,
            api_url=None,
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        controller.set_serial_number("AMR001")
        
        # 초기 상태 확인
        print("1. 초기 정중앙 상태 확인")
        status = controller.get_center_status()
        print(f"   현재 각도: {status['current_angle']:.2f}도")
        print(f"   정중앙 여부: {status['is_centered']}")
        print(f"   각도 오프셋: {status['angle_offset']:.2f}도")
        
        # 정중앙 맞추기
        print("\n2. 정중앙 맞추기 실행")
        if controller.center_robot():
            print("   정중앙 맞추기 시작됨")
            
            # 맞추기 완료 대기
            while controller.is_turning_now():
                time.sleep(0.1)
                current_angle = controller.get_current_angle()
                print(f"   현재 각도: {current_angle:.2f}도")
            
            print("   정중앙 맞추기 완료!")
        else:
            print("   정중앙 맞추기 실패 또는 이미 중앙에 있음")
        
        # 최종 상태 확인
        print("\n3. 최종 정중앙 상태 확인")
        final_status = controller.get_center_status()
        print(f"   현재 각도: {final_status['current_angle']:.2f}도")
        print(f"   정중앙 여부: {final_status['is_centered']}")
        
        print("정중앙 맞추기 테스트 완료!")
        
    except Exception as e:
        print(f"정중앙 맞추기 테스트 오류: {e}")

def test_pid_tuning():
    """PID 튜닝 테스트"""
    print("\n=== PID 튜닝 테스트 ===")
    
    try:
        # 컨트롤러 초기화
        controller = IMUAIMotorController(
            debug=True,
            api_url=None,
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        controller.set_serial_number("AMR001")
        
        # 다양한 PID 게인 테스트
        pid_configs = [
            (1.0, 0.05, 0.3),   # 낮은 게인
            (2.0, 0.1, 0.5),    # 기본 게인
            (3.0, 0.15, 0.7),   # 높은 게인
        ]
        
        for kp, ki, kd in pid_configs:
            print(f"\nPID 게인 테스트: Kp={kp}, Ki={ki}, Kd={kd}")
            controller.set_pid_gains(kp, ki, kd)
            
            # 90도 좌회전 테스트
            if controller.turn_left_90():
                start_time = time.time()
                while controller.is_turning and time.time() - start_time < 10:
                    print(f"회전 중... 현재 각도: {controller.get_current_angle():.2f}도")
                    time.sleep(0.1)
                
                if controller.is_turning:
                    print("시간 초과로 정지")
                    controller.stop_all()
                    controller.is_turning = False
                else:
                    print("회전 완료!")
            
            time.sleep(2)
        
        print("PID 튜닝 테스트 완료!")
        
    except Exception as e:
        print(f"PID 튜닝 테스트 오류: {e}")

def main():
    print("=" * 60)
    print("IMU AI 모터 컨트롤러 테스트 스위트")
    print("=" * 60)
    
    print("\n테스트 옵션:")
    print("1. 기본 동작 테스트")
    print("2. IMU 제어 테스트")
    print("3. AI 통합 테스트")
    print("4. 정중앙 맞추기 테스트")
    print("5. PID 튜닝 테스트")
    print("6. 전체 테스트")
    print("0. 종료")
    
    while True:
        try:
            choice = input("\n테스트를 선택하세요 (0-6): ").strip()
            
            if choice == "0":
                print("테스트 종료")
                break
            elif choice == "1":
                test_basic_movements()
            elif choice == "2":
                test_imu_control()
            elif choice == "3":
                test_ai_integration()
            elif choice == "4":
                test_center_alignment()
            elif choice == "5":
                test_pid_tuning()
            elif choice == "6":
                print("\n전체 테스트 시작...")
                test_basic_movements()
                test_imu_control()
                test_center_alignment()
                test_pid_tuning()
                test_ai_integration()
                print("\n전체 테스트 완료!")
            else:
                print("잘못된 선택입니다. 0-6 중에서 선택해주세요.")
                
        except KeyboardInterrupt:
            print("\n사용자에 의해 중단")
            break
        except Exception as e:
            print(f"오류 발생: {e}")

if __name__ == "__main__":
    main()
