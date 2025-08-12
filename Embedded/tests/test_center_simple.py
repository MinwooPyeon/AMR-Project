#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
정중앙 맞추기 간단 테스트
"""

import time
import sys
import os

# 현재 디렉토리에서 IMU AI 모터 컨트롤러 임포트
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from imu_ai_motor_controller import IMUAIMotorController

def main():
    print("=" * 50)
    print("정중앙 맞추기 테스트")
    print("=" * 50)
    
    try:
        # 컨트롤러 초기화
        print("IMU AI 모터 컨트롤러 초기화 중...")
        controller = IMUAIMotorController(
            debug=True,
            api_url=None,
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        controller.set_serial_number("AMR001")
        print("초기화 완료!")
        
        # 초기 상태 확인
        print("\n1. 초기 상태 확인")
        status = controller.get_center_status()
        print(f"   현재 각도: {status['current_angle']:.2f}도")
        print(f"   정중앙 여부: {status['is_centered']}")
        print(f"   각도 오프셋: {status['angle_offset']:.2f}도")
        
        # 사용자 입력 대기
        input("\n엔터를 누르면 정중앙 맞추기를 시작합니다...")
        
        # 정중앙 맞추기
        print("\n2. 정중앙 맞추기 실행")
        if controller.center_robot():
            print("   정중앙 맞추기 시작됨")
            
            # 맞추기 완료 대기
            start_time = time.time()
            while controller.is_turning_now() and time.time() - start_time < 30:
                time.sleep(0.1)
                current_angle = controller.get_current_angle()
                print(f"   현재 각도: {current_angle:.2f}도", end='\r')
            
            print()  # 줄바꿈
            
            if controller.is_turning_now():
                print("   시간 초과로 정지")
                controller.stop_all()
                controller.is_turning = False
            else:
                print("   정중앙 맞추기 완료!")
        else:
            print("   정중앙 맞추기 실패 또는 이미 중앙에 있음")
        
        # 최종 상태 확인
        print("\n3. 최종 상태 확인")
        final_status = controller.get_center_status()
        print(f"   현재 각도: {final_status['current_angle']:.2f}도")
        print(f"   정중앙 여부: {final_status['is_centered']}")
        
        if final_status['is_centered']:
            print("   ✅ 정중앙 맞추기 성공!")
        else:
            print("   ❌ 정중앙 맞추기 실패")
        
        # 정리
        controller.stop_all()
        controller.stop_control_loop()
        
        print("\n테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단")
        try:
            controller.stop_all()
            controller.stop_control_loop()
        except:
            pass
    except Exception as e:
        print(f"오류 발생: {e}")
        try:
            controller.stop_all()
            controller.stop_control_loop()
        except:
            pass

if __name__ == "__main__":
    main()

