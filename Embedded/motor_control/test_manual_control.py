#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
키보드 수동 제어 테스트 스크립트
"""

import sys
import os
import time

# 프로젝트 루트 경로 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from manual_keyboard_controller import ManualKeyboardController

def test_manual_control():
    """키보드 수동 제어 테스트"""
    print("키보드 수동 제어 테스트 시작")
    print("=" * 50)
    
    controller = None
    
    try:
        # 컨트롤러 초기화
        controller = ManualKeyboardController(debug=True)
        
        print("컨트롤러 초기화 완료")
        print("키보드 조작법:")
        print("  W - 전진")
        print("  S - 후진") 
        print("  A - 좌회전")
        print("  D - 우회전")
        print("  SPACE - 정지")
        print("  1-4 - 속도 조절")
        print("  ESC - 종료")
        print("=" * 50)
        
        # 간단한 테스트 동작
        print("기본 동작 테스트 중...")
        
        # 전진 테스트 (1초)
        print("전진 테스트 (1초)")
        controller.differential_drive(30, 30)
        time.sleep(1)
        controller.stop_all()
        
        time.sleep(0.5)
        
        # 후진 테스트 (1초)
        print("후진 테스트 (1초)")
        controller.differential_drive(-30, -30)
        time.sleep(1)
        controller.stop_all()
        
        time.sleep(0.5)
        
        # 좌회전 테스트 (1초)
        print("좌회전 테스트 (1초)")
        controller.differential_drive(-30, 30)
        time.sleep(1)
        controller.stop_all()
        
        time.sleep(0.5)
        
        # 우회전 테스트 (1초)
        print("우회전 테스트 (1초)")
        controller.differential_drive(30, -30)
        time.sleep(1)
        controller.stop_all()
        
        print("기본 동작 테스트 완료!")
        print("이제 키보드로 수동 제어를 시작합니다.")
        print("=" * 50)
        
        # 키보드 수동 제어 시작
        controller.run_manual_control()
        
    except KeyboardInterrupt:
        print("\n테스트 중단")
    except Exception as e:
        print(f"테스트 오류: {e}")
    finally:
        if controller:
            controller.stop_all()
            controller.stop_control_loop()
        print("테스트 종료")

if __name__ == "__main__":
    test_manual_control()
