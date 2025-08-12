#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
테스트 실행 스크립트
"""

import sys
import os

# 프로젝트 루트를 Python 경로에 추가
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

def run_test_connection_setup():
    """연결 설정 테스트 실행"""
    print("연결 설정 테스트를 실행합니다...")
    try:
        from tests.test_connection_setup import test_embedded_to_backend, test_ai_to_embedded, test_connection_summary
        test_embedded_to_backend()
        test_ai_to_embedded()
        test_connection_summary()
    except Exception as e:
        print(f"테스트 실행 중 오류 발생: {e}")

def run_angle_control_test():
    """각도 제어 테스트 실행"""
    print("각도 제어 테스트를 실행합니다...")
    try:
        from tests.angle_control_test import main
        main()
    except Exception as e:
        print(f"테스트 실행 중 오류 발생: {e}")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="테스트 실행 스크립트")
    parser.add_argument("--test", choices=["connection", "angle"], 
                       help="실행할 테스트 선택 (connection 또는 angle)")
    
    args = parser.parse_args()
    
    if args.test == "connection":
        run_test_connection_setup()
    elif args.test == "angle":
        run_angle_control_test()
    else:
        print("사용법:")
        print("  python run_tests.py --test connection  # 연결 설정 테스트")
        print("  python run_tests.py --test angle      # 각도 제어 테스트") 