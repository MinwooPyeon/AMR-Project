#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
테스트 실행 스크립트
"""

import sys
import os

# 프로젝트 루트를 Python 경로에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

def run_ai_communication_test():
    """AI 통신 테스트 실행"""
    print("AI 통신 테스트를 실행합니다...")
    try:
        from tests.ai_communication_test import main
        main()
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

def run_imu_sensor_test():
    """IMU 센서 테스트 실행"""
    print("IMU 센서 테스트를 실행합니다...")
    try:
        from tests.imu_sensor_test import main
        main()
    except Exception as e:
        print(f"테스트 실행 중 오류 발생: {e}")

def run_mqtt_test():
    """MQTT 테스트 실행"""
    print("MQTT 테스트를 실행합니다...")
    try:
        from tests.mqtt_test import main
        main()
    except Exception as e:
        print(f"테스트 실행 중 오류 발생: {e}")

def run_obstacle_detection_test():
    """장애물 감지 테스트 실행"""
    print("장애물 감지 테스트를 실행합니다...")
    try:
        from tests.obstacle_detection_test import main
        main()
    except Exception as e:
        print(f"테스트 실행 중 오류 발생: {e}")

def run_rotation_detection_test():
    """회전 감지 테스트 실행"""
    print("회전 감지 테스트를 실행합니다...")
    try:
        from tests.rotation_detection_test import main
        main()
    except Exception as e:
        print(f"테스트 실행 중 오류 발생: {e}")

def run_all_tests():
    """모든 테스트 실행"""
    print("모든 테스트를 실행합니다...")
    tests = [
        run_ai_communication_test,
        run_angle_control_test,
        run_imu_sensor_test,
        run_mqtt_test,
        run_obstacle_detection_test,
        run_rotation_detection_test
    ]
    
    for test in tests:
        try:
            test()
            print("-" * 50)
        except Exception as e:
            print(f"테스트 실행 중 오류 발생: {e}")
            print("-" * 50)

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="테스트 실행 스크립트")
    parser.add_argument("--test", choices=["ai", "angle", "imu", "mqtt", "obstacle", "rotation", "all"], 
                       help="실행할 테스트 선택")
    
    args = parser.parse_args()
    
    if args.test == "ai":
        run_ai_communication_test()
    elif args.test == "angle":
        run_angle_control_test()
    elif args.test == "imu":
        run_imu_sensor_test()
    elif args.test == "mqtt":
        run_mqtt_test()
    elif args.test == "obstacle":
        run_obstacle_detection_test()
    elif args.test == "rotation":
        run_rotation_detection_test()
    elif args.test == "all":
        run_all_tests()
    else:
        print("사용법:")
        print("  python run_tests.py --test ai        # AI 통신 테스트")
        print("  python run_tests.py --test angle     # 각도 제어 테스트")
        print("  python run_tests.py --test imu       # IMU 센서 테스트")
        print("  python run_tests.py --test mqtt      # MQTT 테스트")
        print("  python run_tests.py --test obstacle  # 장애물 감지 테스트")
        print("  python run_tests.py --test rotation  # 회전 감지 테스트")
        print("  python run_tests.py --test all       # 모든 테스트") 