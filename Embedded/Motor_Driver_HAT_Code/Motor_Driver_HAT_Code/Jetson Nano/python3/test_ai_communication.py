#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
AI 통신 테스트 스크립트
"""

import json
import time
from motor_control_enhanced import MotorDriverHAT

def test_ai_data_processing():
    """AI 데이터 처리 테스트"""
    print("=" * 60)
    print("AI 데이터 처리 테스트")
    print("=" * 60)
    
    # 모터 드라이버 초기화 (API URL 없이)
    motor = MotorDriverHAT(debug=True, api_url=None)
    
    # 테스트용 AI 데이터
    test_cases = [
        {
            "serial": "AMR001",
            "x": "0.5",
            "y": "0.3",
            "img": "base64_encoded_image_data",
            "case": "forward",
            "timeStamp": "2024-01-01T12:00:00Z"
        },
        {
            "serial": "AMR001",
            "x": "-0.2",
            "y": "0.8",
            "img": "base64_encoded_image_data",
            "case": "left",
            "timeStamp": "2024-01-01T12:01:00Z"
        },
        {
            "serial": "AMR001",
            "x": "0.0",
            "y": "0.0",
            "img": "base64_encoded_image_data",
            "case": "stop",
            "timeStamp": "2024-01-01T12:02:00Z"
        },
        {
            "serial": "AMR001",
            "x": "0.7",
            "y": "0.4",
            "img": "base64_encoded_image_data",
            "case": "custom",
            "timeStamp": "2024-01-01T12:03:00Z"
        }
    ]
    
    try:
        for i, test_data in enumerate(test_cases, 1):
            print(f"\n테스트 케이스 {i}:")
            print(f"입력 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
            
            # AI 명령 처리
            motor.process_ai_command(test_data)
            
            # 잠시 대기
            time.sleep(2)
            
        print("\n모든 테스트 완료!")
        
    except Exception as e:
        print(f"테스트 중 오류 발생: {e}")
    finally:
        motor.stop_all()

def test_serial_number_mismatch():
    """시리얼 번호 불일치 테스트"""
    print("\n" + "=" * 60)
    print("시리얼 번호 불일치 테스트")
    print("=" * 60)
    
    motor = MotorDriverHAT(debug=True, api_url=None)
    motor.set_serial_number("AMR001")
    
    # 다른 시리얼 번호로 테스트
    test_data = {
        "serial": "AMR002",  # 다른 시리얼 번호
        "x": "0.5",
        "y": "0.3",
        "img": "base64_encoded_image_data",
        "case": "forward",
        "timeStamp": "2024-01-01T12:00:00Z"
    }
    
    print(f"테스트 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
    motor.process_ai_command(test_data)
    
    motor.stop_all()

def test_invalid_data():
    """잘못된 데이터 처리 테스트"""
    print("\n" + "=" * 60)
    print("잘못된 데이터 처리 테스트")
    print("=" * 60)
    
    motor = MotorDriverHAT(debug=True, api_url=None)
    
    # 잘못된 데이터들
    invalid_cases = [
        None,  # None 데이터
        {},    # 빈 딕셔너리
        {
            "serial": "AMR001",
            "x": "invalid",  # 잘못된 X 좌표
            "y": "0.3",
            "case": "unknown_case",  # 알 수 없는 케이스
            "timeStamp": "2024-01-01T12:00:00Z"
        }
    ]
    
    try:
        for i, test_data in enumerate(invalid_cases, 1):
            print(f"\n잘못된 데이터 테스트 {i}:")
            if test_data:
                print(f"입력 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
            else:
                print("입력 데이터: None")
                
            motor.process_ai_command(test_data)
            time.sleep(1)
            
    except Exception as e:
        print(f"테스트 중 오류 발생: {e}")
    finally:
        motor.stop_all()

def main():
    """메인 함수"""
    print("AI 통신 테스트 시작")
    
    try:
        # AI 데이터 처리 테스트
        test_ai_data_processing()
        
        # 시리얼 번호 불일치 테스트
        test_serial_number_mismatch()
        
        # 잘못된 데이터 처리 테스트
        test_invalid_data()
        
        print("\n모든 테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단됨")
    except Exception as e:
        print(f"\n오류 발생: {e}")

if __name__ == "__main__":
    main()
