#!/usr/bin/env python3
"""
send_control.py와 imu_ai_motor_controller.py 연동 테스트
"""

import requests
import time
import json

def test_server_connection():
    """서버 연결 테스트"""
    try:
        response = requests.get("http://localhost:5001/status", timeout=3)
        if response.status_code == 200:
            data = response.json()
            print("✅ AI 명령 서버 연결 성공!")
            print(f"   현재 명령: {data['current_command']['code']} - {data['current_command']['name']}")
            return True
        else:
            print(f"❌ 서버 응답 오류: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 서버 연결 실패: {e}")
        return False

def test_command_api():
    """명령 API 테스트"""
    print("\n🧪 명령 API 테스트 시작...")
    
    test_commands = [
        (0, "STOP"),
        (1, "MOVING_FORWARD"), 
        (2, "MOVING_BACKWARD"),
        (3, "ROTATE_LEFT"),
        (4, "ROTATE_RIGHT"),
        (5, "SERVO_UP"),
        (6, "SERVO_DOWN")
    ]
    
    for code, expected_name in test_commands:
        try:
            # POST로 명령 설정
            post_response = requests.post(
                "http://localhost:5001/command",
                json={'code': code},
                timeout=3
            )
            
            if post_response.status_code == 200:
                print(f"✅ POST {code} ({expected_name}) 설정 성공")
                
                # GET으로 명령 확인 (imu_ai_motor_controller.py가 하는 것처럼)
                time.sleep(0.1)
                get_response = requests.get("http://localhost:5001/command", timeout=3)
                
                if get_response.status_code == 200:
                    data = get_response.json()
                    if data['code'] == code and data['name'] == expected_name:
                        print(f"✅ GET {code} ({expected_name}) 확인 성공")
                    else:
                        print(f"❌ GET 응답 불일치: 예상={code}, 실제={data}")
                else:
                    print(f"❌ GET 요청 실패: {get_response.status_code}")
            else:
                print(f"❌ POST {code} 실패: {post_response.status_code}")
                
        except Exception as e:
            print(f"❌ 명령 {code} 테스트 실패: {e}")
        
        time.sleep(0.2)

def main():
    print("=" * 60)
    print("send_control.py ↔ imu_ai_motor_controller.py 연동 테스트")
    print("=" * 60)
    
    print("\n📡 1단계: 서버 연결 확인")
    if not test_server_connection():
        print("\n❌ send_control.py를 먼저 실행하세요:")
        print("   python3 send_control.py")
        print("   선택: 1 (서버만 실행)")
        return
    
    print("\n📡 2단계: 명령 API 테스트")
    test_command_api()
    
    print("\n✅ 연동 테스트 완료!")
    print("\n🚀 이제 다음 명령으로 로봇을 실행하세요:")
    print("   python3 imu_ai_motor_controller.py")
    print("   선택: 3 (AI 제어 루프 실행)")

if __name__ == "__main__":
    main()
