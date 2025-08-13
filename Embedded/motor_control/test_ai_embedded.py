#!/usr/bin/env python3
"""
AI 서버 ↔ 임베디드 장치 연동 테스트
"""

import requests
import time
import threading
import subprocess
import sys

def test_ai_server():
    """AI 서버 연결 테스트"""
    try:
        print("🔍 AI 서버 연결 테스트...")
        response = requests.get("http://localhost:5001/status", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ AI 서버 연결 성공!")
            print(f"   서버 타입: {data.get('server_type', 'UNKNOWN')}")
            print(f"   현재 명령: {data['current_command']['code']} - {data['current_command']['name']}")
            return True
        else:
            print(f"❌ AI 서버 응답 오류: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ AI 서버 연결 실패: {e}")
        return False

def send_test_commands():
    """테스트 명령 자동 전송"""
    test_commands = [
        (1, "MOVING_FORWARD", 3),
        (0, "STOP", 2), 
        (2, "MOVING_BACKWARD", 3),
        (0, "STOP", 2),
        (3, "ROTATE_LEFT", 2),
        (0, "STOP", 2),
        (4, "ROTATE_RIGHT", 2),
        (0, "STOP", 1)
    ]
    
    print("\n🚀 자동 테스트 명령 시퀀스 시작...")
    
    for code, name, duration in test_commands:
        try:
            print(f"\n📤 명령 전송: {code} - {name} ({duration}초 유지)")
            
            response = requests.post(
                "http://localhost:5001/ai_command",
                json={'code': code},
                headers={'Content-Type': 'application/json'},
                timeout=5
            )
            
            if response.status_code == 200:
                print(f"✅ 명령 전송 성공")
                time.sleep(duration)
            else:
                print(f"❌ 명령 전송 실패: {response.status_code}")
                
        except Exception as e:
            print(f"❌ 명령 전송 오류: {e}")
            break
    
    print("\n🏁 자동 테스트 완료!")

def monitor_embedded_requests():
    """임베디드 장치의 요청 모니터링"""
    print("📡 임베디드 장치 요청 모니터링 중...")
    print("   (AI 서버 로그를 확인하여 GET /command 요청을 관찰)")

def main():
    print("🤖 AI 서버 ↔ 임베디드 장치 연동 테스트")
    print("=" * 60)
    
    # AI 서버 연결 확인
    if not test_ai_server():
        print("\n💡 먼저 AI 서버를 실행하세요:")
        print("   python3 ai_client.py")
        print("   선택: 1 (서버만 실행) 또는 2 (서버 + 수동 제어)")
        return
    
    print("\n📋 테스트 옵션:")
    print("1. 자동 테스트 (명령 시퀀스 자동 전송)")
    print("2. 수동 테스트 (직접 명령 입력)")
    print("3. 모니터링만 (임베디드 요청 관찰)")
    
    choice = input("\n선택 (1-3): ").strip()
    
    if choice == "1":
        print("\n⚠️  임베디드 장치가 실행 중인지 확인하세요:")
        print("   python3 imu_ai_motor_controller.py")
        print("   선택: 3 (AI 제어 루프 실행)")
        
        input("\n임베디드 장치 준비 완료 후 Enter를 누르세요...")
        send_test_commands()
        
    elif choice == "2":
        print("\n💡 수동 테스트를 위해 다른 터미널에서 실행하세요:")
        print("   python3 ai_client.py")
        print("   선택: 2 (서버 + 수동 제어)")
        
    elif choice == "3":
        monitor_embedded_requests()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n모니터링 종료")
    else:
        print("❌ 잘못된 선택")

if __name__ == "__main__":
    main()
