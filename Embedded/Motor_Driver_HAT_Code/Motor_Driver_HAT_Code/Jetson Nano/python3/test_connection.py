#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
연결 테스트 스크립트
AI 클라이언트와 로봇 서버 간의 연결을 테스트합니다.
"""

import requests
import time

def test_server_connection():
    """서버 연결 테스트"""
    print("🔍 로봇 서버 연결 테스트 시작")
    print("=" * 50)
    
    # 1. 서버 상태 확인
    try:
        print("📡 서버 상태 확인 중...")
        response = requests.get("http://127.0.0.1:5001/status", timeout=5)
        if response.status_code == 200:
            print("✅ 서버 연결 성공!")
            data = response.json()
            print(f"📊 서버 정보: {data}")
            return True
        else:
            print(f"❌ 서버 응답 오류: {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("❌ 서버에 연결할 수 없습니다.")
        print("💡 로봇 서버가 실행 중인지 확인하세요: python3 simple_motor_test.py")
        return False
    except requests.exceptions.Timeout:
        print("❌ 서버 응답 시간 초과 (5초)")
        return False
    except Exception as e:
        print(f"❌ 연결 테스트 오류: {e}")
        return False

def test_command_endpoint():
    """명령 엔드포인트 테스트"""
    print("\n🎮 명령 엔드포인트 테스트")
    print("-" * 30)
    
    commands = [
        (0, "정지"),
        (1, "직진"),
        (2, "후진"),
        (3, "좌회전"),
        (4, "우회전")
    ]
    
    for code, name in commands:
        try:
            print(f"📤 명령 전송: {code} ({name})")
            response = requests.get(f"http://127.0.0.1:5001/command?code={code}", timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                print(f"✅ 응답: {data}")
            else:
                print(f"❌ 오류 응답: {response.status_code}")
                
            time.sleep(1)  # 명령 간 간격
            
        except requests.exceptions.Timeout:
            print(f"❌ 명령 {code} 시간 초과")
        except Exception as e:
            print(f"❌ 명령 {code} 오류: {e}")

def main():
    """메인 함수"""
    print("🤖 AI-로봇 연결 테스트")
    print("=" * 50)
    
    # 1. 서버 연결 테스트
    if not test_server_connection():
        print("\n💡 해결 방법:")
        print("   1. 로봇 서버 실행: python3 simple_motor_test.py")
        print("   2. 서버가 완전히 시작될 때까지 대기")
        print("   3. 포트 5001이 사용 가능한지 확인")
        return
    
    # 2. 명령 엔드포인트 테스트
    test_command_endpoint()
    
    print("\n✅ 모든 테스트 완료!")
    print("🎯 이제 AI 클라이언트에서 연결할 수 있습니다.")

if __name__ == "__main__":
    main()
