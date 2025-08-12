#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
AI 클라이언트 (수정된 버전)
로봇 서버와 연결하여 명령을 전송합니다.
"""

import sys
import requests
from flask import Flask, jsonify, request, abort

app = Flask(__name__)
API_URL = "http://127.0.0.1:5001/command"  # 로봇 서버 주소

COMMAND_TABLE = {
    0: 'STOP',
    1: 'MOVING_FORWARD',
    2: 'MOVING_BACKWARD',
    3: 'ROTATE_LEFT',
    4: 'ROTATE_RIGHT',
}

@app.route('/command', methods=['GET'])
def get_command():
    # URL 쿼리 파라미터에서 code 값 읽기
    code = request.args.get('code', type=int)

    # 유효성 검사
    if code not in [0, 1, 2, 3, 4]:
        return jsonify({"status": "error", "message": "Invalid command code"}), 400
    name = COMMAND_TABLE.get(code, 'UNKNOWN')
    
    # 실제 처리 로직 (예: 로봇 제어)
    # 여기서는 예시로 단순 응답
    return jsonify({
        'code': code,
        'name': name
    })

def test_server_connection():
    """서버 연결 테스트"""
    try:
        print("🔍 서버 연결 테스트 중...")
        response = requests.get("http://127.0.0.1:5001/status", timeout=5)
        if response.status_code == 200:
            print("✅ 서버 연결 성공!")
            return True
        else:
            print(f"❌ 서버 응답 오류: {response.status_code}")
            return False
    except Exception as e:
        print(f"❌ 서버 연결 실패: {e}")
        return False

def main():
    print("🤖 AI 로봇 제어 클라이언트")
    print("=" * 50)
    
    # 서버 연결 확인
    if not test_server_connection():
        print("💡 로봇 서버가 실행 중인지 확인하세요: python3 simple_motor_test.py")
        return
    
    print("📋 사용 가능한 명령:")
    for code, name in COMMAND_TABLE.items():
        print(f"  {code}: {name}")
    print("  q: 종료")
    print("=" * 50)
    
    while True:
        try:
            code_str = input("\n명령 코드 입력 (0=STOP,1=FORWARD,2=BACKWARD,3=LEFT,4=RIGHT,q=종료): ").strip()
            
            if code_str.lower() == 'q':
                print("👋 프로그램을 종료합니다.")
                break
                
            code = int(code_str)
            
            if code not in [0, 1, 2, 3, 4]:
                print("❌ 잘못된 코드입니다. 0-4 사이의 숫자를 입력하세요.")
                continue
            
            print(f"🚀 명령 전송 중: {code} ({COMMAND_TABLE[code]})")
            
            # timeout을 10초로 늘림 (모터 제어 시간 고려)
            r = requests.get(API_URL, params={"code": code}, timeout=10)
            
            if r.status_code == 200:
                response_data = r.json()
                print("✅ 서버 응답:", response_data)
            else:
                print(f"❌ 서버 오류: {r.status_code}")
                
        except ValueError:
            print("❌ 숫자만 입력하세요.")
        except KeyboardInterrupt:
            print("\n👋 프로그램을 종료합니다.")
            break
        except requests.exceptions.Timeout:
            print("⚠️ 요청 시간 초과 (10초) - 모터 제어가 지연되고 있습니다.")
        except requests.exceptions.ConnectionError:
            print("❌ 서버에 연결할 수 없습니다. 서버가 실행 중인지 확인하세요.")
        except Exception as e:
            print(f"⚠️ 요청 실패: {e}")

if __name__ == "__main__":
    main()
