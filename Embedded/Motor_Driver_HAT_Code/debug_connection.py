#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
연결 문제 진단 스크립트
AI에서 발생하는 ConnectTimeoutError 원인을 찾습니다.
"""

import socket
import requests
import subprocess
import time

def check_port_listening():
    """포트 5001이 실제로 리스닝 중인지 확인"""
    print("🔍 포트 5001 리스닝 상태 확인...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex(('127.0.0.1', 5001))
        sock.close()
        
        if result == 0:
            print("✅ 포트 5001이 리스닝 중입니다")
            return True
        else:
            print("❌ 포트 5001이 리스닝되지 않습니다")
            return False
    except Exception as e:
        print(f"❌ 포트 확인 오류: {e}")
        return False

def check_server_process():
    """서버 프로세스가 실행 중인지 확인"""
    print("🔍 서버 프로세스 확인...")
    try:
        result = subprocess.run(['pgrep', '-f', 'simple_motor_test.py'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            pids = result.stdout.strip().split('\n')
            print(f"✅ 서버 프로세스 실행 중 (PID: {pids})")
            return True
        else:
            print("❌ 서버 프로세스가 실행되지 않습니다")
            return False
    except Exception as e:
        print(f"❌ 프로세스 확인 오류: {e}")
        return False

def test_connection_with_timeout(timeout):
    """지정된 timeout으로 연결 테스트"""
    print(f"🔍 {timeout}초 timeout으로 연결 테스트...")
    try:
        start_time = time.time()
        response = requests.get("http://127.0.0.1:5001/status", timeout=timeout)
        end_time = time.time()
        
        if response.status_code == 200:
            print(f"✅ 연결 성공! 응답 시간: {end_time - start_time:.2f}초")
            return True
        else:
            print(f"❌ 서버 응답 오류: {response.status_code}")
            return False
    except requests.exceptions.ConnectTimeout:
        print(f"❌ 연결 시간 초과 ({timeout}초)")
        return False
    except requests.exceptions.ConnectionError:
        print(f"❌ 연결 오류")
        return False
    except Exception as e:
        print(f"❌ 기타 오류: {e}")
        return False

def check_network_interface():
    """네트워크 인터페이스 확인"""
    print("🔍 네트워크 인터페이스 확인...")
    try:
        result = subprocess.run(['ip', 'addr', 'show', 'lo'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ localhost 인터페이스 정상")
            return True
        else:
            print("❌ localhost 인터페이스 문제")
            return False
    except Exception as e:
        print(f"❌ 네트워크 확인 오류: {e}")
        return False

def main():
    """메인 진단 함수"""
    print("🔧 AI-로봇 연결 문제 진단")
    print("=" * 50)
    
    # 1. 서버 프로세스 확인
    if not check_server_process():
        print("\n💡 해결 방법: python3 simple_motor_test.py로 서버를 실행하세요")
        return
    
    # 2. 포트 리스닝 확인
    if not check_port_listening():
        print("\n💡 해결 방법: 서버가 완전히 시작될 때까지 기다리세요")
        return
    
    # 3. 네트워크 인터페이스 확인
    if not check_network_interface():
        print("\n💡 해결 방법: 네트워크 설정을 확인하세요")
        return
    
    # 4. 다양한 timeout으로 테스트
    print("\n🎯 연결 테스트 시작...")
    timeouts = [1, 2, 5, 10]
    
    for timeout in timeouts:
        if test_connection_with_timeout(timeout):
            print(f"✅ {timeout}초 timeout으로 연결 성공!")
            break
        else:
            print(f"❌ {timeout}초 timeout으로 연결 실패")
    
    print("\n📋 진단 완료!")
    print("💡 만약 모든 timeout에서 실패한다면:")
    print("   1. 서버 재시작: Ctrl+C 후 python3 simple_motor_test.py")
    print("   2. 방화벽 확인: sudo ufw status")
    print("   3. 포트 충돌 확인: netstat -tlnp | grep 5001")

if __name__ == "__main__":
    main()
