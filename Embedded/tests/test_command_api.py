#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
명령 API 테스트 스크립트
"""

import requests
import json
import time

def test_command_api():
    base_url = "http://localhost:5000"
    
    print("=== 명령 API 테스트 ===")
    
    # 1. 서버 상태 확인
    try:
        response = requests.get(f"{base_url}/status")
        print(f"서버 상태: {response.json()}")
    except Exception as e:
        print(f"서버 연결 실패: {e}")
        return
    
    # 2. 현재 명령 조회
    try:
        response = requests.get(f"{base_url}/command")
        current_command = response.json()
        print(f"현재 명령: {current_command}")
    except Exception as e:
        print(f"명령 조회 실패: {e}")
        return
    
    # 3. 각 명령 테스트
    commands = [
        (1, "MOVING_FORWARD"),
        (2, "MOVING_BACKWARD"),
        (3, "ROTATE_LEFT"),
        (4, "ROTATE_RIGHT"),
        (0, "STOP")
    ]
    
    for code, name in commands:
        print(f"\n--- {name} (코드: {code}) 테스트 ---")
        
        # 명령 설정
        try:
            response = requests.post(f"{base_url}/command", 
                                   json={"code": code},
                                   headers={"Content-Type": "application/json"})
            result = response.json()
            print(f"명령 설정 결과: {result}")
        except Exception as e:
            print(f"명령 설정 실패: {e}")
            continue
        
        # 2초 대기
        time.sleep(2)
        
        # 현재 명령 확인
        try:
            response = requests.get(f"{base_url}/command")
            current = response.json()
            print(f"현재 명령: {current}")
        except Exception as e:
            print(f"명령 확인 실패: {e}")
    
    print("\n=== 테스트 완료 ===")

if __name__ == "__main__":
    test_command_api()
