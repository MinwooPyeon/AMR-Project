#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI HTTP 수신 테스트 스크립트
HTTP API로 AI 데이터를 가져오는 테스트
"""

import requests
import json
import time
import sys

def get_ai_data():
    """HTTP로 AI 데이터 가져오기"""
    try:
        response = requests.get("http://localhost:8080/ai_data", timeout=5)
        if response.status_code == 200:
            return response.json()
        else:
            print(f"HTTP 오류: {response.status_code}")
            return None
    except requests.exceptions.ConnectionError:
        print("AI 서버에 연결할 수 없습니다.")
        return None
    except Exception as e:
        print(f"AI 데이터 가져오기 실패: {e}")
        return None

def main():
    """메인 함수"""
    print("=== AI HTTP 수신 테스트 ===")
    print("http://localhost:8080/ai_data에서 AI 데이터를 가져옵니다.")
    print("종료하려면 Ctrl+C를 누르세요.\n")
    
    try:
        while True:
            ai_data = get_ai_data()
            
            if ai_data:
                print(f"\n📥 AI 데이터 수신:")
                print(f"   시리얼: {ai_data.get('serial', 'N/A')}")
                print(f"   위치: ({ai_data.get('x', 0)}, {ai_data.get('y', 0)})")
                print(f"   이미지: {'있음' if ai_data.get('img') else '없음'}")
                print(f"   케이스: {ai_data.get('case', 'N/A')}")
                print(f"   타임스탬프: {ai_data.get('timeStamp', 'N/A')}")
            else:
                print("⏳ AI 데이터 대기 중...")
            
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\n\n⚠️  프로그램 종료 중...")

if __name__ == "__main__":
    main() 