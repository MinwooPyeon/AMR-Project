#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI 파일 수신 테스트 스크립트
파일에서 AI 데이터를 읽는 테스트
"""

import json
import os
import time
import sys

def get_ai_data_from_file():
    """파일에서 AI 데이터 읽기"""
    try:
        with open("/tmp/ai_data.json", "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except json.JSONDecodeError:
        print("JSON 파싱 오류")
        return None
    except Exception as e:
        print(f"파일 읽기 실패: {e}")
        return None

def create_sample_ai_data():
    """샘플 AI 데이터 파일 생성 (테스트용)"""
    sample_data = {
        "serial": "AMR001",
        "x": 10.5,
        "y": 20.3,
        "img": "base64_encoded_image_data",
        "case": "obstacle_detected",
        "timeStamp": "2025-08-06T15:57:00Z"
    }
    
    try:
        with open("/tmp/ai_data.json", "w") as f:
            json.dump(sample_data, f, indent=2)
        print("✅ 샘플 AI 데이터 파일 생성 완료")
    except Exception as e:
        print(f"❌ 샘플 파일 생성 실패: {e}")

def main():
    """메인 함수"""
    print("=== AI 파일 수신 테스트 ===")
    print("/tmp/ai_data.json에서 AI 데이터를 읽습니다.")
    print("종료하려면 Ctrl+C를 누르세요.\n")
    
    # 샘플 데이터 파일 생성 (테스트용)
    create_sample_ai_data()
    
    try:
        while True:
            ai_data = get_ai_data_from_file()
            
            if ai_data:
                print(f"\n📥 AI 데이터 수신:")
                print(f"   시리얼: {ai_data.get('serial', 'N/A')}")
                print(f"   위치: ({ai_data.get('x', 0)}, {ai_data.get('y', 0)})")
                print(f"   이미지: {'있음' if ai_data.get('img') else '없음'}")
                print(f"   케이스: {ai_data.get('case', 'N/A')}")
                print(f"   타임스탬프: {ai_data.get('timeStamp', 'N/A')}")
            else:
                print("⏳ AI 데이터 파일 대기 중...")
            
            time.sleep(2)
            
    except KeyboardInterrupt:
        print("\n\n⚠️  프로그램 종료 중...")

if __name__ == "__main__":
    main() 