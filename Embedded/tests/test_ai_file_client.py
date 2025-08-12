#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
파일 기반 AI 데이터 수신 클라이언트 테스트
AI 시스템이 JSON 파일로 데이터를 전달하는 방식 테스트
"""

import sys
import os
import time
import json
import signal

# 프로젝트 루트를 Python 경로에 추가
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from ai.ai_file_client import AIFileClient

def ai_data_callback(data):
    """AI 데이터 수신 콜백"""
    print(f"\n📥 AI 데이터 수신:")
    print(f"   시리얼: {data.get('serial', 'N/A')}")
    print(f"   위치: ({data.get('x', 0)}, {data.get('y', 0)})")
    print(f"   이미지: {'있음' if data.get('img') else '없음'}")
    print(f"   케이스: {data.get('case', 'N/A')}")
    print(f"   타임스탬프: {data.get('timeStamp', 'N/A')}")

def create_test_data(file_path: str):
    """테스트용 AI 데이터 생성"""
    test_data = {
        "serial": "AMR001",
        "x": 15.7,
        "y": 25.9,
        "img": "base64_test_image_data",
        "case": "test_situation",
        "timeStamp": "2025-08-06T16:00:00Z"
    }
    
    try:
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(test_data, f, indent=2, ensure_ascii=False)
        print(f"✅ 테스트 데이터 생성 완료: {file_path}")
        return True
    except Exception as e:
        print(f"❌ 테스트 데이터 생성 실패: {e}")
        return False

def main():
    """메인 함수"""
    print("=== 파일 기반 AI 데이터 수신 테스트 ===")
    print("AI 시스템이 JSON 파일로 데이터를 전달하는 방식 테스트")
    print("종료하려면 Ctrl+C를 누르세요.\n")
    
    # AI 파일 클라이언트 생성
    file_path = "/tmp/ai_data.json"
    ai_client = AIFileClient(file_path, "AMR001")
    
    # AI 데이터 콜백 설정
    ai_client.set_ai_data_callback(ai_data_callback)
    
    # 샘플 데이터 생성
    ai_client.create_sample_data()
    
    # 파일 모니터링 시작
    if ai_client.start_monitoring(interval=1.0):
        print("✅ AI 파일 모니터링 시작 성공")
        print(f"\n⏳ AI 데이터 파일 모니터링 중... ({file_path})")
        print("AI 시스템이 파일을 업데이트하면 자동으로 감지됩니다.")
        
        try:
            while True:
                time.sleep(1)
                
                # 현재 통계 출력 (10초마다)
                if int(time.time()) % 10 == 0:
                    stats = ai_client.get_reception_stats()
                    print(f"\n📊 수신 통계:")
                    print(f"   총 수신: {stats['total_received']}")
                    print(f"   모니터링: {'실행 중' if stats['monitoring'] else '중지됨'}")
                    print(f"   파일 존재: {'있음' if stats['file_exists'] else '없음'}")
                    print(f"   파일 경로: {stats['file_path']}")
                    
                    # 최신 데이터 출력
                    latest_data = stats['latest_ai_data']
                    if latest_data.get('serial'):
                        print(f"   최신 데이터: {latest_data['serial']} - ({latest_data['x']}, {latest_data['y']})")
                    
        except KeyboardInterrupt:
            print("\n\n⚠️  프로그램 종료 중...")
            
    else:
        print("❌ AI 파일 모니터링 시작 실패")
    
    # 모니터링 중지
    ai_client.stop_monitoring()

def test_manual_data_update():
    """수동 데이터 업데이트 테스트"""
    print("\n=== 수동 데이터 업데이트 테스트 ===")
    
    file_path = "/tmp/ai_data.json"
    ai_client = AIFileClient(file_path, "AMR001")
    
    # 콜백 설정
    ai_client.set_ai_data_callback(ai_data_callback)
    
    print("1. 초기 데이터 생성...")
    ai_client.create_sample_data()
    
    print("2. 데이터 읽기 테스트...")
    data = ai_client.get_ai_data()
    if data:
        print("✅ 데이터 읽기 성공")
    else:
        print("❌ 데이터 읽기 실패")
    
    print("3. 5초 후 새로운 데이터 생성...")
    time.sleep(5)
    
    new_data = {
        "serial": "AMR001",
        "x": 30.5,
        "y": 40.7,
        "img": "new_base64_image",
        "case": "updated_situation",
        "timeStamp": "2025-08-06T16:05:00Z"
    }
    
    try:
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(new_data, f, indent=2, ensure_ascii=False)
        print("✅ 새로운 데이터 생성 완료")
        
        # 데이터 읽기
        data = ai_client.get_ai_data()
        if data:
            print("✅ 새로운 데이터 읽기 성공")
        else:
            print("❌ 새로운 데이터 읽기 실패")
            
    except Exception as e:
        print(f"❌ 새로운 데이터 생성 실패: {e}")

if __name__ == "__main__":
    # 메인 테스트
    main()
    
    # 수동 업데이트 테스트
    test_manual_data_update() 