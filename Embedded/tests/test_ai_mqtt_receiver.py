#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI MQTT 수신 테스트 스크립트
localhost:1883에서 AI 데이터를 수신하는 테스트
"""

import sys
import os
import time
import json
import signal

# 프로젝트 루트를 Python 경로에 추가
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from mqtt.ai_mqtt_client import AIMQTTClient
from utils.logger import mqtt_logger

def ai_data_callback(data):
    """AI 데이터 수신 콜백"""
    print(f"\n📥 AI 데이터 수신:")
    print(f"   시리얼: {data.get('serial', 'N/A')}")
    print(f"   위치: ({data.get('x', 0)}, {data.get('y', 0)})")
    print(f"   이미지: {'있음' if data.get('img') else '없음'}")
    print(f"   케이스: {data.get('case', 'N/A')}")
    print(f"   타임스탬프: {data.get('timeStamp', 'N/A')}")

def main():
    """메인 함수"""
    print("=== AI MQTT 수신 테스트 ===")
    print("localhost:1883에서 AI 데이터를 수신합니다.")
    print("종료하려면 Ctrl+C를 누르세요.\n")
    
    # AI MQTT 클라이언트 생성
    ai_client = AIMQTTClient("AMR001", "localhost", 1883)
    
    # AI 데이터 콜백 설정
    ai_client.set_ai_data_callback(ai_data_callback)
    
    # MQTT 연결
    if ai_client.connect_mqtt():
        print("✅ AI MQTT 연결 성공")
        
        # AI 데이터 구독
        if ai_client.subscribe_to_ai_data("AMR001"):
            print("✅ AI 데이터 구독 성공")
            print("\n⏳ AI 데이터 수신 대기 중...")
            
            try:
                while True:
                    time.sleep(1)
                    
                    # 현재 통계 출력 (10초마다)
                    if int(time.time()) % 10 == 0:
                        stats = ai_client.get_reception_stats()
                        print(f"\n📊 수신 통계:")
                        print(f"   총 수신: {stats['total_received']}")
                        print(f"   연결 상태: {'연결됨' if stats['mqtt_connected'] else '연결 안됨'}")
                        
            except KeyboardInterrupt:
                print("\n\n⚠️  프로그램 종료 중...")
                
        else:
            print("❌ AI 데이터 구독 실패")
        
        ai_client.disconnect_mqtt()
    else:
        print("❌ AI MQTT 연결 실패")

if __name__ == "__main__":
    main() 