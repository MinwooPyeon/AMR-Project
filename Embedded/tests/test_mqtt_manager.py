#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MQTT 매니저 통합 테스트 스크립트
AI 수신 + Backend 전송 통합 테스트
"""

import sys
import os
import time
import json
import signal

# 프로젝트 루트를 Python 경로에 추가
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from mqtt.mqtt_manager import MQTTManager
from utils.logger import mqtt_logger

def ai_command_callback(command):
    """AI 명령 수신 콜백"""
    print(f"\n🤖 AI 명령 수신:")
    print(f"   명령: {command}")
    
    # Backend로 데이터 전송
    backend_data = {
        "state": "RUNNING",
        "speed": "25"
    }
    
    if mqtt_manager.send_to_backend(backend_data):
        print("✅ Backend 전송 성공")
    else:
        print("❌ Backend 전송 실패")

def main():
    """메인 함수"""
    print("=== MQTT 매니저 통합 테스트 ===")
    print("AI 수신 + Backend 전송 통합 테스트")
    print("종료하려면 Ctrl+C를 누르세요.\n")
    
    global mqtt_manager
    mqtt_manager = MQTTManager("AMR001")
    
    # AI 명령 콜백 설정
    mqtt_manager.set_ai_command_callback(ai_command_callback)
    
    # 모든 MQTT 연결
    if mqtt_manager.connect_all():
        print("✅ 모든 MQTT 연결 성공")
        print("\n⏳ AI 데이터 수신 및 Backend 전송 대기 중...")
        
        try:
            while True:
                time.sleep(1)
                
                # 현재 통계 출력 (10초마다)
                if int(time.time()) % 10 == 0:
                    stats = mqtt_manager.get_stats()
                    connection_status = mqtt_manager.get_connection_status()
                    
                    print(f"\n📊 통합 통계:")
                    print(f"   Backend 전송: {stats['backend_sent']}")
                    print(f"   AI 수신: {stats['ai_received']}")
                    print(f"   Backend 연결: {'연결됨' if connection_status['backend_transmitter'] else '연결 안됨'}")
                    print(f"   AI 연결: {'연결됨' if connection_status['ai_client'] else '연결 안됨'}")
                    
        except KeyboardInterrupt:
            print("\n\n⚠️  프로그램 종료 중...")
            
    else:
        print("❌ MQTT 연결 실패")
    
    # 연결 해제
    mqtt_manager.disconnect_all()

if __name__ == "__main__":
    main() 