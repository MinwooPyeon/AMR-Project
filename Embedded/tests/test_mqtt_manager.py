#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from mqtt.mqtt_manager import MQTTManager

def test_mqtt_manager():
    print("=== MQTT Manager 테스트 ===")
    print("백엔드 송신과 AI 수신을 테스트합니다.")
    print("백엔드: 192.168.100.141:1883 (status 토픽) - 송신만")
    print("AI: localhost:1883 (position 토픽) - 수신만")
    print("=" * 60)
    
    manager = MQTTManager("AMR001")
    
    def ai_command_callback(data):
        print(f"\r AI 명령: ")
        for key, value in data.items():
            if value:  # 값이 있는 명령만 표시
                print(f"  - {key}: {value}")
        print()
    
    manager.set_ai_command_callback(ai_command_callback)
    
    print("모든 MQTT 연결 시도 중...")
    if not manager.connect_all():
        print("일부 MQTT 연결 실패")
        return
    
    print("MQTT 연결 성공")
    
    print("\n데이터 전송 테스트 시작...")
    print("Ctrl+C로 종료하거나 명령을 입력하세요:")
    print("  - 'backend': 백엔드로 데이터 전송")
    print("  - 'status': 연결 상태 확인")
    print("  - 'ai_command': 현재 AI 명령 확인")
    print("  - 'ai_situation': AI 상황 확인")
    
    try:
        while True:
            try:
                import select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    command = input().strip()
                    
                    if command == "backend":
                        # AI에서 받은 좌표로 백엔드 데이터 전송
                        success = manager.send_to_backend({"state": "RUNNING"})
                        if success:
                            print("백엔드로 데이터 전송 성공 (AI 좌표 사용)")
                        else:
                            print("백엔드로 데이터 전송 실패")
                        
                    elif command == "status":
                        status = manager.get_connection_status()
                        stats = manager.get_stats()
                        print(f"\n연결 상태:")
                        for key, value in status.items():
                            print(f"  - {key}: {'연결됨' if value else '연결 안됨'}")
                        print(f"\n통계:")
                        for key, value in stats.items():
                            if not key.endswith('_connected'):
                                print(f"  - {key}: {value}")
                        
                    elif command == "ai_command":
                        active_cmd = manager.get_active_ai_command()
                        ai_data = manager.get_ai_command_data()
                        print(f"\n현재 AI 상태:")
                        print(f"  - 활성 명령: {active_cmd}")
                        if ai_data:
                            print(f"  - AI 데이터: {json.dumps(ai_data, indent=2, ensure_ascii=False)}")
                        
                    elif command == "ai_situation":
                        situation = manager.get_ai_situation()
                        x, y = manager.get_ai_position()
                        img = manager.get_ai_image()
                        print(f"\nAI 상황:")
                        print(f"  - 상황: {situation}")
                        print(f"  - 위치: ({x}, {y})")
                        print(f"  - 이미지: {img}")
                        
                    else:
                        print(f"알 수 없는 명령: {command}")
                        
            except (EOFError, KeyboardInterrupt):
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n 테스트 중단됨")
    
    print("\n" + "=" * 60)
    print("=== 최종 통계 ===")
    print("=" * 60)
    
    stats = manager.get_stats()
    for key, value in stats.items():
        print(f"  - {key}: {value}")
    
    print("\n시스템 정리 중...")
    manager.disconnect_all()
    print("MQTT Manager 정리 완료")

if __name__ == "__main__":
    test_mqtt_manager() 