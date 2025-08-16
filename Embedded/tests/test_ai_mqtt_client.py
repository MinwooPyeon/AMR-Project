#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from mqtt.ai_mqtt_client import AIMQTTClient

def test_ai_mqtt_client():
    print("=== AI MQTT Client 테스트 ===")
    print("임베디드와 AI 통신을 테스트합니다.")
    print("MQTT 브로커: localhost:1883")
    print("토픽: robot_data (임베디드 → AI), ai_data (AI 수신)")
    print("=" * 60)
    
    ai_client = AIMQTTClient("AMR001", "localhost", 1883)
    
    def embedded_data_callback(data):
        print(f"\r임베디드 데이터 수신: ")
        print(f"  - Serial: {data.get('serial', 'N/A')}")
        print(f"  - State: {data.get('state', 'N/A')}")
        print(f"  - Position: ({data.get('x', 0)}, {data.get('y', 0)})")
        print(f"  - Speed: {data.get('speed', 0)}")
        print(f"  - Angle: {data.get('angle', 0)}")
        print()
    
    def ai_data_callback(data):
        print(f"\rAI 데이터 수신: ")
        print(f"  - Serial: {data.get('serial', 'N/A')}")
        print(f"  - Position: ({data.get('x', 0)}, {data.get('y', 0)})")
        print(f"  - Image: {len(data.get('img', ''))} bytes")
        print(f"  - Case: {data.get('case', 'N/A')}")
        print(f"  - Timestamp: {data.get('timeStamp', 'N/A')}")
        print()
    
    ai_client.set_embedded_data_callback(embedded_data_callback)
    ai_client.set_ai_data_callback(ai_data_callback)
    
    print("AI MQTT 브로커에 연결 중...")
    if not ai_client.connect_mqtt():
        print("AI MQTT 연결 실패")
        return
    
    print("AI MQTT 연결 성공")
    
    print("임베디드 데이터 구독 중...")
    if not ai_client.subscribe_to_embedded_data("AMR001"):
        print("임베디드 데이터 구독 실패")
        return
    
    print("AI 데이터 구독 중...")
    if not ai_client.subscribe_to_ai_data("AMR001"):
        print("AI 데이터 구독 실패")
        return
    
    print("모든 토픽 구독 성공")
    
    print("\n데이터 수신 대기 중... (30초)")
    print("Ctrl+C로 종료하거나 명령을 입력하세요:")
    print("  - 'status': 현재 상태 확인")
    print("  - 'embedded': 임베디드 데이터 확인")
    print("  - 'ai': AI 데이터 확인")
    print("  - 'position': 로봇 위치 확인")
    
    start_time = time.time()
    
    try:
        while time.time() - start_time < 30:
            try:
                import select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    command = input().strip()
                    
                    if command == "status":
                        stats = ai_client.get_reception_stats()
                        print(f"\n통신 통계:")
                        for key, value in stats.items():
                            if key not in ["latest_embedded_data", "latest_ai_data"]:
                                print(f"  - {key}: {value}")
                        
                    elif command == "embedded":
                        embedded_data = ai_client.get_latest_embedded_data()
                        print(f"\n최신 임베디드 데이터:")
                        print(json.dumps(embedded_data, indent=2, ensure_ascii=False))
                        
                    elif command == "ai":
                        ai_data = ai_client.get_latest_ai_data()
                        print(f"\n최신 AI 데이터:")
                        print(json.dumps(ai_data, indent=2, ensure_ascii=False))
                        
                    elif command == "position":
                        x, y = ai_client.get_robot_position()
                        state = ai_client.get_robot_state()
                        speed = ai_client.get_robot_speed()
                        angle = ai_client.get_robot_angle()
                        print(f"\n로봇 상태:")
                        print(f"  - 위치: ({x}, {y})")
                        print(f"  - 상태: {state}")
                        print(f"  - 속도: {speed}")
                        print(f"  - 각도: {angle}")
                        
                    else:
                        print(f"알 수 없는 명령: {command}")
                        
            except (EOFError, KeyboardInterrupt):
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n테스트 중단됨")
    
    stats = ai_client.get_reception_stats()
    print(f"\n통신 통계:")
    for key, value in stats.items():
        if key not in ["latest_embedded_data", "latest_ai_data"]:
            print(f"  - {key}: {value}")
    
    embedded_data = stats.get("latest_embedded_data", {})
    if embedded_data:
        print(f"\n최신 임베디드 데이터:")
        print(json.dumps(embedded_data, indent=2, ensure_ascii=False))
    
    ai_data = stats.get("latest_ai_data", {})
    if ai_data:
        print(f"\n최신 AI 데이터:")
        print(json.dumps(ai_data, indent=2, ensure_ascii=False))
    
    ai_client.disconnect_mqtt()
    print("\nAI MQTT Client 정리 완료")

if __name__ == "__main__":
    test_ai_mqtt_client() 