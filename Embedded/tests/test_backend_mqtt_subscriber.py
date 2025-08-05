#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from mqtt.backend_mqtt_subscriber import BackendMQTTSubscriber

def test_backend_mqtt_subscriber():
    print("=== 백엔드 MQTT Subscriber 테스트 ===")
    print("AMR에서 전송하는 데이터를 수신하고 명령을 보낼 수 있습니다.")
    print("MQTT 브로커: 192.168.100.141:1883")
    print("=" * 60)
    
    backend = BackendMQTTSubscriber("192.168.100.141", 1883)
    
    def amr_data_callback(data):
        print(f"\rAMR 데이터 수신: "
              f"시리얼={data.get('serial', 'N/A')} | "
              f"상태={data.get('status', 'N/A')} | "
              f"위치=({data.get('x', 0):.1f}, {data.get('y', 0):.1f}) | "
              f"속도={data.get('speed', 0):.1f}", end="")
    
    def command_callback(data):
        print(f"\n명령 수신: {data}")
    
    backend.set_amr_data_callback(amr_data_callback)
    backend.set_command_callback(command_callback)
    
    print("MQTT 브로커에 연결 중...")
    if not backend.connect_mqtt():
        print("MQTT 연결 실패")
        return
    
    print("MQTT 연결 성공")
    
    print("AMR 데이터 구독 중...")
    if not backend.subscribe_to_amr_data("AMR001"):
        print("AMR 데이터 구독 실패")
        return
    
    print("AMR 데이터 구독 성공")
    
    print("명령 구독 중...")
    if not backend.subscribe_to_commands("AMR001"):
        print("명령 구독 실패")
        return
    
    print("명령 구독 성공")
    
    print("\n데이터 수신 대기 중... (30초)")
    print("Ctrl+C로 종료하거나 명령을 입력하세요:")
    print("  - 'MOVE_FORWARD': 전진 명령")
    print("  - 'MOVE_BACKWARD': 후진 명령")
    print("  - 'ROTATE_LEFT': 좌회전 명령")
    print("  - 'ROTATE_RIGHT': 우회전 명령")
    print("  - 'stop': 정지 명령")
    print("  - 'custom': 사용자 정의 명령")
    
    start_time = time.time()
    
    try:
        while time.time() - start_time < 30:
            try:
                import select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    command = input().strip()
                    
                    if command == "MOVE_FORWARD":
                        backend.publish_command("AMR001", {
                            "action": "MOVE_FORWARD",
                            "speed": 50.0
                        })
                    elif command == "MOVE_BACKWARD":
                        backend.publish_command("AMR001", {
                            "action": "MOVE_BACKWARD",
                            "speed": 50.0
                        })
                    elif command == "ROTATE_LEFT":
                        backend.publish_command("AMR001", {
                            "action": "ROTATE_LEFT",
                            "speed": 50.0
                        })
                    elif command == "ROTATE_RIGHT":
                        backend.publish_command("AMR001", {
                            "action": "ROTATE_RIGHT",
                            "speed": 50.0
                        })
                    elif command == "stop":
                        backend.publish_command("AMR001", {
                            "action": "stop_motor"
                        })
                    elif command == "custom":
                        print("사용자 정의 명령을 입력하세요 (JSON 형식):")
                        try:
                            custom_cmd = json.loads(input())
                            backend.publish_command("AMR001", custom_cmd)
                        except json.JSONDecodeError:
                            print("잘못된 JSON 형식입니다.")
                    else:
                        print(f"알 수 없는 명령: {command}")
                        
            except (EOFError, KeyboardInterrupt):
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n테스트 중단됨")
    
    stats = backend.get_reception_stats()
    print(f"\n수신 통계:")
    for key, value in stats.items():
        if key != "latest_data":
            print(f"  - {key}: {value}")
    
    latest_data = stats.get("latest_data", {})
    if latest_data:
        print(f"\n최신 AMR 데이터:")
        print(json.dumps(latest_data, indent=2, ensure_ascii=False))
    
    backend.disconnect_mqtt()
    print("\n백엔드 MQTT Subscriber 정리 완료")

if __name__ == "__main__":
    test_backend_mqtt_subscriber() 