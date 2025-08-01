#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
양방향 MQTT 통신 테스트
AMR과 백엔드 간의 양방향 통신을 테스트
"""

import time
import threading
import logging
from amr_real_data_sync import AMRRealDataSync
from backend_mqtt_subscriber import BackendMQTTSubscriber

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def test_bidirectional_communication():
    """양방향 MQTT 통신 테스트"""
    print("=== 양방향 MQTT 통신 테스트 ===")
    print("AMR과 백엔드 간의 양방향 통신을 테스트합니다.")
    print("AMR: 데이터 전송 + 명령 수신")
    print("백엔드: 데이터 수신 + 명령 전송")
    print("=" * 60)
    
    # AMR 시스템 생성
    print("AMR 시스템 초기화 중...")
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True)
    
    # 백엔드 시스템 생성
    print("백엔드 시스템 초기화 중...")
    backend = BackendMQTTSubscriber("192.168.100.141", 1883)
    
    # AMR 데이터 콜백 설정
    def amr_data_callback(data):
        print(f"\r🤖 AMR 데이터: "
              f"시리얼={data.get('serial', 'N/A')} | "
              f"상태={data.get('status', 'N/A')} | "
              f"배터리={data.get('battery_level', 0):.1f}% | "
              f"위치=({data.get('x', 0):.1f}, {data.get('y', 0):.1f}) | "
              f"속도={data.get('speed', 0):.1f}", end="")
    
    # 백엔드 명령 콜백 설정
    def command_callback(data):
        print(f"\n📨 백엔드 명령 수신: {data}")
    
    backend.set_amr_data_callback(amr_data_callback)
    backend.set_command_callback(command_callback)
    
    # MQTT 연결
    print("MQTT 브로커에 연결 중...")
    if not backend.connect_mqtt():
        print("❌ 백엔드 MQTT 연결 실패")
        return
    
    print("✅ 백엔드 MQTT 연결 성공")
    
    # 구독 설정
    print("토픽 구독 중...")
    if not backend.subscribe_to_amr_data("AMR001"):
        print("❌ AMR 데이터 구독 실패")
        return
    
    if not backend.subscribe_to_commands("AMR001"):
        print("❌ 명령 구독 실패")
        return
    
    print("✅ 토픽 구독 성공")
    
    # AMR 동기화 시작
    print("AMR 동기화 시작...")
    amr_sync.start_sync()
    
    print("\n양방향 통신 테스트 시작...")
    print("명령을 입력하세요:")
    print("  - 'MOVE_FORWARD': 전진")
    print("  - 'MOVE_BACKWARD': 후진")
    print("  - 'ROTATE_LEFT': 좌회전")
    print("  - 'ROTATE_RIGHT': 우회전")
    print("  - 'stop': 정지")
    print("  - 'custom': 사용자 정의 명령")
    print("  - 'quit': 종료")
    
    try:
        while True:
            # 사용자 입력 확인
            try:
                import select
                import sys
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    command = input().strip()
                    
                    if command == "quit":
                        break
                    elif command == "MOVE_FORWARD":
                        backend.publish_command("AMR001", {
                            "action": "MOVE_FORWARD",
                            "speed": 50.0
                        })
                        print("📤 전진 명령 전송")
                    elif command == "MOVE_BACKWARD":
                        backend.publish_command("AMR001", {
                            "action": "MOVE_BACKWARD",
                            "speed": 50.0
                        })
                        print("📤 후진 명령 전송")
                    elif command == "ROTATE_LEFT":
                        backend.publish_command("AMR001", {
                            "action": "ROTATE_LEFT",
                            "speed": 50.0
                        })
                        print("📤 좌회전 명령 전송")
                    elif command == "ROTATE_RIGHT":
                        backend.publish_command("AMR001", {
                            "action": "ROTATE_RIGHT",
                            "speed": 50.0
                        })
                        print("📤 우회전 명령 전송")
                    elif command == "stop":
                        backend.publish_command("AMR001", {
                            "action": "stop_motor"
                        })
                        print("📤 정지 명령 전송")
                    elif command == "custom":
                        print("사용자 정의 명령을 입력하세요 (JSON 형식):")
                        try:
                            import json
                            custom_cmd = json.loads(input())
                            backend.publish_command("AMR001", custom_cmd)
                            print("📤 사용자 정의 명령 전송")
                        except json.JSONDecodeError:
                            print("❌ 잘못된 JSON 형식입니다.")
                    else:
                        print(f"❓ 알 수 없는 명령: {command}")
                        
            except (EOFError, KeyboardInterrupt):
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
    
    # 최종 통계 출력
    print("\n" + "=" * 60)
    print("=== 최종 통계 ===")
    print("=" * 60)
    
    # AMR 통계
    amr_stats = amr_sync.get_sync_stats()
    print(f"\n🤖 AMR 시스템 통계:")
    for key, value in amr_stats.items():
        print(f"  - {key}: {value}")
    
    # 백엔드 통계
    backend_stats = backend.get_reception_stats()
    print(f"\n📡 백엔드 시스템 통계:")
    for key, value in backend_stats.items():
        if key != "latest_data":
            print(f"  - {key}: {value}")
    
    # 최신 데이터 출력
    latest_data = backend_stats.get("latest_data", {})
    if latest_data:
        print(f"\n📋 최신 AMR 데이터:")
        import json
        print(json.dumps(latest_data, indent=2, ensure_ascii=False))
    
    # 시스템 정리
    print("\n시스템 정리 중...")
    amr_sync.stop_sync()
    backend.disconnect_mqtt()
    print("✅ 양방향 통신 테스트 완료")

if __name__ == "__main__":
    test_bidirectional_communication() 