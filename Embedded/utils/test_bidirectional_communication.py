#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
<<<<<<< HEAD
단방향 MQTT 통신 테스트
=======
단방향 통신 테스트
AI → Embedded → Backend 데이터 흐름 테스트
>>>>>>> c63d83b (merge: Resolve conflicts)
"""

import time
import threading
from main.amr_real_data_sync import AMRRealDataSync
<<<<<<< HEAD
from mqtt.backend_mqtt_subscriber import BackendMQTTSubscriber
from utils.logger import mqtt_logger

def test_unidirectional_communication():
    mqtt_logger.info("=== 단방향 MQTT 통신 테스트 ===")
    mqtt_logger.info("임베디드에서 백엔드로 데이터를 전송하는 단방향 통신을 테스트합니다.")
    mqtt_logger.info("임베디드: 데이터 전송")
    mqtt_logger.info("백엔드: 데이터 수신")
=======

from utils.logger import mqtt_logger

def test_unidirectional_communication():
    """단방향 통신 테스트"""
    print("=" * 60)
    print("=== 단방향 통신 테스트 ===")
    print("=" * 60)
    mqtt_logger.info("AI → Embedded → Backend 단방향 통신을 테스트합니다.")
    mqtt_logger.info("AI: localhost:1883 (데이터 전송)")
    mqtt_logger.info("Embedded: 데이터 수신 및 처리")
    mqtt_logger.info("Backend: 192.168.100.141:1883 (데이터 전송)")
>>>>>>> c63d83b (merge: Resolve conflicts)
    mqtt_logger.info("=" * 60)
    
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True)
    
<<<<<<< HEAD
    backend = BackendMQTTSubscriber("192.168.100.141", 1883)
    
    def amr_data_callback(data):
        print(f"\r🤖 임베디드 데이터 수신: "
              f"시리얼={data.get('serial', 'N/A')} | "
              f"상태={data.get('state', 'N/A')} | "
              f"위치=({data.get('x', 0):.1f}, {data.get('y', 0):.1f}) | "
              f"속도={data.get('speed', 0):.1f}", end="")
    
    backend.set_amr_data_callback(amr_data_callback)
    
    print("MQTT 브로커에 연결 중...")
    if not backend.connect_mqtt():
        print("❌ 백엔드 MQTT 연결 실패")
        return
    
    print("✅ 백엔드 MQTT 연결 성공")
    
    print("임베디드 데이터 구독 중...")
    if not backend.subscribe_to_amr_data("AMR001"):
        print("❌ AMR 데이터 구독 실패")
        return
    
    print("✅ AMR 데이터 구독 성공")
=======
    print("MQTT 브로커에 연결 중...")
    if not amr_sync._connect_mqtt_with_retry():
        print("❌ MQTT 연결 실패")
        return
    
    print("✅ MQTT 연결 성공")
>>>>>>> c63d83b (merge: Resolve conflicts)
    
    print("임베디드 동기화 시작...")
    amr_sync.start_sync()
    
    print("\n단방향 통신 테스트 시작...")
<<<<<<< HEAD
    print("임베디드에서 백엔드로 데이터가 전송됩니다.")
=======
    print("AI → Embedded → Backend 데이터 흐름을 테스트합니다.")
>>>>>>> c63d83b (merge: Resolve conflicts)
    print("전송 주기: 1초마다")
    print("전송 데이터: 시리얼, 상태, 위치, 속도")
    print("Ctrl+C로 종료")
    
    try:
        print("\n\n1. 전진 (5초)")
        print("   속도: 50% (좌측/우측 모터)")
<<<<<<< HEAD
        print("   임베디드 → 백엔드 데이터 전송 중...")
=======
        print("   AI → Embedded → Backend 데이터 전송 중...")
>>>>>>> c63d83b (merge: Resolve conflicts)
        amr_sync.move_forward(50.0)
        time.sleep(5)
        
        print("\n2. 정지 (3초)")
        print("   모터 정지")
<<<<<<< HEAD
        print("   임베디드 → 백엔드 데이터 전송 중...")
=======
        print("   AI → Embedded → Backend 데이터 전송 중...")
>>>>>>> c63d83b (merge: Resolve conflicts)
        amr_sync.stop_motor()
        time.sleep(3)
        
        print("\n3. 좌회전 (5초)")
        print("   속도: 좌측 35%, 우측 50%")
<<<<<<< HEAD
        print("   임베디드 → 백엔드 데이터 전송 중...")
=======
        print("   AI → Embedded → Backend 데이터 전송 중...")
>>>>>>> c63d83b (merge: Resolve conflicts)
        amr_sync.turn_left(50.0)
        time.sleep(5)
        
        print("\n4. 정지 (3초)")
        print("   모터 정지")
<<<<<<< HEAD
        print("   임베디드 → 백엔드 데이터 전송 중...")
=======
        print("   AI → Embedded → Backend 데이터 전송 중...")
>>>>>>> c63d83b (merge: Resolve conflicts)
        amr_sync.stop_motor()
        time.sleep(3)
        
        print("\n5. 우회전 (5초)")
        print("   속도: 좌측 50%, 우측 35%")
<<<<<<< HEAD
        print("   임베디드 → 백엔드 데이터 전송 중...")
=======
        print("   AI → Embedded → Backend 데이터 전송 중...")
>>>>>>> c63d83b (merge: Resolve conflicts)
        amr_sync.turn_right(50.0)
        time.sleep(5)
        
        print("\n6. 최종 정지 (3초)")
        print("   모터 정지")
<<<<<<< HEAD
        print("   임베디드 → 백엔드 데이터 전송 중...")
=======
        print("   AI → Embedded → Backend 데이터 전송 중...")
>>>>>>> c63d83b (merge: Resolve conflicts)
        amr_sync.stop_motor()
        time.sleep(3)
        
        print("\n" + "=" * 60)
        print("=== 테스트 완료 ===")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
        amr_sync.stop_motor()
    
    print("\n" + "=" * 60)
    print("=== 최종 통계 ===")
    print("=" * 60)
    
    amr_stats = amr_sync.get_sync_stats()
    print(f"\n🤖 임베디드 시스템 통계:")
<<<<<<< HEAD
    print(f"  - 등록된 센서 수: {amr_stats['registered_sensors']}")
    print(f"  - 활성 센서 수: {amr_stats['active_sensors']}")
    print(f"  - 동기화 속도: {amr_stats['sync_rate']:.2f} Hz")
    print(f"  - 데이터 손실률: {amr_stats['data_loss_rate']:.2f}%")

    print(f"  - 모터 상태: {amr_stats['motor_status']}")
    
    backend_stats = backend.get_reception_stats()
    print(f"\n📡 백엔드 시스템 통계:")
    print(f"  - 총 수신 메시지: {backend_stats['total_received']}")
    print(f"  - 마지막 수신 시간: {backend_stats['last_received_time']}")
    print(f"  - MQTT 연결 상태: {'연결됨' if backend_stats['mqtt_connected'] else '연결 안됨'}")
    
    latest_data = backend_stats.get("latest_data", {})
    if latest_data:
        print(f"\n📋 최신 임베디드 데이터:")
        import json
        print(json.dumps(latest_data, indent=2, ensure_ascii=False))
    
    print(f"\n📤 임베디드에서 전송되는 데이터 구조:")
    sample_data = {
        "serial": "AMR001",
        "state": "RUNNING",
        "x": "10.0",
        "y": "20.0",
        "speed": "25.0"
    }
    print(json.dumps(sample_data, indent=2, ensure_ascii=False))
    
    print(f"\n💡 단방향 통신 동작 방식:")
    print(f"  - 임베디드: 1초마다 센서 데이터 수집 및 MQTT 전송")
    print(f"  - 백엔드: status 토픽에서 데이터 수신")
    print(f"  - 전송 데이터: 시리얼, 상태, 위치(x,y), 속도")
    print(f"  - 데이터 형식: JSON")
    print(f"  - 전송 주기: 1Hz (1초마다)")
    
    print("\n시스템 정리 중...")
    amr_sync.stop_sync()
    backend.disconnect_mqtt()
    print("✅ 단방향 통신 테스트 완료")
=======
    print(f"  - 동기화 횟수: {amr_stats['total_sync_count']}")
    print(f"  - 마지막 동기화: {amr_stats['last_sync_time']}")
    print(f"  - 동기화 상태: {'실행 중' if amr_stats['sync_running'] else '중지됨'}")
    print(f"  - 데이터 손실: {amr_stats['data_loss_count']}")
    print(f"  - 모터 상태: {amr_stats['motor_status']}")
    
    # MQTT 통계
    if amr_sync.mqtt_transmitter:
        mqtt_stats = amr_sync.mqtt_transmitter.get_transmission_stats()
        print(f"\n📡 MQTT 통신 통계:")
        print(f"  - 전송된 메시지 수: {mqtt_stats['total_sent']}")
        print(f"  - 마지막 전송 시간: {mqtt_stats['last_sent_time']}")
        print(f"  - 연결 상태: {'연결됨' if mqtt_stats['connected'] else '연결 안됨'}")
    
    print("\n" + "=" * 60)
    print("=== 데이터 흐름 요약 ===")
    print("=" * 60)
    print("AI 시스템 (localhost:1883)")
    print("    ↓ 데이터 전송")
    print("Embedded 시스템 (데이터 수신 및 처리)")
    print("    ↓ 데이터 전송")
    print("Backend 시스템 (192.168.100.141:1883)")
    print("=" * 60)
>>>>>>> c63d83b (merge: Resolve conflicts)

if __name__ == "__main__":
    test_unidirectional_communication() 