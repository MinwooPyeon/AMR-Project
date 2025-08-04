#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
단방향 MQTT 통신 테스트
"""

import time
import threading
from main.amr_real_data_sync import AMRRealDataSync
from mqtt.backend_mqtt_subscriber import BackendMQTTSubscriber
from utils.logger import mqtt_logger

def test_unidirectional_communication():
    mqtt_logger.info("=== 단방향 MQTT 통신 테스트 ===")
    mqtt_logger.info("임베디드에서 백엔드로 데이터를 전송하는 단방향 통신을 테스트합니다.")
    mqtt_logger.info("임베디드: 데이터 전송")
    mqtt_logger.info("백엔드: 데이터 수신")
    mqtt_logger.info("=" * 60)
    
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True)
    
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
    
    print("임베디드 동기화 시작...")
    amr_sync.start_sync()
    
    print("\n단방향 통신 테스트 시작...")
    print("임베디드에서 백엔드로 데이터가 전송됩니다.")
    print("전송 주기: 1초마다")
    print("전송 데이터: 시리얼, 상태, 위치, 속도")
    print("Ctrl+C로 종료")
    
    try:
        print("\n\n1. 전진 (5초)")
        print("   속도: 50% (좌측/우측 모터)")
        print("   임베디드 → 백엔드 데이터 전송 중...")
        amr_sync.move_forward(50.0)
        time.sleep(5)
        
        print("\n2. 정지 (3초)")
        print("   모터 정지")
        print("   임베디드 → 백엔드 데이터 전송 중...")
        amr_sync.stop_motor()
        time.sleep(3)
        
        print("\n3. 좌회전 (5초)")
        print("   속도: 좌측 35%, 우측 50%")
        print("   임베디드 → 백엔드 데이터 전송 중...")
        amr_sync.turn_left(50.0)
        time.sleep(5)
        
        print("\n4. 정지 (3초)")
        print("   모터 정지")
        print("   임베디드 → 백엔드 데이터 전송 중...")
        amr_sync.stop_motor()
        time.sleep(3)
        
        print("\n5. 우회전 (5초)")
        print("   속도: 좌측 50%, 우측 35%")
        print("   임베디드 → 백엔드 데이터 전송 중...")
        amr_sync.turn_right(50.0)
        time.sleep(5)
        
        print("\n6. 최종 정지 (3초)")
        print("   모터 정지")
        print("   임베디드 → 백엔드 데이터 전송 중...")
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

if __name__ == "__main__":
    test_unidirectional_communication() 