#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI Subscriber와 AMR 실제 데이터 동기화 통합 테스트
"""

import time
import threading
import json
from amr_real_data_sync import AMRRealDataSync

def test_amr_with_ai_subscriber():
    """AI Subscriber와 AMR 실제 데이터 동기화 통합 테스트"""
    print("=== AI Subscriber와 AMR 실제 데이터 동기화 통합 테스트 ===")
    print("AI에서 전송하는 위치 및 명령 데이터를 수신하여 AMR 시스템에 통합합니다.")
    print("=" * 80)
    
    # AMR 실제 데이터 동기화 시스템 생성
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True, enable_backup=True)
    
    # AI Subscriber 상태 확인
    if amr_sync.ai_subscriber:
        print("✅ AI Position Subscriber 활성화됨")
        print("구독 중인 토픽:")
        print("  - /ai/position (Pose2D)")
        print("  - /ai/position_point (Point)")
        print("  - /ai/position_array (Float64MultiArray)")
        print("  - /ai/position_json (String - JSON)")
        print("  - /position (String - AI 명령 데이터)")
        print("")
        print("AI 명령 데이터 구조:")
        print("  {")
        print('    "MOVING_FORWARD": "",')
        print('    "ROTATE_LEFT": "",')
        print('    "ROTATE_RIGHT": "",')
        print('    "MOVING_BACKWARD": "",')
        print('    "STOP": "",')
        print('    "img": ".jpg",')
        print('    "situation": "",')
        print('    "x": "10.0",')
        print('    "y": "10.0"')
        print("  }")
    else:
        print("❌ AI Position Subscriber 비활성화됨")
        print("AI 위치 데이터는 시뮬레이션으로 대체됩니다")
    
    # 데이터 콜백 설정 - 센서 데이터를 실시간으로 출력
    def data_callback(data):
        motor_speeds = data['motor_speeds']
        # 평균 속도 계산
        left_speed = abs(motor_speeds['left_speed'])
        right_speed = abs(motor_speeds['right_speed'])
        average_speed = (left_speed + right_speed) / 2.0
        
        # AI 명령 정보
        ai_command = amr_sync.get_ai_command()
        ai_situation = amr_sync.get_ai_situation()
        ai_image = amr_sync.get_ai_image()
        
        mqtt_status = "✅ MQTT" if amr_sync.enable_mqtt else "❌ MQTT"
        ai_status = "✅ AI" if amr_sync.ai_subscriber else "❌ AI"
        
        print(f"\r실시간 센서 데이터: 배터리 {data['battery_level']:.1f}% | "
              f"속도 {average_speed:.1f} | "
              f"위치 ({data['position'][0]:.1f}, {data['position'][1]:.1f}) | "
              f"모터 상태: L={motor_speeds['left_speed']:.1f}, R={motor_speeds['right_speed']:.1f} | "
              f"{mqtt_status} | {ai_status} | AI명령: {ai_command} | 상황: {ai_situation}", end="")
    
    amr_sync.set_data_callback(data_callback)
    
    # 동기화 시작
    amr_sync.start_sync()
    
    print("\nAI Subscriber 통합 테스트 시작...")
    print(f"MQTT 전송 상태: {'활성화' if amr_sync.enable_mqtt else '비활성화'}")
    print(f"AI Subscriber 상태: {'활성화' if amr_sync.ai_subscriber else '비활성화'}")
    
    # AI 위치 데이터 시뮬레이션 (AI Subscriber가 없을 때만 사용)
    def simulate_ai_position():
        import random
        while True:
            x = 10.0 + random.uniform(-2.0, 2.0)
            y = 20.0 + random.uniform(-2.0, 2.0)
            amr_sync.update_ai_position(x, y)
            time.sleep(0.3)
    
    # AI Subscriber가 없으면 시뮬레이션 스레드 시작
    if not amr_sync.ai_subscriber:
        ai_thread = threading.Thread(target=simulate_ai_position, daemon=True)
        ai_thread.start()
        print("AI 위치 데이터 시뮬레이션 활성화")
    else:
        print("AI Position Subscriber 활성화 - 실제 AI 데이터 사용")
        print("AI에서 다음 토픽으로 데이터를 전송하세요:")
        print("  - /position (AI 명령 데이터)")
        print("  - /ai/position (위치 데이터)")
    
    try:
        # 테스트 실행 (30초)
        print("\n30초간 AI 데이터 수신 테스트...")
        for i in range(30):
            time.sleep(1)
            if i % 5 == 0:
                print(f"\n진행률: {i+1}/30초")
        
        print("\n" + "=" * 80)
        print("=== 테스트 완료 ===")
        print("=" * 80)
        
        # 최종 통계 출력
        stats = amr_sync.get_sync_stats()
        print(f"\n📊 최종 통계:")
        print(f"  - 등록된 센서 수: {stats['registered_sensors']}")
        print(f"  - 활성 센서 수: {stats['active_sensors']}")
        print(f"  - 동기화 속도: {stats['sync_rate']:.2f} Hz (1초마다)")
        print(f"  - 데이터 손실률: {stats['data_loss_rate']:.2f}%")
        print(f"  - 배터리 레벨: {stats['battery_level']:.1f}%")
        print(f"  - 모터 상태: {stats['motor_status']}")
        
        # AI 데이터 통계
        if amr_sync.ai_subscriber:
            ai_data = amr_sync.ai_subscriber.get_current_ai_data()
            print(f"\n🤖 AI 데이터 통계:")
            print(f"  - 현재 AI 명령: {amr_sync.get_ai_command()}")
            print(f"  - 현재 AI 상황: {amr_sync.get_ai_situation()}")
            print(f"  - 현재 AI 이미지: {amr_sync.get_ai_image()}")
            print(f"  - AI 위치: ({amr_sync.get_ai_position()[0]:.1f}, {amr_sync.get_ai_position()[1]:.1f})")
        
        # MQTT 전송 통계
        if amr_sync.enable_mqtt and amr_sync.mqtt_transmitter:
            mqtt_stats = amr_sync.mqtt_transmitter.get_transmission_stats()
            print(f"\n📡 MQTT 전송 통계:")
            for key, value in mqtt_stats.items():
                print(f"  - {key}: {value}")
        
        # 백엔드 JSON 데이터 예시 출력
        if amr_sync.enable_mqtt:
            print(f"\n📋 백엔드로 전송되는 JSON 데이터 예시:")
            sample_data = {
                "serial": "AMR001",
                "state": "RUNNING",
                "x": "10.0",
                "y": "20.0",
                "speed": "25.0"
            }
            print(json.dumps(sample_data, indent=2, ensure_ascii=False))
        
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
        amr_sync.stop_motor()
    finally:
        # 동기화 중지
        amr_sync.stop_sync()
        print("\n✅ 시스템 정리 완료")

if __name__ == "__main__":
    test_amr_with_ai_subscriber() 