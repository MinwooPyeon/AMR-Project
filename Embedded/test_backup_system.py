#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
백업 시스템 테스트
MQTT 통신이 안될 때만 데이터를 백업하는 기능을 테스트
"""

import time
import json
import os
from datetime import datetime
from amr_real_data_sync import AMRRealDataSync

def test_backup_system():
    """백업 시스템 테스트"""
    print("=== 백업 시스템 테스트 ===")
    print("MQTT 통신이 안될 때만 데이터를 백업하는 기능을 테스트합니다.")
    print("백업 조건: MQTT 연결이 안될 때만")
    print("백업 주기: 5초마다")
    print("백업 데이터 구조: 백엔드 JSON 형태")
    print("=" * 60)
    
    # AMR 실제 데이터 동기화 시스템 생성 (MQTT 비활성화, 백업 활성화)
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=False, enable_backup=True)
    
    # 백업 통계 확인
    backup_stats = amr_sync.get_backup_stats()
    print(f"\n📊 백업 시스템 상태:")
    print(f"  - 활성화: {'예' if backup_stats['enabled'] else '아니오'}")
    if backup_stats['enabled']:
        print(f"  - 백업 디렉토리: {backup_stats['backup_directory']}")
        print(f"  - 백업 주기: {backup_stats['backup_interval']}초")
        print(f"  - 기존 백업 파일 수: {backup_stats['backup_files_count']}")
        print(f"  - MQTT 연결 상태: {'연결됨' if backup_stats['mqtt_connected'] else '연결 안됨'}")
        print(f"  - 백업 조건: {backup_stats['backup_condition']}")
    
    # 데이터 콜백 설정 - 센서 데이터를 실시간으로 출력
    def data_callback(data):
        motor_speeds = data['motor_speeds']
        # 평균 속도 계산
        left_speed = abs(motor_speeds['left_speed'])
        right_speed = abs(motor_speeds['right_speed'])
        average_speed = (left_speed + right_speed) / 2.0
        
        # MQTT 연결 상태 확인
        mqtt_connected = amr_sync._is_mqtt_connected()
        backup_status = "✅ 백업" if not mqtt_connected else "❌ 백업 (MQTT 연결됨)"
        mqtt_status = "✅ MQTT" if mqtt_connected else "❌ MQTT"
        
        print(f"\r실시간 센서 데이터: 배터리 {data['battery_level']:.1f}% | "
              f"속도 {average_speed:.1f} | "
              f"위치 ({data['position'][0]:.1f}, {data['position'][1]:.1f}) | "
              f"모터 상태: L={motor_speeds['left_speed']:.1f}, R={motor_speeds['right_speed']:.1f} | "
              f"{mqtt_status} | {backup_status}", end="")
    
    amr_sync.set_data_callback(data_callback)
    
    # 동기화 시작
    amr_sync.start_sync()
    
    print("\n백업 테스트 시작...")
    print(f"백업 상태: {'활성화' if amr_sync.enable_backup else '비활성화'}")
    print(f"MQTT 상태: {'활성화' if amr_sync.enable_mqtt else '비활성화'}")
    print(f"백업 조건: MQTT 연결이 안될 때만 백업")
    
    # AI 위치 데이터 시뮬레이션
    def simulate_ai_position():
        import random
        while True:
            x = 10.0 + random.uniform(-2.0, 2.0)
            y = 10.0 + random.uniform(-2.0, 2.0)
            amr_sync.update_ai_position(x, y)
            time.sleep(0.3)
    
    # AI 시뮬레이션 스레드 시작
    import threading
    ai_thread = threading.Thread(target=simulate_ai_position, daemon=True)
    ai_thread.start()
    print("AI 위치 데이터 시뮬레이션 활성화")
    
    try:
        # 1. 전진 (3초)
        print("\n\n1. 전진 (3초)")
        print("   속도: 50% (좌측/우측 모터)")
        print("   MQTT 연결 없음 - 백업 데이터 저장 중...")
        amr_sync.move_forward(50.0)
        time.sleep(3)
        
        # 2. 정지 (2초)
        print("\n2. 정지 (2초)")
        print("   모터 정지")
        print("   MQTT 연결 없음 - 백업 데이터 저장 중...")
        amr_sync.stop_motor()
        time.sleep(2)
        
        # 3. 좌회전 (3초)
        print("\n3. 좌회전 (3초)")
        print("   속도: 좌측 35%, 우측 50%")
        print("   MQTT 연결 없음 - 백업 데이터 저장 중...")
        amr_sync.turn_left(50.0)
        time.sleep(3)
        
        # 4. 정지 (2초)
        print("\n4. 정지 (2초)")
        print("   모터 정지")
        print("   MQTT 연결 없음 - 백업 데이터 저장 중...")
        amr_sync.stop_motor()
        time.sleep(2)
        
        # 5. 우회전 (3초)
        print("\n5. 우회전 (3초)")
        print("   속도: 좌측 50%, 우측 35%")
        print("   MQTT 연결 없음 - 백업 데이터 저장 중...")
        amr_sync.turn_right(50.0)
        time.sleep(3)
        
        # 6. 최종 정지 (2초)
        print("\n6. 최종 정지 (2초)")
        print("   모터 정지")
        print("   MQTT 연결 없음 - 백업 데이터 저장 중...")
        amr_sync.stop_motor()
        time.sleep(2)
        
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
        
        # 백업 통계 출력
        backup_stats = amr_sync.get_backup_stats()
        print(f"\n💾 백업 통계:")
        print(f"  - 백업 활성화: {'예' if backup_stats['enabled'] else '아니오'}")
        if backup_stats['enabled']:
            print(f"  - 백업 디렉토리: {backup_stats['backup_directory']}")
            print(f"  - 백업 횟수: {backup_stats['backup_count']}")
            print(f"  - 백업 파일 수: {backup_stats['backup_files_count']}")
            print(f"  - 백업 주기: {backup_stats['backup_interval']}초")
            print(f"  - MQTT 연결 상태: {'연결됨' if backup_stats['mqtt_connected'] else '연결 안됨'}")
            print(f"  - 백업 조건: {backup_stats['backup_condition']}")
            print(f"  - MQTT 재연결 시도 횟수: {backup_stats['mqtt_reconnect_attempts']}")
            print(f"  - MQTT 재연결 간격: {backup_stats['mqtt_reconnect_delay']}초")
            print(f"  - MQTT 연결 시도 횟수: {backup_stats['mqtt_connection_attempts']}")
        
        # 백업 파일 목록 출력
        backup_files = amr_sync.list_backup_files()
        if backup_files:
            print(f"\n📁 백업 파일 목록 (최신순):")
            for i, file_info in enumerate(backup_files[:5]):  # 최신 5개만 표시
                file_size_kb = file_info['size'] / 1024
                modified_time = datetime.fromtimestamp(file_info['modified']).strftime("%H:%M:%S")
                print(f"  {i+1}. {file_info['filename']} ({file_size_kb:.1f}KB, {modified_time})")
            
            if len(backup_files) > 5:
                print(f"  ... 외 {len(backup_files) - 5}개 파일")
        
        # 최신 백업 파일 복원 테스트
        if backup_files:
            latest_file = backup_files[0]['filename']
            print(f"\n🔄 최신 백업 파일 복원 테스트: {latest_file}")
            
            restore_result = amr_sync.restore_backup_data(latest_file)
            if restore_result['success']:
                restored_data = restore_result['data']
                print(f"  ✅ 복원 성공:")
                print(f"    - Serial: {restored_data['serial']}")
                print(f"    - Status: {restored_data['status']}")
                print(f"    - Battery: {restored_data['battery_level']}%")
                print(f"    - Position: ({restored_data['x']:.1f}, {restored_data['y']:.1f})")
                print(f"    - Speed: {restored_data['speed']:.1f}")
            else:
                print(f"  ❌ 복원 실패: {restore_result['error']}")
        
        # 백업 데이터 예시 출력
        print(f"\n📋 백업 데이터 구조 예시:")
        sample_backup_data = {
            "serial": "AMR001",
            "state": "RUNNING",
            "x": "10.0",
            "y": "10.0",
            "speed": "25.0"
        }
        print(json.dumps(sample_backup_data, indent=2, ensure_ascii=False))
        
        print(f"\n💡 백업 시스템 동작 방식:")
        print(f"  - MQTT 연결됨: 백업 안함 (데이터는 MQTT로 전송)")
        print(f"  - MQTT 연결 안됨: 5초마다 백업 파일 생성")
        print(f"  - MQTT 재연결: 최대 {backup_stats['mqtt_reconnect_attempts']}회 시도, {backup_stats['mqtt_reconnect_delay']}초 간격")
        print(f"  - MQTT 연결 모니터링: 10초마다 연결 상태 확인")
        print(f"  - 백업 파일 위치: {backup_stats['backup_directory']}/")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
        amr_sync.stop_motor()
    finally:
        # 동기화 중지
        amr_sync.stop_sync()
        print("\n✅ 시스템 정리 완료")

if __name__ == "__main__":
    test_backup_system() 