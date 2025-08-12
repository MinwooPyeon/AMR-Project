import sys
import os
import json
import time

# 프로젝트 루트를 Python 경로에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from mqtt.sensor_data_transmitter import SensorDataTransmitter
from mqtt.ai_mqtt_client import AIMQTTClient

def test_embedded_to_backend():
    """Embedded -> Backend 연결 테스트"""
    print("=== Embedded -> Backend 연결 테스트 ===")
    
    # Embedded -> Backend 연결 (192.168.100.141:1883)
    transmitter = SensorDataTransmitter("AMR001", "192.168.100.141", 1883)
    
    if transmitter.connect_mqtt():
        print("✅ Embedded -> Backend 연결 성공")
        
        # 테스트 데이터 전송
        test_data = {
            "serial": "AMR001",
            "state": "RUNNING",
            "x": "10.5",
            "y": "20.3",
            "speed": "5.0",
            "angle": "45.0"
        }
        
        transmitter.update_embedded_data(
            serial=test_data["serial"],
            state=test_data["state"],
            x=float(test_data["x"]),
            y=float(test_data["y"]),
            speed=float(test_data["speed"]),
            angle=float(test_data["angle"])
        )
        
        if transmitter.send_embedded_data():
            print("✅ Embedded 데이터 전송 성공")
            print(f"   전송 데이터: {json.dumps(test_data, indent=2)}")
        else:
            print("❌ Embedded 데이터 전송 실패")
        
        # 주기적 전송 테스트
        print("\n=== 주기적 데이터 전송 테스트 ===")
        if transmitter.start_periodic_sending(interval=1.0):
            print("✅ 주기적 전송 시작 성공")
            print("⏳ 5초간 주기적 전송 테스트 중...")
            time.sleep(5)
            
            transmitter.stop_periodic_sending()
            print("✅ 주기적 전송 중지 완료")
        else:
            print("❌ 주기적 전송 시작 실패")
        
        transmitter.disconnect_mqtt()
    else:
        print("❌ Embedded -> Backend 연결 실패")

def test_ai_to_embedded():
    """AI -> Embedded 연결 테스트"""
    print("\n=== AI -> Embedded 연결 테스트 ===")
    
    # AI -> Embedded 연결 (localhost:1883)
    ai_client = AIMQTTClient("AMR001", "localhost", 1883)
    
    if ai_client.connect_mqtt():
        print("✅ AI -> Embedded 연결 성공")
        
        # AI 데이터 구독
        if ai_client.subscribe_to_ai_data():
            print("✅ AI 데이터 구독 성공")
            
            # AI 데이터 콜백 설정
            def ai_data_callback(data):
                print(f"📥 AI 데이터 수신: {json.dumps(data, indent=2)}")
            
            ai_client.set_ai_data_callback(ai_data_callback)
            
            # 5초간 대기하여 데이터 수신 테스트
            print("⏳ 5초간 AI 데이터 수신 대기 중...")
            time.sleep(5)
            
            # 현재 AI 데이터 조회
            current_data = ai_client.get_latest_ai_data()
            print(f"📊 현재 AI 데이터: {json.dumps(current_data, indent=2)}")
            
        else:
            print("❌ AI 데이터 구독 실패")
        
        ai_client.disconnect_mqtt()
    else:
        print("❌ AI -> Embedded 연결 실패")

def test_connection_summary():
    """연결 요약"""
    print("\n=== 연결 설정 요약 ===")
    print("🔗 Embedded -> Backend: 192.168.100.141:1883")
    print("   JSON 형태:")
    print("   - serial: string")
    print("   - state: string")
    print("   - x: float")
    print("   - y: float")
    print("   - speed: float")
    print("   - angle: float")
    
    print("\n🔗 AI -> Embedded: localhost:1883")
    print("   JSON 형태:")
    print("   - serial: string")
    print("   - x: float")
    print("   - y: float")
    print("   - img: Base64")
    print("   - case: string")
    print("   - timeStamp: string")

if __name__ == "__main__":
    test_embedded_to_backend()
    test_ai_to_embedded()
    test_connection_summary() 