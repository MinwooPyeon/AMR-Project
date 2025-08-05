#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MQTT 통신 테스트 스크립트
임베디드 데이터와 AI 데이터 통신을 테스트합니다.
"""

import json
import time
import base64
import paho.mqtt.client as mqtt
from datetime import datetime

class MQTTTester:
    def __init__(self):
        # MQTT 클라이언트 설정
        self.embedded_client = mqtt.Client()
        self.ai_client = mqtt.Client()
        
        # 콜백 설정
        self.embedded_client.on_connect = self.on_embedded_connect
        self.embedded_client.on_message = self.on_embedded_message
        self.ai_client.on_connect = self.on_ai_connect
        self.ai_client.on_message = self.on_ai_message
        
        # 데이터 저장소
        self.embedded_data = {}
        self.ai_data = {}
        
    def on_embedded_connect(self, client, userdata, flags, rc):
        print(f"임베디드 MQTT 연결 성공: {rc}")
        client.subscribe("robot_data")
        
    def on_ai_connect(self, client, userdata, flags, rc):
        print(f"AI MQTT 연결 성공: {rc}")
        client.subscribe("ai_data")
        
    def on_embedded_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            print(f"\n임베디드 데이터 수신:")
            print(f"  - Serial: {data.get('serial', 'N/A')}")
            print(f"  - State: {data.get('state', 'N/A')}")
            print(f"  - Position: ({data.get('x', 0)}, {data.get('y', 0)})")
            print(f"  - Speed: {data.get('speed', 0)}")
            print(f"  - Angle: {data.get('angle', 0)}")
            self.embedded_data = data
        except Exception as e:
            print(f"임베디드 데이터 파싱 오류: {e}")
            
    def on_ai_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            print(f"\nAI 데이터 수신:")
            print(f"  - Serial: {data.get('serial', 'N/A')}")
            print(f"  - Position: ({data.get('x', 0)}, {data.get('y', 0)})")
            print(f"  - Image: {len(data.get('img', ''))} bytes")
            print(f"  - Case: {data.get('case', 'N/A')}")
            print(f"  - Timestamp: {data.get('timeStamp', 'N/A')}")
            self.ai_data = data
        except Exception as e:
            print(f"AI 데이터 파싱 오류: {e}")
    
    def connect_embedded(self):
        try:
            self.embedded_client.connect("192.168.100.141", 1883, 60)
            self.embedded_client.loop_start()
            return True
        except Exception as e:
            print(f"임베디드 MQTT 연결 실패: {e}")
            return False
    
    def connect_ai(self):
        try:
            self.ai_client.connect("localhost", 1883, 60)
            self.ai_client.loop_start()
            return True
        except Exception as e:
            print(f"AI MQTT 연결 실패: {e}")
            return False
    
    def publish_embedded_data(self):
        test_data = {
            "serial": "AMR001",
            "state": "moving",
            "x": 10.5,
            "y": 20.3,
            "speed": 1.5,
            "angle": 45.0
        }
        
        json_data = json.dumps(test_data)
        self.embedded_client.publish("robot_data", json_data)
        print(f"임베디드 데이터 발행: {json_data}")
    
    def publish_ai_data(self):
        test_data = {
            "serial": "AMR001",
            "x": 15.2,
            "y": 8.7,
            "img": base64.b64encode(b"test_image_data_for_ai").decode('utf-8'),
            "case": "normal",
            "timeStamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        
        json_data = json.dumps(test_data)
        self.ai_client.publish("ai_data", json_data)
        print(f"AI 데이터 발행: {json_data}")
    
    def disconnect(self):
        self.embedded_client.loop_stop()
        self.embedded_client.disconnect()
        self.ai_client.loop_stop()
        self.ai_client.disconnect()
        print("MQTT 연결 해제 완료")

def main():
    print("=== MQTT 통신 테스트 ===")
    print("임베디드와 AI MQTT 통신을 테스트합니다.")
    print("=" * 50)
    
    tester = MQTTTester()
    
    print("\n1. 임베디드 MQTT 연결 중...")
    if not tester.connect_embedded():
        print("임베디드 MQTT 연결 실패")
        return
    
    print("\n2. AI MQTT 연결 중...")
    if not tester.connect_ai():
        print("AI MQTT 연결 실패")
        return
    
    print("\n모든 MQTT 연결 성공")
    
    print("\n3. 테스트 데이터 발행...")
    time.sleep(2)
    
    print("\n임베디드 데이터 발행 테스트")
    tester.publish_embedded_data()
    
    time.sleep(2)
    
    print("\nAI 데이터 발행 테스트")
    tester.publish_ai_data()
    
    print("\n4. 데이터 수신 대기 중... (10초)")
    print("Ctrl+C로 종료하세요.")
    
    try:
        time.sleep(10)
    except KeyboardInterrupt:
        print("\n\n테스트 중단됨")
    
    print("\n테스트 결과:")
    if tester.embedded_data:
        print(f"임베디드 데이터: {json.dumps(tester.embedded_data, indent=2, ensure_ascii=False)}")
    else:
        print("임베디드 데이터: 수신되지 않음")
    
    if tester.ai_data:
        print(f"AI 데이터: {json.dumps(tester.ai_data, indent=2, ensure_ascii=False)}")
    else:
        print("AI 데이터: 수신되지 않음")
    
    tester.disconnect()
    print("\nMQTT 테스트 완료")

if __name__ == "__main__":
    main() 