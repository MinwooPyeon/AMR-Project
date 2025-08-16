import json
import time
import base64
import paho.mqtt.client as mqtt
from datetime import datetime

class MQTTTester:
    def __init__(self):
        self.embedded_client = mqtt.Client()
        self.ai_client = mqtt.Client()
        
        self.embedded_client.on_connect = self.on_embedded_connect
        self.embedded_client.on_message = self.on_embedded_message
        self.ai_client.on_connect = self.on_ai_connect
        self.ai_client.on_message = self.on_ai_message
        
        self.embedded_data = {}
        self.ai_data = {}
        self.embedded_data = {}
        self.ai_data = {}
        
    def on_embedded_connect(self, client, userdata, flags, rc):
        print(f"Embedded MQTT connection successful: {rc}")
        client.subscribe("robot_data")
        
    def on_ai_connect(self, client, userdata, flags, rc):
        print(f"AI MQTT connection successful: {rc}")
        client.subscribe("ai_data")
        
    def on_embedded_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            print(f"\nEmbedded data received:")
            print(f"  - Serial: {data.get('serial', 'N/A')}")
            print(f"  - State: {data.get('state', 'N/A')}")
            print(f"  - Position: ({data.get('x', 0)}, {data.get('y', 0)})")
            print(f"  - Speed: {data.get('speed', 0)}")
            print(f"  - Angle: {data.get('angle', 0)}")
            self.embedded_data = data
        except Exception as e:
            print(f"Embedded data parsing error: {e}")
            
    def on_ai_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            print(f"\nAI data received:")
            print(f"  - Serial: {data.get('serial', 'N/A')}")
            print(f"  - Position: ({data.get('x', 0)}, {data.get('y', 0)})")
            print(f"  - Image: {len(data.get('img', ''))} bytes")
            print(f"  - Case: {data.get('case', 'N/A')}")
            print(f"  - Timestamp: {data.get('timeStamp', 'N/A')}")
            self.ai_data = data
        except Exception as e:
            print(f"AI data parsing error: {e}")
    
    def connect_embedded(self):
        try:
            self.embedded_client.connect("192.168.100.141", 1883, 60)
            self.embedded_client.loop_start()
            return True
        except Exception as e:
            print(f"Embedded MQTT connection failed: {e}")
            return False
    
    def connect_ai(self):
        try:
            self.ai_client.connect("localhost", 1883, 60)
            self.ai_client.loop_start()
            return True
        except Exception as e:
            print(f"AI MQTT connection failed: {e}")
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
        print(f"Embedded data published: {json_data}")
    
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
        print(f"AI data published: {json_data}")
    
    def disconnect(self):
        self.embedded_client.loop_stop()
        self.embedded_client.disconnect()
        self.ai_client.loop_stop()
        self.ai_client.disconnect()
        print("MQTT connection disconnected")

def main():
    print("=== MQTT Communication Test ===")
    print("Testing embedded and AI MQTT communication.")
    print("=" * 50)
    
    tester = MQTTTester()
    
    print("1. Connecting to embedded MQTT...")
    if not tester.connect_embedded():
        print("Embedded MQTT connection failed")
        return
    
    print("2. Connecting to AI MQTT...")
    if not tester.connect_ai():
        print("AI MQTT connection failed")
        return
    
    print("All MQTT connections successful")
    
    print("3. Publishing test data...")
    time.sleep(2)
    
    print("Embedded data publishing test")
    tester.publish_embedded_data()
    
    time.sleep(2)
    
    print("AI data publishing test")
    tester.publish_ai_data()
    
    print("4. Waiting for data reception... (10 seconds)")
    print("Press Ctrl+C to exit.")
    
    try:
        time.sleep(10)
    except KeyboardInterrupt:
        print("Test interrupted")
    
    print("Test results:")
    if tester.embedded_data:
        print(f"Embedded data: {json.dumps(tester.embedded_data, indent=2, ensure_ascii=False)}")
    else:
        print("Embedded data: Not received")
    
    if tester.ai_data:
        print(f"AI data: {json.dumps(tester.ai_data, indent=2, ensure_ascii=False)}")
    else:
        print("AI data: Not received")
    
    tester.disconnect()
    print("MQTT test completed")

if __name__ == "__main__":
    main() 