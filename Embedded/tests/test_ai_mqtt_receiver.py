import sys
import os
import time
import json
import signal
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from mqtt.ai_mqtt_client import AIMQTTClient
from utils.logger import mqtt_logger

def ai_data_callback(data):
    print(f"AI Data Received:")
    print(f"   Serial: {data.get('serial', 'N/A')}")
    print(f"   Position: ({data.get('x', 0)}, {data.get('y', 0)})")
    print(f"   Image: {'Present' if data.get('img') else 'Not present'}")
    print(f"   Case: {data.get('case', 'N/A')}")
    print(f"   Timestamp: {data.get('timeStamp', 'N/A')}")

def main():
    print("=== AI MQTT Reception Test ===")
    print("Receiving AI data from localhost:1883.")
    print("Press Ctrl+C to exit.\n")
    
    ai_client = AIMQTTClient("AMR001", "localhost", 1883)
    ai_client.set_ai_data_callback(ai_data_callback)
    
    if ai_client.connect_mqtt():
        print("AI MQTT connection successful")
        
        if ai_client.subscribe_to_ai_data("AMR001"):
            print("AI data subscription successful")
            print("Waiting for AI data reception...")
            
            try:
                while True:
                    time.sleep(1)
                    
                    if int(time.time()) % 10 == 0:
                        stats = ai_client.get_reception_stats()
                        print("Reception Statistics:")
                        print(f"   Total received: {stats['total_received']}")
                        print(f"   Connection status: {'Connected' if stats['mqtt_connected'] else 'Not connected'}")
                        
            except KeyboardInterrupt:
                print("Shutting down program...")
                
        else:
            print("AI data subscription failed")
        
        ai_client.disconnect_mqtt()
    else:
        print("AI MQTT connection failed")

if __name__ == "__main__":
    main() 