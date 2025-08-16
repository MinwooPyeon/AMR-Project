import sys
import os
import time
import json
import signal
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from mqtt.mqtt_manager import MQTTManager
from utils.logger import mqtt_logger

def ai_command_callback(command):
    print(f"AI Command Received:")
    print(f"   Command: {command}")
    
    backend_data = {
        "state": "RUNNING",
        "speed": "25"
    }
    
    if mqtt_manager.send_to_backend(backend_data):
        print("Backend transmission successful")
    else:
        print("Backend transmission failed")

def main():
    print("=== MQTT Manager Integration Test ===")
    print("AI Reception + Backend Transmission Integration Test")
    print("Press Ctrl+C to exit.\n")
    
    global mqtt_manager
    mqtt_manager = MQTTManager("AMR001")
    
    mqtt_manager.set_ai_command_callback(ai_command_callback)
    
    if mqtt_manager.connect_all():
        print("All MQTT connections successful")
        print("Waiting for AI data reception and Backend transmission...")
        
        try:
            while True:
                time.sleep(1)
                
                if int(time.time()) % 10 == 0:
                    stats = mqtt_manager.get_stats()
                    connection_status = mqtt_manager.get_connection_status()
                    
                    print("Integration Statistics:")
                    print(f"   Backend transmission: {stats['backend_sent']}")
                    print(f"   AI reception: {stats['ai_received']}")
                    print(f"   Backend connection: {'Connected' if connection_status['backend_transmitter'] else 'Not connected'}")
                    print(f"   AI connection: {'Connected' if connection_status['ai_client'] else 'Not connected'}")
                    
        except KeyboardInterrupt:
            print("Shutting down program...")
            
    else:
        print("MQTT connection failed")
    
    mqtt_manager.disconnect_all()

if __name__ == "__main__":
    main() 