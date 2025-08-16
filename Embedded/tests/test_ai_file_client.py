import sys
import os
import time
import json
import signal
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

from ai.ai_file_client import AIFileClient

def ai_data_callback(data):
    print(f"AI Data Received:")
    print(f"   Serial: {data.get('serial', 'N/A')}")
    print(f"   Position: ({data.get('x', 0)}, {data.get('y', 0)})")
    print(f"   Image: {'Present' if data.get('img') else 'Not present'}")
    print(f"   Case: {data.get('case', 'N/A')}")
    print(f"   Timestamp: {data.get('timeStamp', 'N/A')}")

def create_test_data(file_path: str):
    test_data = {
        "serial": "AMR001",
        "x": 15.7,
        "y": 25.9,
        "img": "base64_test_image_data",
        "case": "test_situation",
        "timeStamp": "2025-08-06T16:00:00Z"
    }
    
    try:
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(test_data, f, indent=2, ensure_ascii=False)
        print(f"Test data creation completed: {file_path}")
        return True
    except Exception as e:
        print(f"Test data creation failed: {e}")
        return False

def main():
    print("=== File-based AI Data Reception Test ===")
    print("Test for AI system delivering data via JSON files")
    print("Press Ctrl+C to exit.\n")
    
    file_path = "/tmp/ai_data.json"
    ai_client = AIFileClient(file_path, "AMR001")
    ai_client.set_ai_data_callback(ai_data_callback)
    
    ai_client.create_sample_data()
    
    if ai_client.start_monitoring(interval=1.0):
        print("AI file monitoring started successfully")
        print(f"Monitoring AI data file... ({file_path})")
        print("Data will be automatically detected when AI system updates the file.")
        
        try:
            while True:
                time.sleep(1)
                
                if int(time.time()) % 10 == 0:
                    stats = ai_client.get_reception_stats()
                    print("Reception Statistics:")
                    print(f"   Total received: {stats['total_received']}")
                    print(f"   Monitoring: {'Running' if stats['monitoring'] else 'Stopped'}")
                    print(f"   File exists: {'Yes' if stats['file_exists'] else 'No'}")
                    print(f"   File path: {stats['file_path']}")
                    
                    latest_data = stats['latest_ai_data']
                    if latest_data.get('serial'):
                        print(f"   Latest data: {latest_data['serial']} - ({latest_data['x']}, {latest_data['y']})")
                    
        except KeyboardInterrupt:
            print("Shutting down program...")
            
    else:
        print("AI file monitoring start failed")
    
    ai_client.stop_monitoring()

def test_manual_data_update():
    print("=== Manual Data Update Test ===")
    
    file_path = "/tmp/ai_data.json"
    ai_client = AIFileClient(file_path, "AMR001")
    
    ai_client.set_ai_data_callback(ai_data_callback)
    
    print("1. Creating initial data...")
    ai_client.create_sample_data()
    
    print("2. Data reading test...")
    data = ai_client.get_ai_data()
    if data:
        print("Data reading successful")
    else:
        print("Data reading failed")
    
    print("3. Creating new data after 5 seconds...")
    time.sleep(5)
    
    new_data = {
        "serial": "AMR001",
        "x": 30.5,
        "y": 40.7,
        "img": "new_base64_image",
        "case": "updated_situation",
        "timeStamp": "2025-08-06T16:05:00Z"
    }
    
    try:
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(new_data, f, indent=2, ensure_ascii=False)
        print("New data creation completed")
        
        data = ai_client.get_ai_data()
        if data:
            print("New data reading successful")
        else:
            print("New data reading failed")
            
    except Exception as e:
        print(f"New data creation failed: {e}")

if __name__ == "__main__":
    main()
    
    test_manual_data_update() 