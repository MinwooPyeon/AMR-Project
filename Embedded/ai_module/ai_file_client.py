import json
import os
import time
import threading
from typing import Dict, Optional, Callable, Any
from datetime import datetime

from .config import AIConfig
from .utils import (
    setup_logger, 
    parse_json_message, 
    validate_ai_data, 
    get_file_modified_time
)

class AIFileClient:
    def __init__(self, file_path: str = None, robot_id: str = None):
        self.file_path = file_path or AIConfig.AI_DATA_FILE_PATH
        self.robot_id = robot_id or AIConfig.DEFAULT_ROBOT_ID
        self.logger = setup_logger("AIFileClient")
        
        self.ai_received_data = {
            "serial": "",
            "x": 0.0,
            "y": 0.0,
            "img": "",
            "case": "",
            "timeStamp": ""
        }
        
        self.data_lock = threading.Lock()
        
        self.ai_data_callback: Optional[Callable[[Dict], None]] = None
        
        self.stats_lock = threading.Lock()
        self.total_received = 0
        self.last_received_time = 0
        self.last_file_modified = 0
        
        self.monitoring = False
        self.monitor_thread = None
        self.monitor_interval = 1.0
        
        self.logger.info(f"AI File Client initialized - file: {self.file_path}")
    
    def get_ai_data(self) -> Optional[Dict]:
        try:
            if not os.path.exists(self.file_path):
                return None
            
            current_modified = get_file_modified_time(self.file_path)
            if current_modified <= self.last_file_modified:
                return None
            
            with open(self.file_path, "r", encoding="utf-8") as f:
                data = parse_json_message(f.read())
            
            if not data:
                return None
            
            if not validate_ai_data(data):
                self.logger.warning("AI data validation failed")
                return None
            
            self.last_file_modified = current_modified
            
            with self.stats_lock:
                self.total_received += 1
                self.last_received_time = time.time()
            
            with self.data_lock:
                required_fields = ["serial", "x", "y", "img", "case", "timeStamp"]
                for field in required_fields:
                    if field in data:
                        if field in ["x", "y"]:
                            self.ai_received_data[field] = float(data[field])
                        else:
                            self.ai_received_data[field] = str(data[field])
            
            return data
            
        except FileNotFoundError:
            return None
        except Exception as e:
            self.logger.error(f"File read failed: {e}")
            return None
    
    def start_monitoring(self, interval: float = 1.0) -> bool:
        if self.monitoring:
            self.logger.warning("Already monitoring")
            return False
        
        self.monitor_interval = interval
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        self.logger.info(f"File monitoring started - interval: {interval}s")
        return True
    
    def stop_monitoring(self):
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        self.logger.info("File monitoring stopped")
    
    def _monitor_loop(self):
        while self.monitoring:
            try:
                data = self.get_ai_data()
                if data and self.ai_data_callback:
                    self.ai_data_callback(data)
                time.sleep(self.monitor_interval)
            except Exception as e:
                self.logger.error(f"Monitoring loop error: {e}")
                time.sleep(self.monitor_interval)
    
    def set_ai_data_callback(self, callback: Callable[[Dict], None]):
        self.ai_data_callback = callback
        self.logger.info("AI data callback function set")
    
    def get_current_data(self) -> Dict:
        with self.data_lock:
            return self.ai_received_data.copy()
    
    def get_statistics(self) -> Dict[str, Any]:
        with self.stats_lock:
            return {
                "total_received": self.total_received,
                "last_received_time": self.last_received_time,
                "monitoring": self.monitoring,
                "file_path": self.file_path,
                "robot_id": self.robot_id
            }
    
    def reset_statistics(self):
        with self.stats_lock:
            self.total_received = 0
            self.last_received_time = 0
        self.logger.info("Statistics reset completed")
    
    def is_file_available(self) -> bool:
        return os.path.exists(self.file_path)
    
    def get_file_info(self) -> Dict[str, Any]:
        try:
            if os.path.exists(self.file_path):
                stat = os.stat(self.file_path)
                return {
                    "exists": True,
                    "size": stat.st_size,
                    "modified_time": stat.st_mtime,
                    "path": self.file_path
                }
            else:
                return {
                    "exists": False,
                    "path": self.file_path
                }
        except Exception as e:
            self.logger.error(f"File info query failed: {e}")
            return {"exists": False, "error": str(e)}

def main():
    print("=== AI File Client Test ===")
    
    def on_ai_data(data):
        print(f"AI data received: {data}")
    
    client = AIFileClient()
    client.set_ai_data_callback(on_ai_data)
    
    try:
        print("File monitoring started... (Exit: Ctrl+C)")
        client.start_monitoring(interval=1.0)
        
        while True:
            time.sleep(5)
            stats = client.get_statistics()
            print(f"Statistics: {stats}")
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped...")
        client.stop_monitoring()

if __name__ == "__main__":
    main() 