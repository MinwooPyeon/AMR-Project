import json
import time
import random
import os
from datetime import datetime
from typing import Dict

from .config import AIConfig
from .utils import setup_logger, create_json_message, generate_timestamp

class AIDataSimulator:
    def __init__(self, file_path: str = None, robot_id: str = None):
        self.file_path = file_path or AIConfig.AI_DATA_FILE_PATH
        self.robot_id = robot_id or AIConfig.DEFAULT_ROBOT_ID
        self.running = False
        self.logger = setup_logger("AIDataSimulator")
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_case = "normal"
        
        self.logger.info(f"AI Data Simulator initialized - file: {self.file_path}")
    
    def start_simulation(self, interval: float = 2.0):
        self.running = True
        self.logger.info(f"AI data simulation started - interval: {interval}s")
        
        try:
            while self.running:
                ai_data = self.generate_ai_data()
                
                self.save_ai_data(ai_data)
                
                self.logger.info(f"AI data sent: ({ai_data['x']:.1f}, {ai_data['y']:.1f}) - {ai_data['case']}")
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            self.logger.warning("Simulation interrupted")
        finally:
            self.running = False
            self.logger.info("AI data simulation ended")
    
    def stop_simulation(self):
        self.running = False
    
    def generate_ai_data(self) -> Dict:
        self.current_x += random.uniform(-2.0, 2.0)
        self.current_y += random.uniform(-2.0, 2.0)
        
        cases = ["normal", "obstacle_detected", "emergency_stop", "path_planning"]
        if random.random() < 0.3:
            self.current_case = random.choice(cases)
        
        image_data = f"base64_simulated_image_{int(time.time())}"
        
        return {
            "serial": self.robot_id,
            "x": round(self.current_x, 2),
            "y": round(self.current_y, 2),
            "img": image_data,
            "case": self.current_case,
            "timeStamp": generate_timestamp()
        }
    
    def save_ai_data(self, data: Dict):
        try:
            os.makedirs(os.path.dirname(self.file_path), exist_ok=True)
            
            with open(self.file_path, "w", encoding="utf-8") as f:
                f.write(create_json_message(data))
        except Exception as e:
            self.logger.error(f"AI data save failed: {e}")
    
    def create_emergency_data(self):
        emergency_data = {
            "serial": self.robot_id,
            "x": self.current_x,
            "y": self.current_y,
            "img": "base64_emergency_image",
            "case": "emergency_stop",
            "timeStamp": generate_timestamp()
        }
        
        self.save_ai_data(emergency_data)
        self.logger.warning("Emergency data created!")
    
    def create_obstacle_data(self):
        obstacle_data = {
            "serial": self.robot_id,
            "x": self.current_x,
            "y": self.current_y,
            "img": "base64_obstacle_image",
            "case": "obstacle_detected",
            "timeStamp": generate_timestamp()
        }
        
        self.save_ai_data(obstacle_data)
        self.logger.warning("Obstacle detection data created!")

def main():
    print("=== AI Data Simulator ===")
    print("AI system data simulation via JSON file")
    print("Press Ctrl+C to exit.\n")
    
    simulator = AIDataSimulator()
    
    try:
        simulator.start_simulation(interval=2.0)
        
    except KeyboardInterrupt:
        print("\n\nShutting down simulation...")
        simulator.stop_simulation()

if __name__ == "__main__":
    main() 