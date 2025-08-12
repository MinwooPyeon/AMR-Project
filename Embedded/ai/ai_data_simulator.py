#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI 데이터 시뮬레이터
AI 시스템이 JSON 파일로 데이터를 전달하는 시뮬레이션
"""

import json
import time
import random
import os
from datetime import datetime
from typing import Dict

class AIDataSimulator:
    def __init__(self, file_path: str = "/tmp/ai_data.json", robot_id: str = "AMR001"):
        self.file_path = file_path
        self.robot_id = robot_id
        self.running = False
        
        # 시뮬레이션 데이터
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_case = "normal"
        
        print(f"AI 데이터 시뮬레이터 초기화 완료 - 파일: {file_path}")
    
    def start_simulation(self, interval: float = 2.0):
        """AI 데이터 시뮬레이션 시작"""
        self.running = True
        print(f"AI 데이터 시뮬레이션 시작 - 간격: {interval}초")
        
        try:
            while self.running:
                # 새로운 AI 데이터 생성
                ai_data = self.generate_ai_data()
                
                # 파일로 저장
                self.save_ai_data(ai_data)
                
                print(f"📤 AI 데이터 전송: ({ai_data['x']:.1f}, {ai_data['y']:.1f}) - {ai_data['case']}")
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\n⚠️  시뮬레이션 중단됨")
        finally:
            self.running = False
            print("AI 데이터 시뮬레이션 종료")
    
    def stop_simulation(self):
        """AI 데이터 시뮬레이션 중지"""
        self.running = False
    
    def generate_ai_data(self) -> Dict:
        """AI 데이터 생성"""
        # 위치 업데이트 (랜덤 움직임)
        self.current_x += random.uniform(-2.0, 2.0)
        self.current_y += random.uniform(-2.0, 2.0)
        
        # 케이스 업데이트 (랜덤 상황)
        cases = ["normal", "obstacle_detected", "emergency_stop", "path_planning"]
        if random.random() < 0.3:  # 30% 확률로 상황 변경
            self.current_case = random.choice(cases)
        
        # Base64 이미지 시뮬레이션
        image_data = f"base64_simulated_image_{int(time.time())}"
        
        return {
            "serial": self.robot_id,
            "x": round(self.current_x, 2),
            "y": round(self.current_y, 2),
            "img": image_data,
            "case": self.current_case,
            "timeStamp": datetime.now().isoformat()
        }
    
    def save_ai_data(self, data: Dict):
        """AI 데이터를 파일로 저장"""
        try:
            with open(self.file_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"❌ AI 데이터 저장 실패: {e}")
    
    def create_emergency_data(self):
        """긴급 상황 데이터 생성"""
        emergency_data = {
            "serial": self.robot_id,
            "x": self.current_x,
            "y": self.current_y,
            "img": "base64_emergency_image",
            "case": "emergency_stop",
            "timeStamp": datetime.now().isoformat()
        }
        
        self.save_ai_data(emergency_data)
        print("🚨 긴급 상황 데이터 생성!")
    
    def create_obstacle_data(self):
        """장애물 감지 데이터 생성"""
        obstacle_data = {
            "serial": self.robot_id,
            "x": self.current_x,
            "y": self.current_y,
            "img": "base64_obstacle_image",
            "case": "obstacle_detected",
            "timeStamp": datetime.now().isoformat()
        }
        
        self.save_ai_data(obstacle_data)
        print("⚠️  장애물 감지 데이터 생성!")

def main():
    """메인 함수"""
    print("=== AI 데이터 시뮬레이터 ===")
    print("AI 시스템이 JSON 파일로 데이터를 전달하는 시뮬레이션")
    print("종료하려면 Ctrl+C를 누르세요.\n")
    
    simulator = AIDataSimulator("/tmp/ai_data.json", "AMR001")
    
    try:
        # 시뮬레이션 시작
        simulator.start_simulation(interval=2.0)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  시뮬레이션 종료 중...")
        simulator.stop_simulation()

if __name__ == "__main__":
    main() 