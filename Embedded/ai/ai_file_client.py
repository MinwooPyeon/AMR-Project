#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
파일 기반 AI 데이터 수신 클라이언트
AI 시스템이 JSON 파일로 데이터를 전달하는 방식
"""

import json
import os
import time
import threading
from typing import Dict, Optional, Callable, Any
from datetime import datetime

class AIFileClient:
    def __init__(self, file_path: str = "/tmp/ai_data.json", robot_id: str = "AMR001"):
        self.file_path = file_path
        self.robot_id = robot_id
        
        # AI에서 받는 데이터 구조
        self.ai_received_data = {
            "serial": "",
            "x": 0.0,
            "y": 0.0,
            "img": "",
            "case": "",
            "timeStamp": ""
        }
        
        self.data_lock = threading.Lock()
        
        # 콜백 함수들
        self.ai_data_callback: Optional[Callable[[Dict], None]] = None
        
        # 통계
        self.stats_lock = threading.Lock()
        self.total_received = 0
        self.last_received_time = 0
        self.last_file_modified = 0
        
        # 모니터링 관련
        self.monitoring = False
        self.monitor_thread = None
        self.monitor_interval = 1.0  # 1초마다 파일 확인
        
        print(f"AI File Client 초기화 완료 - 파일: {file_path}")
    
    def get_ai_data(self) -> Optional[Dict]:
        """파일에서 AI 데이터 읽기"""
        try:
            # 파일이 존재하는지 확인
            if not os.path.exists(self.file_path):
                return None
            
            # 파일 수정 시간 확인 (변경된 경우만 읽기)
            current_modified = os.path.getmtime(self.file_path)
            if current_modified <= self.last_file_modified:
                return None
            
            # 파일 읽기
            with open(self.file_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            
            # 수정 시간 업데이트
            self.last_file_modified = current_modified
            
            # 통계 업데이트
            with self.stats_lock:
                self.total_received += 1
                self.last_received_time = time.time()
            
            # 데이터 업데이트
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
        except json.JSONDecodeError as e:
            print(f"JSON 파싱 오류: {e}")
            return None
        except Exception as e:
            print(f"파일 읽기 실패: {e}")
            return None
    
    def start_monitoring(self, interval: float = 1.0) -> bool:
        """파일 모니터링 시작"""
        if self.monitoring:
            print("이미 모니터링 중입니다")
            return False
        
        self.monitor_interval = interval
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        print(f"AI 파일 모니터링 시작 - 간격: {interval}초")
        return True
    
    def stop_monitoring(self):
        """파일 모니터링 중지"""
        if not self.monitoring:
            return
        
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        
        print("AI 파일 모니터링 중지")
    
    def _monitor_loop(self):
        """모니터링 루프"""
        while self.monitoring:
            try:
                data = self.get_ai_data()
                if data and self.ai_data_callback:
                    self.ai_data_callback(data)
                
                time.sleep(self.monitor_interval)
            except Exception as e:
                print(f"모니터링 중 오류: {e}")
                time.sleep(self.monitor_interval)
    
    def set_ai_data_callback(self, callback: Callable[[Dict], None]):
        """AI 데이터 콜백 설정"""
        self.ai_data_callback = callback
        print("AI 데이터 콜백 설정 완료")
    
    def get_latest_ai_data(self) -> Dict:
        """최신 AI 데이터 조회"""
        with self.data_lock:
            return self.ai_received_data.copy()
    
    def get_ai_serial(self) -> str:
        """AI 시리얼 조회"""
        with self.data_lock:
            return self.ai_received_data.get("serial", "")
    
    def get_ai_position(self) -> tuple:
        """AI 위치 조회"""
        with self.data_lock:
            x = self.ai_received_data.get("x", 0.0)
            y = self.ai_received_data.get("y", 0.0)
            return (x, y)
    
    def get_ai_image(self) -> str:
        """AI 이미지 조회 (Base64)"""
        with self.data_lock:
            return self.ai_received_data.get("img", "")
    
    def get_ai_case(self) -> str:
        """AI 케이스 조회"""
        with self.data_lock:
            return self.ai_received_data.get("case", "")
    
    def get_ai_timestamp(self) -> str:
        """AI 타임스탬프 조회"""
        with self.data_lock:
            return self.ai_received_data.get("timeStamp", "")
    
    def get_reception_stats(self) -> Dict[str, Any]:
        """수신 통계 조회"""
        with self.stats_lock:
            stats = {
                "total_received": self.total_received,
                "last_received_time": self.last_received_time,
                "monitoring": self.monitoring,
                "latest_ai_data": self.get_latest_ai_data(),
                "file_path": self.file_path,
                "file_exists": os.path.exists(self.file_path)
            }
        return stats
    
    def create_sample_data(self):
        """샘플 AI 데이터 파일 생성 (테스트용)"""
        sample_data = {
            "serial": self.robot_id,
            "x": 10.5,
            "y": 20.3,
            "img": "base64_encoded_image_data",
            "case": "obstacle_detected",
            "timeStamp": datetime.now().isoformat()
        }
        
        try:
            with open(self.file_path, "w", encoding="utf-8") as f:
                json.dump(sample_data, f, indent=2, ensure_ascii=False)
            print(f"✅ 샘플 AI 데이터 파일 생성 완료: {self.file_path}")
            return True
        except Exception as e:
            print(f"❌ 샘플 파일 생성 실패: {e}")
            return False 