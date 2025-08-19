#!/usr/bin/env python3
"""
AI 데이터 처리 및 명령 해석 시스템
실시간 AI 명령 처리, 이미지 분석, 상황 인식 기능
"""

import os
import sys
import time
import json
import logging
import threading
import numpy as np
from typing import Dict, List, Optional, Callable, Any
from pathlib import Path
import cv2
from PIL import Image
import base64
import io

# 프로젝트 루트 추가
sys.path.append(str(Path(__file__).parent.parent))
from config.system_config import get_config
from security.security_manager import get_security_manager

class AIProcessor:
    """AI 데이터 처리 및 명령 해석 클래스"""
    
    def __init__(self):
        self.config = get_config()
        self.security_manager = get_security_manager()
        self.logger = self._setup_logger()
        
        # AI 설정
        self.ai_config = {
            'model_path': 'ai_module/models/',
            'confidence_threshold': 0.7,
            'max_detection_count': 10,
            'image_processing_enabled': True,
            'object_detection_enabled': True,
            'path_planning_enabled': True,
            'voice_recognition_enabled': False,
            'gesture_recognition_enabled': False
        }
        
        # AI 상태
        self.is_processing = False
        self.current_task = None
        self.detected_objects = []
        self.path_waypoints = []
        self.emergency_stop = False
        
        # 콜백 함수들
        self.command_callbacks = {}
        self.detection_callbacks = []
        self.status_callbacks = []
        
        # 스레드 안전을 위한 락
        self.lock = threading.Lock()
        
        # 초기화
        self._initialize_ai()
    
    def _setup_logger(self) -> logging.Logger:
        """로거 설정"""
        logger = logging.getLogger('ai_processor')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(name)s - %(levelname)s: %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        
        return logger
    
    def _initialize_ai(self):
        """AI 시스템 초기화"""
        try:
            # 모델 디렉토리 생성
            model_dir = Path(self.ai_config['model_path'])
            model_dir.mkdir(parents=True, exist_ok=True)
            
            # AI 모델 로드
            self._load_ai_models()
            
            # AI 처리 스레드 시작
            self._start_ai_processing()
            
            self.logger.info("AI processor initialized successfully")
            
        except Exception as e:
            self.logger.error(f"AI initialization failed: {e}")
            raise
    
    def _load_ai_models(self):
        """AI 모델 로드"""
        try:
            # 객체 감지 모델 로드
            if self.ai_config['object_detection_enabled']:
                self._load_object_detection_model()
            
            # 경로 계획 모델 로드
            if self.ai_config['path_planning_enabled']:
                self._load_path_planning_model()
            
            # 음성 인식 모델 로드
            if self.ai_config['voice_recognition_enabled']:
                self._load_voice_recognition_model()
            
            # 제스처 인식 모델 로드
            if self.ai_config['gesture_recognition_enabled']:
                self._load_gesture_recognition_model()
                
        except Exception as e:
            self.logger.error(f"Failed to load AI models: {e}")
    
    def _load_object_detection_model(self):
        """객체 감지 모델 로드"""
        # 실제 구현에서는 YOLO, TensorFlow, PyTorch 등 사용
        self.logger.info("Object detection model loaded")
    
    def _load_path_planning_model(self):
        """경로 계획 모델 로드"""
        # 실제 구현에서는 A*, RRT, D* 등 사용
        self.logger.info("Path planning model loaded")
    
    def _load_voice_recognition_model(self):
        """음성 인식 모델 로드"""
        # 실제 구현에서는 Whisper, SpeechRecognition 등 사용
        self.logger.info("Voice recognition model loaded")
    
    def _load_gesture_recognition_model(self):
        """제스처 인식 모델 로드"""
        # 실제 구현에서는 MediaPipe, OpenPose 등 사용
        self.logger.info("Gesture recognition model loaded")
    
    def _start_ai_processing(self):
        """AI 처리 스레드 시작"""
        processing_thread = threading.Thread(
            target=self._ai_processing_loop,
            daemon=True
        )
        processing_thread.start()
    
    def _ai_processing_loop(self):
        """AI 처리 루프"""
        while True:
            try:
                if not self.emergency_stop:
                    # AI 처리 작업 수행
                    self._process_ai_tasks()
                
                time.sleep(0.1)  # 100ms 간격
                
            except Exception as e:
                self.logger.error(f"AI processing error: {e}")
                time.sleep(1)
    
    def _process_ai_tasks(self):
        """AI 처리 작업 수행"""
        with self.lock:
            if self.current_task:
                self._execute_task(self.current_task)
    
    def process_image(self, image_data: str) -> Dict[str, Any]:
        """이미지 처리 및 객체 감지"""
        try:
            # Base64 이미지 디코딩
            image_bytes = base64.b64decode(image_data)
            image = Image.open(io.BytesIO(image_bytes))
            
            # OpenCV 형식으로 변환
            cv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
            
            # 객체 감지 수행
            detections = self._detect_objects(cv_image)
            
            # 결과 반환
            result = {
                'timestamp': time.time(),
                'detections': detections,
                'image_processed': True,
                'confidence': self._calculate_confidence(detections)
            }
            
            self.logger.info(f"Image processed: {len(detections)} objects detected")
            return result
            
        except Exception as e:
            self.logger.error(f"Image processing failed: {e}")
            return {
                'timestamp': time.time(),
                'detections': [],
                'image_processed': False,
                'error': str(e)
            }
    
    def _detect_objects(self, image) -> List[Dict]:
        """객체 감지 수행"""
        detections = []
        
        try:
            # 실제 구현에서는 YOLO, TensorFlow 등 사용
            # 여기서는 시뮬레이션된 객체 감지
            
            # 예시 객체들
            sample_objects = [
                {'class': 'person', 'confidence': 0.85, 'bbox': [100, 100, 200, 300]},
                {'class': 'chair', 'confidence': 0.72, 'bbox': [300, 200, 400, 350]},
                {'class': 'table', 'confidence': 0.68, 'bbox': [500, 150, 700, 400]}
            ]
            
            for obj in sample_objects:
                if obj['confidence'] >= self.ai_config['confidence_threshold']:
                    detections.append(obj)
            
            # 최대 감지 개수 제한
            detections = detections[:self.ai_config['max_detection_count']]
            
        except Exception as e:
            self.logger.error(f"Object detection failed: {e}")
        
        return detections
    
    def _calculate_confidence(self, detections: List[Dict]) -> float:
        """전체 신뢰도 계산"""
        if not detections:
            return 0.0
        
        total_confidence = sum(d['confidence'] for d in detections)
        return total_confidence / len(detections)
    
    def plan_path(self, start_point: List[float], end_point: List[float], 
                  obstacles: List[Dict] = None) -> Dict[str, Any]:
        """경로 계획"""
        try:
            # 실제 구현에서는 A*, RRT, D* 등 사용
            # 여기서는 간단한 직선 경로 생성
            
            waypoints = self._generate_waypoints(start_point, end_point, obstacles)
            
            result = {
                'timestamp': time.time(),
                'start_point': start_point,
                'end_point': end_point,
                'waypoints': waypoints,
                'path_length': self._calculate_path_length(waypoints),
                'estimated_time': self._estimate_travel_time(waypoints)
            }
            
            self.logger.info(f"Path planned: {len(waypoints)} waypoints")
            return result
            
        except Exception as e:
            self.logger.error(f"Path planning failed: {e}")
            return {
                'timestamp': time.time(),
                'error': str(e),
                'waypoints': []
            }
    
    def _generate_waypoints(self, start: List[float], end: List[float], 
                           obstacles: List[Dict] = None) -> List[List[float]]:
        """경유점 생성"""
        waypoints = [start]
        
        # 간단한 직선 경로 생성
        steps = 10
        for i in range(1, steps):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])
            waypoints.append([x, y])
        
        waypoints.append(end)
        return waypoints
    
    def _calculate_path_length(self, waypoints: List[List[float]]) -> float:
        """경로 길이 계산"""
        if len(waypoints) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            total_length += distance
        
        return total_length
    
    def _estimate_travel_time(self, waypoints: List[List[float]]) -> float:
        """이동 시간 추정"""
        path_length = self._calculate_path_length(waypoints)
        # 평균 속도 1m/s 가정
        return path_length / 1.0
    
    def process_command(self, command: str) -> Dict[str, Any]:
        """AI 명령 처리"""
        try:
            # 명령 파싱
            parsed_command = self._parse_command(command)
            
            # 명령 유효성 검사
            if not self._validate_command(parsed_command):
                return {
                    'timestamp': time.time(),
                    'command': command,
                    'valid': False,
                    'error': 'Invalid command'
                }
            
            # 명령 실행
            result = self._execute_command(parsed_command)
            
            return {
                'timestamp': time.time(),
                'command': command,
                'parsed': parsed_command,
                'valid': True,
                'result': result
            }
            
        except Exception as e:
            self.logger.error(f"Command processing failed: {e}")
            return {
                'timestamp': time.time(),
                'command': command,
                'valid': False,
                'error': str(e)
            }
    
    def _parse_command(self, command: str) -> Dict[str, Any]:
        """명령 파싱"""
        # 간단한 명령 파싱 (실제로는 NLP 사용)
        command = command.lower().strip()
        
        if 'move' in command:
            if 'forward' in command:
                return {'action': 'move', 'direction': 'forward'}
            elif 'backward' in command:
                return {'action': 'move', 'direction': 'backward'}
            elif 'left' in command:
                return {'action': 'move', 'direction': 'left'}
            elif 'right' in command:
                return {'action': 'move', 'direction': 'right'}
        elif 'stop' in command:
            return {'action': 'stop'}
        elif 'turn' in command:
            if 'left' in command:
                return {'action': 'turn', 'direction': 'left'}
            elif 'right' in command:
                return {'action': 'turn', 'direction': 'right'}
        elif 'detect' in command:
            return {'action': 'detect', 'target': 'objects'}
        elif 'plan' in command and 'path' in command:
            return {'action': 'plan_path'}
        
        return {'action': 'unknown', 'raw_command': command}
    
    def _validate_command(self, parsed_command: Dict[str, Any]) -> bool:
        """명령 유효성 검사"""
        valid_actions = ['move', 'stop', 'turn', 'detect', 'plan_path']
        return parsed_command.get('action') in valid_actions
    
    def _execute_command(self, parsed_command: Dict[str, Any]) -> Dict[str, Any]:
        """명령 실행"""
        action = parsed_command.get('action')
        
        if action == 'move':
            return self._execute_move_command(parsed_command)
        elif action == 'stop':
            return self._execute_stop_command()
        elif action == 'turn':
            return self._execute_turn_command(parsed_command)
        elif action == 'detect':
            return self._execute_detect_command()
        elif action == 'plan_path':
            return self._execute_path_planning_command()
        
        return {'status': 'unknown_action'}
    
    def _execute_move_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """이동 명령 실행"""
        direction = command.get('direction', 'forward')
        
        # 모터 제어 명령 생성
        motor_command = {
            'action': 'move',
            'direction': direction,
            'speed': 50,  # 기본 속도
            'duration': 1.0  # 1초
        }
        
        # 콜백 함수 호출
        if 'move' in self.command_callbacks:
            self.command_callbacks['move'](motor_command)
        
        return {
            'status': 'executing',
            'motor_command': motor_command
        }
    
    def _execute_stop_command(self) -> Dict[str, Any]:
        """정지 명령 실행"""
        motor_command = {
            'action': 'stop',
            'emergency': False
        }
        
        if 'stop' in self.command_callbacks:
            self.command_callbacks['stop'](motor_command)
        
        return {
            'status': 'stopped',
            'motor_command': motor_command
        }
    
    def _execute_turn_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """회전 명령 실행"""
        direction = command.get('direction', 'left')
        angle = 90 if direction == 'left' else -90
        
        motor_command = {
            'action': 'turn',
            'direction': direction,
            'angle': angle,
            'speed': 30
        }
        
        if 'turn' in self.command_callbacks:
            self.command_callbacks['turn'](motor_command)
        
        return {
            'status': 'turning',
            'motor_command': motor_command
        }
    
    def _execute_detect_command(self) -> Dict[str, Any]:
        """감지 명령 실행"""
        # 객체 감지 수행
        # 실제로는 카메라 이미지를 받아서 처리
        detections = [
            {'class': 'person', 'confidence': 0.85},
            {'class': 'chair', 'confidence': 0.72}
        ]
        
        return {
            'status': 'detected',
            'objects': detections,
            'count': len(detections)
        }
    
    def _execute_path_planning_command(self) -> Dict[str, Any]:
        """경로 계획 명령 실행"""
        # 현재 위치에서 목표 지점까지 경로 계획
        start_point = [0.0, 0.0]  # 현재 위치
        end_point = [5.0, 5.0]    # 목표 위치
        
        path_result = self.plan_path(start_point, end_point)
        
        return {
            'status': 'planned',
            'path': path_result
        }
    
    def _execute_task(self, task: Dict[str, Any]):
        """작업 실행"""
        task_type = task.get('type')
        
        if task_type == 'image_processing':
            self._process_image_task(task)
        elif task_type == 'path_planning':
            self._process_path_planning_task(task)
        elif task_type == 'object_detection':
            self._process_object_detection_task(task)
    
    def _process_image_task(self, task: Dict[str, Any]):
        """이미지 처리 작업"""
        image_data = task.get('image_data')
        if image_data:
            result = self.process_image(image_data)
            task['result'] = result
            task['completed'] = True
    
    def _process_path_planning_task(self, task: Dict[str, Any]):
        """경로 계획 작업"""
        start_point = task.get('start_point', [0.0, 0.0])
        end_point = task.get('end_point', [5.0, 5.0])
        
        result = self.plan_path(start_point, end_point)
        task['result'] = result
        task['completed'] = True
    
    def _process_object_detection_task(self, task: Dict[str, Any]):
        """객체 감지 작업"""
        image_data = task.get('image_data')
        if image_data:
            result = self.process_image(image_data)
            task['result'] = result
            task['completed'] = True
    
    def add_command_callback(self, command_type: str, callback: Callable):
        """명령 콜백 함수 추가"""
        self.command_callbacks[command_type] = callback
    
    def add_detection_callback(self, callback: Callable):
        """감지 콜백 함수 추가"""
        self.detection_callbacks.append(callback)
    
    def add_status_callback(self, callback: Callable):
        """상태 콜백 함수 추가"""
        self.status_callbacks.append(callback)
    
    def set_emergency_stop(self, stop: bool):
        """비상 정지 설정"""
        self.emergency_stop = stop
        if stop:
            self.logger.warning("Emergency stop activated")
        else:
            self.logger.info("Emergency stop deactivated")
    
    def get_ai_status(self) -> Dict[str, Any]:
        """AI 상태 반환"""
        return {
            'is_processing': self.is_processing,
            'current_task': self.current_task,
            'detected_objects_count': len(self.detected_objects),
            'emergency_stop': self.emergency_stop,
            'models_loaded': {
                'object_detection': self.ai_config['object_detection_enabled'],
                'path_planning': self.ai_config['path_planning_enabled'],
                'voice_recognition': self.ai_config['voice_recognition_enabled'],
                'gesture_recognition': self.ai_config['gesture_recognition_enabled']
            }
        }
    
    def update_config(self, new_config: Dict[str, Any]):
        """AI 설정 업데이트"""
        with self.lock:
            self.ai_config.update(new_config)
            self.logger.info("AI configuration updated")

# 전역 AI 프로세서 인스턴스
ai_processor = AIProcessor()

def get_ai_processor() -> AIProcessor:
    """AI 프로세서 인스턴스 반환"""
    return ai_processor

if __name__ == "__main__":
    # AI 프로세서 테스트
    processor = get_ai_processor()
    
    print("=== AI Processor Test ===")
    print(f"AI Status: {processor.get_ai_status()}")
    
    # 명령 처리 테스트
    print("\nCommand Processing Test:")
    commands = [
        "move forward",
        "turn left",
        "stop",
        "detect objects",
        "plan path"
    ]
    
    for command in commands:
        result = processor.process_command(command)
        print(f"Command: {command}")
        print(f"Result: {result}")
        print()
    
    # 이미지 처리 테스트 (시뮬레이션)
    print("Image Processing Test:")
    # 실제로는 카메라에서 이미지를 받아서 처리
    print("Image processing simulation completed")
    
    print("\nAI Processor test completed!")
