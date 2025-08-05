#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AMR 실시간 데이터 동기화 시스템
"""

import json
import time
import threading
import os
from typing import Dict, Optional, Callable
from datetime import datetime
from utils.logger import main_logger

class AMRRealDataSync:
    def __init__(self, robot_id: str = "AMR001", enable_mqtt: bool = True, enable_backup: bool = True):
        self.robot_id = robot_id
        self.enable_mqtt = enable_mqtt
        self.enable_backup = enable_backup
        
        # 동기화 상태
        self.sync_running = False
        self.sync_thread = None
        self.sync_interval = 1.0  # 1초
        
        # 센서 데이터
        self.sensor_data = {}
        self.data_lock = threading.Lock()
        
        # AI 데이터
        self.ai_position = {"x": 0.0, "y": 0.0}
        self.ai_command = ""
        self.ai_situation = ""
        self.ai_image = ""
        self.position_lock = threading.Lock()
        
        # 모터 제어
        self.motor_controller = None
        self.motor_monitor = None
        
        # MQTT 관련
        self.mqtt_transmitter = None
        self.mqtt_client = None
        
        # 백업 시스템
        self.backup_dir = "backup_data"
        self.backup_files = []
        self.situation_backup_active = False
        self.situation_backup_start = 0
        self.situation_backup_duration = 180  # 3분
        
        # LCD 디스플레이
        self.lcd_controller = None
        
        # AI Subscriber
        self.ai_subscriber = None
        
        # 콜백
        self.data_callback = None
        
        # 통계
        self.stats_lock = threading.Lock()
        self.total_sync_count = 0
        self.last_sync_time = 0
        self.data_loss_count = 0
        
        self._create_backup_directory()
        self._setup_components()
        main_logger.success(f"AMR Real Data Sync 초기화 완료 - Robot ID: {robot_id}")
    
    def _create_backup_directory(self):
        if not os.path.exists(self.backup_dir):
            os.makedirs(self.backup_dir)
            main_logger.info(f"백업 디렉토리 생성: {self.backup_dir}")
    
    def _is_mqtt_connected(self) -> bool:
        if self.mqtt_transmitter:
            return self.mqtt_transmitter.connected
        return False
    
    def _backup_data(self, data: Dict):
        if not self.enable_backup:
            return
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"backup_{timestamp}.json"
            filepath = os.path.join(self.backup_dir, filename)
            
            backup_data = {
                "timestamp": timestamp,
                "robot_id": self.robot_id,
                "data": data
            }
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(backup_data, f, ensure_ascii=False, indent=2)
            
            self.backup_files.append(filename)
            
            # 오래된 백업 파일 정리 (최대 10개 유지)
            if len(self.backup_files) > 10:
                old_file = self.backup_files.pop(0)
                old_filepath = os.path.join(self.backup_dir, old_file)
                if os.path.exists(old_filepath):
                    os.remove(old_filepath)
            
            main_logger.info(f"데이터 백업 완료: {filename}")
            
        except Exception as e:
            main_logger.error(f"백업 오류: {e}")
    
    def get_backup_stats(self) -> Dict:
        return {
            "backup_enabled": self.enable_backup,
            "backup_directory": self.backup_dir,
            "backup_files_count": len(self.backup_files),
            "backup_files": self.backup_files,
            "situation_backup_active": self.situation_backup_active,
            "situation_backup_start": self.situation_backup_start,
            "situation_backup_duration": self.situation_backup_duration
        }
    
    def list_backup_files(self) -> list:
        backup_files = []
        if os.path.exists(self.backup_dir):
            for filename in os.listdir(self.backup_dir):
                if filename.endswith('.json'):
                    filepath = os.path.join(self.backup_dir, filename)
                    file_size = os.path.getsize(filepath)
                    backup_files.append({
                        "filename": filename,
                        "size": file_size,
                        "path": filepath
                    })
        return sorted(backup_files, key=lambda x: x["filename"])
    
    def restore_backup_data(self, filename: str) -> Dict:
        try:
            filepath = os.path.join(self.backup_dir, filename)
            if not os.path.exists(filepath):
                main_logger.error(f"백업 파일이 존재하지 않습니다: {filename}")
                return {}
            
            with open(filepath, 'r', encoding='utf-8') as f:
                backup_data = json.load(f)
            
            main_logger.info(f"백업 데이터 복원 완료: {filename}")
            return backup_data
            
        except Exception as e:
            main_logger.error(f"백업 복원 오류: {e}")
            return {}
    
    def cleanup_old_backups(self, keep_count: int = 10):
        if len(self.backup_files) > keep_count:
            files_to_remove = self.backup_files[:-keep_count]
            for filename in files_to_remove:
                filepath = os.path.join(self.backup_dir, filename)
                if os.path.exists(filepath):
                    os.remove(filepath)
                    self.backup_files.remove(filename)
                    main_logger.info(f"오래된 백업 파일 삭제: {filename}")
    
    def _setup_components(self):
        try:
            # MQTT 센서 설정
            if self.enable_mqtt:
                self._setup_mqtt_sensors()
            
            # 모터 제어 설정
            self._setup_motor_control()
            

            
            # LCD 디스플레이 설정
            self._setup_lcd_display()
            
            # AI Subscriber 설정
            self._setup_ai_subscriber()
            
            # 센서 등록
            self._register_amr_sensors()
            
        except Exception as e:
            main_logger.error(f"컴포넌트 설정 오류: {e}")
    
    def _setup_mqtt_sensors(self):
        try:
            from mqtt.sensor_data_transmitter import SensorDataTransmitter
            self.mqtt_transmitter = SensorDataTransmitter(self.robot_id)
            
            if self._connect_mqtt_with_retry():
                main_logger.success("MQTT 센서 설정 완료")
            else:
                main_logger.warn("MQTT 연결 실패, 백업 모드로 동작")
                
        except ImportError as e:
            main_logger.error(f"MQTT 모듈 import 오류: {e}")
            self.mqtt_transmitter = None
        except Exception as e:
            main_logger.error(f"MQTT 센서 설정 오류: {e}")
            self.mqtt_transmitter = None
    
    def _setup_motor_control(self):
        try:
            from motor_control.real_motor_controller import RealMotorController
            from motor_control.motor_speed_monitor import MotorSpeedMonitor
            
            self.motor_controller = RealMotorController()
            self.motor_monitor = MotorSpeedMonitor()
            main_logger.success("모터 제어 설정 완료")
            
        except ImportError as e:
            main_logger.error(f"모터 제어 모듈 import 오류: {e}")
            self.motor_controller = None
            self.motor_monitor = None
        except Exception as e:
            main_logger.error(f"모터 제어 설정 오류: {e}")
            self.motor_controller = None
            self.motor_monitor = None
    
    def _connect_mqtt_with_retry(self) -> bool:
        max_retries = 3
        for attempt in range(max_retries):
            try:
                if self.mqtt_transmitter.connect_mqtt():
                    return True
                else:
                    main_logger.warn(f"MQTT 연결 시도 {attempt + 1}/{max_retries} 실패")
                    time.sleep(2)
            except Exception as e:
                main_logger.error(f"MQTT 연결 오류: {e}")
                time.sleep(2)
        
        return False
    
    def _connect_mqtt_command_client(self) -> bool:
        try:
            import paho.mqtt.client as mqtt
            
            self.mqtt_client = mqtt.Client(client_id=f"amr_command_client_{self.robot_id}")
            self.mqtt_client.on_connect = self._on_mqtt_connect
            self.mqtt_client.on_message = self._on_mqtt_message
            
            self.mqtt_client.connect("192.168.100.141", 1883, 60)
            self.mqtt_client.loop_start()
            
            self._setup_command_subscription()
            return True
            
        except Exception as e:
            main_logger.error(f"MQTT 명령 클라이언트 연결 오류: {e}")
            return False
    
    def _register_amr_sensors(self):
        try:
            from sensor_sync.sensor_data_sync import SensorDataSync
            
            self.sensor_sync = SensorDataSync()
            main_logger.info("AMR 센서 동기화 초기화 완료")
            
        except ImportError as e:
            main_logger.error(f"센서 동기화 모듈 import 오류: {e}")
            self.sensor_sync = None
        except Exception as e:
            main_logger.error(f"센서 등록 오류: {e}")
            self.sensor_sync = None
    
    def _setup_command_subscription(self):
        if not self.mqtt_client:
            return
        
        topic = "command"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == 0:
            main_logger.info(f"명령 구독 성공: {topic}")
        else:
            main_logger.error(f"명령 구독 실패: {result[0]}")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            main_logger.info("MQTT 명령 수신 클라이언트 연결 성공")
        else:
            main_logger.error(f"MQTT 명령 수신 클라이언트 연결 실패. 코드: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        try:
            import json
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            main_logger.info(f"명령 수신: {data}")
            
            self._process_command(data)
            
        except json.JSONDecodeError as e:
            main_logger.error(f"명령 JSON 파싱 오류: {e}")
        except Exception as e:
            main_logger.error(f"명령 처리 오류: {e}")
    
    def _process_command(self, command: Dict):
        action = command.get("action", "")
        
        if action == "MOVE_FORWARD":
            speed = command.get("speed", 50.0)
            self.move_forward(speed)
            main_logger.info(f"전진 명령 실행 - 속도: {speed}")
            
        elif action == "MOVE_BACKWARD":
            speed = command.get("speed", 50.0)
            self.move_backward(speed)
            main_logger.info(f"후진 명령 실행 - 속도: {speed}")
            
        elif action == "ROTATE_LEFT":
            speed = command.get("speed", 50.0)
            self.turn_left(speed)
            main_logger.info(f"좌회전 명령 실행 - 속도: {speed}")
            
        elif action == "ROTATE_RIGHT":
            speed = command.get("speed", 50.0)
            self.turn_right(speed)
            main_logger.info(f"우회전 명령 실행 - 속도: {speed}")
            
        elif action == "stop_motor":
            self.stop_motor()
            main_logger.info("모터 정지 명령 실행")
            
        elif action == "set_motor_speeds":
            left_speed = command.get("left_speed", 0.0)
            right_speed = command.get("right_speed", 0.0)
            self.set_motor_speeds(left_speed, right_speed)
            main_logger.info(f"모터 속도 설정 명령 실행 - L: {left_speed}, R: {right_speed}")
            
        else:
            main_logger.warning(f"알 수 없는 명령: {action}")
    
    def set_data_callback(self, callback: Callable[[Dict], None]):
        self.data_callback = callback
    
    def update_ai_position(self, x: float, y: float):
        with self.position_lock:
            self.ai_position["x"] = x
            self.ai_position["y"] = y
            main_logger.debug(f"AI 위치 업데이트: x={x:.2f}, y={y:.2f}")
        
        if self.ai_subscriber:
            try:
                import rclpy
                rclpy.spin_once(self.ai_subscriber, timeout_sec=0.001)
            except Exception as e:
                main_logger.debug(f"ROS2 스핀 오류 (무시): {e}")
    
    def get_ai_position(self) -> tuple:
        with self.position_lock:
            return (self.ai_position["x"], self.ai_position["y"])
    
    def start_sync(self):
        if self.sync_running:
            main_logger.warn("이미 동기화가 실행 중입니다")
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._sync_worker, daemon=True)
        self.sync_thread.start()
        
        main_logger.success("AMR 데이터 동기화 시작")
    
    def stop_sync(self):
        if not self.sync_running:
            return
        
        self.sync_running = False
        if self.sync_thread:
            self.sync_thread.join(timeout=5)
        
        main_logger.info("AMR 데이터 동기화 중지")
    
    def _sync_worker(self):
        while self.sync_running:
            try:
                start_time = time.time()
                
                # 실시간 데이터 업데이트
                self._update_real_data()
                
                # MQTT 데이터 전송
                if self.enable_mqtt:
                    self._send_mqtt_data()
                
                # 통계 업데이트
                with self.stats_lock:
                    self.total_sync_count += 1
                    self.last_sync_time = time.time()
                
                # 동기화 간격 대기
                elapsed = time.time() - start_time
                sleep_time = max(0, self.sync_interval - elapsed)
                time.sleep(sleep_time)
                
            except Exception as e:
                main_logger.error(f"동기화 작업 오류: {e}")
                time.sleep(1)
    
    def _attempt_mqtt_reconnection(self):
        if not self.enable_mqtt or not self.mqtt_transmitter:
            return
        
        try:
            if not self.mqtt_transmitter.connected:
                main_logger.info("MQTT 재연결 시도 중...")
                if self.mqtt_transmitter.connect_mqtt():
                    main_logger.success("MQTT 재연결 성공")
                else:
                    main_logger.warn("MQTT 재연결 실패")
        except Exception as e:
            main_logger.error(f"MQTT 재연결 오류: {e}")
    
    def _update_real_data(self):
        try:
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # 기본 센서 데이터
            sensor_data = {
                "timestamp": current_time,
                "robot_id": self.robot_id,
                "sensor_data": {
                    "lidar": {"distance": random.uniform(50, 200)},
                    "camera": {"image": "camera_data.jpg"},
                    "imu": {
                        "acceleration": {"x": random.uniform(-1, 1), "y": random.uniform(-1, 1), "z": random.uniform(9, 11)},
                        "gyroscope": {"x": random.uniform(-0.1, 0.1), "y": random.uniform(-0.1, 0.1), "z": random.uniform(-0.1, 0.1)},
                        "magnetometer": {"x": random.uniform(-50, 50), "y": random.uniform(-50, 50), "z": random.uniform(-50, 50)}
                    }
                }
            }
            
            # 모터 속도 데이터 추가
            if self.motor_controller:
                motor_speeds = self.motor_controller.get_speeds()
                sensor_data["motor_data"] = motor_speeds
            else:
                sensor_data["motor_data"] = {"left_speed": 0.0, "right_speed": 0.0}
            
            # AI 위치 데이터 추가
            ai_position = self.get_ai_position()
            if ai_position[0] is not None and ai_position[1] is not None:
                sensor_data["ai_position"] = {"x": ai_position[0], "y": ai_position[1]}
            
            # AI 명령 데이터 추가
            ai_command = self.get_ai_command()
            if ai_command:
                sensor_data["ai_command"] = ai_command
            
            # AI 상황 데이터 추가
            ai_situation = self.get_ai_situation()
            if ai_situation:
                sensor_data["ai_situation"] = ai_situation
            
            # AI 이미지 데이터 추가
            ai_image = self.get_ai_image()
            if ai_image:
                sensor_data["ai_image"] = ai_image
            
            self.current_data = sensor_data
            
        except Exception as e:
            main_logger.error(f"실시간 데이터 업데이트 오류: {e}")
    
    def _send_mqtt_data(self):
        if not self.mqtt_transmitter:
            return
        
        try:
            motor_speeds = self.motor_controller.get_speeds()
            x, y = self.get_ai_position()
            
            left_speed = abs(motor_speeds.get('left_speed', 0.0))
            right_speed = abs(motor_speeds.get('right_speed', 0.0))
            average_speed = (left_speed + right_speed) / 2.0
            
            json_data = {
                "serial": "AMR001",
                "state": "RUNNING",
                "x": str(x),
                "y": str(y),
                "speed": str(average_speed)
            }
            
            self.mqtt_transmitter.send_sensor_data(json_data)
            
        except Exception as e:
            main_logger.error(f"MQTT 데이터 전송 오류: {e}")
    
    
    def move_forward(self, speed: float = 50.0):
        if not self.motor_controller:
            main_logger.error("모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        success = self.motor_controller.set_speed(speed, speed)
        if success:
            main_logger.info(f"전진 명령 실행: {speed}%")
        return success
    
    def move_backward(self, speed: float = 50.0):
        if not self.motor_controller:
            main_logger.error("모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        success = self.motor_controller.set_speed(-speed, -speed)
        if success:
            main_logger.info(f"후진 명령 실행: {speed}%")
        return success
    
    def turn_left(self, speed: float = 50.0):
        if not self.motor_controller:
            main_logger.error("모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        success = self.motor_controller.set_speed(-speed, speed)
        if success:
            main_logger.info(f"좌회전 명령 실행: {speed}%")
        return success
    
    def turn_right(self, speed: float = 50.0):
        if not self.motor_controller:
            main_logger.error("모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        success = self.motor_controller.set_speed(speed, -speed)
        if success:
            main_logger.info(f"우회전 명령 실행: {speed}%")
        return success
    
    def stop_motor(self):
        if not self.motor_controller:
            main_logger.error("모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        success = self.motor_controller.stop()
        if success:
            main_logger.info("모터 정지 명령 실행")
        return success
    
    def set_motor_speeds(self, left_speed: float, right_speed: float):
        if not self.motor_controller:
            main_logger.error("모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        success = self.motor_controller.set_speed(left_speed, right_speed)
        if success:
            motor_speeds = self.motor_controller.get_speeds()
            actual_left = motor_speeds.get('left_speed', left_speed)
            actual_right = motor_speeds.get('right_speed', right_speed)
            if self.motor_monitor:
                self.motor_monitor.update_motor_speeds(actual_left, actual_right)
            main_logger.info(f"모터 속도 설정 - 실제 속도: L={actual_left:.1f}, R={actual_right:.1f}")
        return success
    

    
    def _setup_lcd_display(self):
        try:
            from display.lcd_display_controller import LCDDisplayController
            self.lcd_display = LCDDisplayController()
            main_logger.success("LCD 디스플레이 설정 완료")
            
        except ImportError as e:
            main_logger.error(f"LCD 디스플레이 모듈 import 오류: {e}")
            self.lcd_display = None
        except Exception as e:
            main_logger.error(f"LCD 디스플레이 설정 오류: {e}")
            self.lcd_display = None
    
    def _setup_ai_subscriber(self):
        try:
            from ros2.ai_position_subscriber import AIPositionSubscriber
            self.ai_subscriber = AIPositionSubscriber()
            main_logger.success("AI Subscriber 설정 완료")
            
        except ImportError as e:
            main_logger.error(f"AI Subscriber 모듈 import 오류: {e}")
            self.ai_subscriber = None
        except Exception as e:
            main_logger.error(f"AI Subscriber 설정 오류: {e}")
            self.ai_subscriber = None
    

    
    def get_motor_speeds(self) -> Dict:
        if self.motor_controller:
            return self.motor_controller.get_speeds()
        return {"left_speed": 0.0, "right_speed": 0.0}
    
    def get_sync_stats(self) -> Dict:
        with self.stats_lock:
            return {
                "total_sync_count": self.total_sync_count,
                "last_sync_time": self.last_sync_time,
                "sync_running": self.sync_running,
                "data_loss_count": self.data_loss_count,
                "motor_status": "active" if self.motor_controller else "inactive"
            }
    
    def process_ai_command(self, ai_data: Dict):
        try:
            command = ai_data.get("command", "")
            situation = ai_data.get("situation", "")
            image = ai_data.get("image", "")
            
            with self.position_lock:
                self.ai_command = command
                self.ai_situation = situation
                self.ai_image = image
            
            # 상황 기반 백업 트리거
            if situation and situation != "normal":
                self._trigger_situation_backup(situation)
            
            # LCD 디스플레이 업데이트
            if self.lcd_controller:
                self.lcd_controller.update_display(situation)
            
            main_logger.info(f"AI 명령 처리: {command}, 상황: {situation}")
            
        except Exception as e:
            main_logger.error(f"AI 명령 처리 오류: {e}")
    
    def _execute_ai_command(self, command: str):
        try:
            if command == "MOVE_FORWARD":
                self.move_forward(50.0)
            elif command == "MOVE_BACKWARD":
                self.move_backward(50.0)
            elif command == "TURN_LEFT":
                self.turn_left(50.0)
            elif command == "TURN_RIGHT":
                self.turn_right(50.0)
            elif command == "STOP":
                self.stop_motor()
            else:
                main_logger.warning(f"알 수 없는 AI 명령: {command}")
                
        except Exception as e:
            main_logger.error(f"AI 명령 실행 오류: {e}")
    
    def get_ai_command(self) -> str:
        with self.position_lock:
            return self.ai_command
    
    def get_ai_situation(self) -> str:
        with self.position_lock:
            return self.ai_situation
    
    def get_ai_image(self) -> str:
        with self.position_lock:
            return self.ai_image
    
    def _trigger_situation_backup(self, situation: str):
        if not self.situation_backup_active:
            self.situation_backup_active = True
            self.situation_backup_start = time.time()
            main_logger.info(f"상황 기반 백업 시작: {situation}")
            
            # 백업 데이터 생성
            backup_data = {
                "situation": situation,
                "timestamp": time.time(),
                "robot_id": self.robot_id,
                "sensor_data": self.sensor_data.copy()
            }
            
            self._backup_data(backup_data)
    
    def get_situation_backup_status(self) -> Dict:
        if self.situation_backup_active:
            elapsed = time.time() - self.situation_backup_start
            remaining = max(0, self.situation_backup_duration - elapsed)
            
            if elapsed >= self.situation_backup_duration:
                self.situation_backup_active = False
                main_logger.info("상황 기반 백업 완료")
            
            return {
                "active": self.situation_backup_active,
                "elapsed": elapsed,
                "remaining": remaining,
                "duration": self.situation_backup_duration
            }
        else:
            return {"active": False, "elapsed": 0, "remaining": 0, "duration": self.situation_backup_duration}
    
    def get_lcd_display_status(self) -> Dict:
        if self.lcd_controller:
            return self.lcd_controller.get_display_status()
        return {"active": False, "current_mode": "unknown"}

def test_amr_real_data_sync():
    print("=== AMR 시스템 초기화 테스트 ===")
    print("=" * 50)
    
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True, enable_backup=True)
    
    print("\n📊 시스템 상태:")
    print(f"  - MQTT 연결: {'연결됨' if amr_sync.mqtt_transmitter else '연결 안됨'}")
    print(f"  - 모터 컨트롤러: {'초기화됨' if amr_sync.motor_controller else '초기화 안됨'}")
    print(f"  - 서보모터 컨트롤러: {'초기화됨' if amr_sync.servo_controller else '초기화 안됨'}")
    print(f"  - LCD 디스플레이: {'초기화됨' if amr_sync.lcd_display else '초기화 안됨'}")
    print(f"  - AI Subscriber: {'초기화됨' if amr_sync.ai_subscriber else '초기화 안됨'}")
    
    print("\n✅ AMR 시스템 초기화 테스트 완료")

if __name__ == "__main__":
    test_amr_real_data_sync() 