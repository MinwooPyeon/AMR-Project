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
            
            # 서보모터 제어 설정
            self._setup_servo_control()
            
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
        except Exception as e:
            main_logger.error(f"MQTT 센서 설정 오류: {e}")
    
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
            from sensor_sync.sensor_data_sync import SensorDataSync, SensorType
            
            self.sensor_sync = SensorDataSync()
            self.sensor_sync.register_sensor(SensorType.POSITION, "position")
            self.sensor_sync.register_sensor(SensorType.SPEED, "speed")
            
            main_logger.info("AMR 센서 등록 완료")
            
        except ImportError as e:
            main_logger.error(f"센서 동기화 모듈 import 오류: {e}")
        except Exception as e:
            main_logger.error(f"센서 등록 오류: {e}")
    
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
            # AI 위치 데이터 가져오기
            x, y = self.get_ai_position()
            
            # 모터 속도 데이터 가져오기
            motor_speeds = {}
            if self.motor_controller:
                motor_speeds = self.motor_controller.get_speeds()
            
            # 센서 데이터 업데이트
            with self.data_lock:
                self.sensor_data.update({
                    "position": {"x": x, "y": y},
                    "motor_speeds": motor_speeds,
                    "timestamp": time.time()
                })
            
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
        success = self.motor_controller.set_speed(speed, speed)
        if success:
            motor_speeds = self.motor_controller.get_speeds()
            left_speed = motor_speeds.get('left_speed', speed)
            right_speed = motor_speeds.get('right_speed', speed)
            self.motor_monitor.update_motor_speeds(left_speed, right_speed)
            main_logger.info(f"전진 명령 실행 - 실제 속도: L={left_speed:.1f}, R={right_speed:.1f}")
        return success
    
    def move_backward(self, speed: float = 50.0):
        success = self.motor_controller.set_speed(-speed, -speed)
        if success:
            motor_speeds = self.motor_controller.get_speeds()
            left_speed = motor_speeds.get('left_speed', -speed)
            right_speed = motor_speeds.get('right_speed', -speed)
            self.motor_monitor.update_motor_speeds(left_speed, right_speed)
            main_logger.info(f"후진 명령 실행 - 실제 속도: L={left_speed:.1f}, R={right_speed:.1f}")
        return success
    
    def turn_left(self, speed: float = 50.0):
        left_speed = speed * 0.7
        right_speed = speed
        success = self.motor_controller.set_speed(left_speed, right_speed)
        if success:
            motor_speeds = self.motor_controller.get_speeds()
            actual_left = motor_speeds.get('left_speed', left_speed)
            actual_right = motor_speeds.get('right_speed', right_speed)
            self.motor_monitor.update_motor_speeds(actual_left, actual_right)
            main_logger.info(f"좌회전 명령 실행 - 실제 속도: L={actual_left:.1f}, R={actual_right:.1f}")
        return success
    
    def turn_right(self, speed: float = 50.0):
        left_speed = speed
        right_speed = speed * 0.7
        success = self.motor_controller.set_speed(left_speed, right_speed)
        if success:
            motor_speeds = self.motor_controller.get_speeds()
            actual_left = motor_speeds.get('left_speed', left_speed)
            actual_right = motor_speeds.get('right_speed', right_speed)
            self.motor_monitor.update_motor_speeds(actual_left, actual_right)
            main_logger.info(f"우회전 명령 실행 - 실제 속도: L={actual_left:.1f}, R={actual_right:.1f}")
        return success
    
    def stop_motor(self):
        success = self.motor_controller.stop()
        if success:
            motor_speeds = self.motor_controller.get_speeds()
            left_speed = motor_speeds.get('left_speed', 0.0)
            right_speed = motor_speeds.get('right_speed', 0.0)
            self.motor_monitor.update_motor_speeds(left_speed, right_speed)
            main_logger.info(f"모터 정지 명령 실행 - 실제 속도: L={left_speed:.1f}, R={right_speed:.1f}")
        return success
    
    def set_motor_speeds(self, left_speed: float, right_speed: float):
        success = self.motor_controller.set_speed(left_speed, right_speed)
        if success:
            motor_speeds = self.motor_controller.get_speeds()
            actual_left = motor_speeds.get('left_speed', left_speed)
            actual_right = motor_speeds.get('right_speed', right_speed)
            self.motor_monitor.update_motor_speeds(actual_left, actual_right)
            main_logger.info(f"모터 속도 설정 - 실제 속도: L={actual_left:.1f}, R={actual_right:.1f}")
        return success
    
    def _setup_servo_control(self):
        try:
            from motor_control.servo_motor_controller import ServoMotorController
            self.servo_controller = ServoMotorController()
            main_logger.success("서보모터 컨트롤러 설정 완료")
            
        except ImportError as e:
            main_logger.error(f"서보모터 컨트롤러 모듈 import 오류: {e}")
            self.servo_controller = None
        except Exception as e:
            main_logger.error(f"서보모터 컨트롤러 설정 오류: {e}")
            self.servo_controller = None
    
    def set_servo_angle(self, servo_name: str, angle: float) -> bool:
        if not self.servo_controller:
            main_logger.error("서보모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        return self.servo_controller.set_servo_angle(servo_name, angle)
    
    def set_all_servos(self, angle: float) -> bool:
        if not self.servo_controller:
            main_logger.error("서보모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        return self.servo_controller.set_all_servos(angle)
    
    def set_servo_angles(self, angles: Dict[str, float]) -> bool:
        if not self.servo_controller:
            main_logger.error("서보모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        return self.servo_controller.set_servo_angles(angles)
    
    def get_servo_angle(self, servo_name: str) -> Optional[float]:
        if not self.servo_controller:
            return None
        
        return self.servo_controller.get_servo_angle(servo_name)
    
    def get_all_servo_angles(self) -> Dict[str, float]:
        if not self.servo_controller:
            return {}
        
        return self.servo_controller.get_all_angles()
    
    def reset_all_servos(self) -> bool:
        if not self.servo_controller:
            main_logger.error("서보모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        return self.servo_controller.reset_all_servos()
    
    def sweep_servo(self, servo_name: str, start_angle: float = 0, end_angle: float = 180, 
                   step: float = 5, delay: float = 0.1) -> bool:
        if not self.servo_controller:
            main_logger.error("서보모터 컨트롤러가 초기화되지 않았습니다")
            return False
        
        return self.servo_controller.sweep_servo(servo_name, start_angle, end_angle, step, delay)
    
    def get_servo_status(self) -> Dict:
        if not self.servo_controller:
            return {"initialized": False, "error": "서보모터 컨트롤러가 초기화되지 않음"}
        
        return self.servo_controller.get_status()
    
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
    print("=== AMR 실시간 데이터 동기화 테스트 ===")
    print("전진 → 정지 → 좌회전 → 정지 → 우회전 → 정지 순서로 동작합니다.")
    print("센서 데이터가 JSON 형식으로 백엔드(192.168.100.141:1883)로 전송됩니다.")
    print("MQTT 토픽: status")
    print("전송 주기: 1초마다 (1Hz)")
    print("백업 기능: MQTT 연결 없을 때 또는 AI 상황 발생 시")
    print("=" * 60)
    
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True, enable_backup=True)
    
    def data_callback(data):
        print(f"\r🤖 센서 데이터: "
              f"위치=({data.get('x', 0):.1f}, {data.get('y', 0):.1f}) | "
              f"속도={data.get('speed', 0):.1f} | "
              f"상태={data.get('state', 'N/A')}", end="")
    
    amr_sync.set_data_callback(data_callback)
    
    def simulate_ai_position():
        import math
        import time
        
        radius = 5.0
        center_x, center_y = 10.0, 10.0
        angle = 0
        
        while amr_sync.sync_running:
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            amr_sync.update_ai_position(x, y)
            
            angle += 0.1
            time.sleep(1)
    
    amr_sync.start_sync()
    
    try:
        print("\n1. 전진 (5초)")
        amr_sync.move_forward(50.0)
        time.sleep(5)
        
        print("\n2. 정지 (3초)")
        amr_sync.stop_motor()
        time.sleep(3)
        
        print("\n3. 좌회전 (5초)")
        amr_sync.turn_left(50.0)
        time.sleep(5)
        
        print("\n4. 정지 (3초)")
        amr_sync.stop_motor()
        time.sleep(3)
        
        print("\n5. 우회전 (5초)")
        amr_sync.turn_right(50.0)
        time.sleep(5)
        
        print("\n6. 최종 정지 (3초)")
        amr_sync.stop_motor()
        time.sleep(3)
        
        print("\n" + "=" * 60)
        print("=== 테스트 완료 ===")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
        amr_sync.stop_motor()
    
    stats = amr_sync.get_sync_stats()
    print(f"\n📊 동기화 통계:")
    for key, value in stats.items():
        print(f"  - {key}: {value}")
    
    backup_stats = amr_sync.get_backup_stats()
    print(f"\n💾 백업 통계:")
    for key, value in backup_stats.items():
        if key != "backup_files":
            print(f"  - {key}: {value}")
    
    amr_sync.stop_sync()
    print("\n✅ AMR 실시간 데이터 동기화 테스트 완료")

if __name__ == "__main__":
    test_amr_real_data_sync() 