#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
실제 AMR 데이터 동기화 시스템
실제 모터 제어와 고정 배터리 값을 연동
"""

import time
import threading
import logging
import json
import os
from datetime import datetime
from typing import Dict, Optional, Callable
from sensor_data_sync import SensorDataSync, SensorType
from motor_speed_monitor import MotorSpeedMonitor, MotorController

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class AMRRealDataSync:
    """실제 AMR 데이터 동기화 클래스"""
    
    def __init__(self, robot_id: str = "AMR001", enable_mqtt: bool = True, enable_backup: bool = True):
        self.robot_id = robot_id
        self.enable_mqtt = enable_mqtt
        self.enable_backup = enable_backup
        
        # MQTT 재연결 관련 설정
        self.mqtt_reconnect_attempts = 2  # 재연결 시도 횟수
        self.mqtt_reconnect_delay = 3.0   # 재연결 간격 (초)
        self.mqtt_connection_attempts = 0  # 현재 시도 횟수
        
        # 백업 관련 설정
        self.backup_dir = "amr_data_backup"
        self.backup_interval = 5.0  # 5초마다 백업
        self.last_backup_time = 0
        self.backup_count = 0
        
        # AI 상황 기반 백업 설정
        self.backup_triggered_by_situation = False
        self.current_ai_situation = ""
        self.situation_backup_duration = 180.0 
        self.situation_backup_start_time = 0
        
        # 백업 디렉토리 생성
        if self.enable_backup:
            self._create_backup_directory()
        
        # 센서 데이터 동기화 시스템
        self.sensor_sync = SensorDataSync(robot_id)
        
        # 모터 속도 모니터
        self.motor_monitor = MotorSpeedMonitor()
        
        # 모터 컨트롤러
        self.motor_controller = MotorController()
        self.motor_monitor.set_motor_controller(self.motor_controller)
        
        # 동기화 상태
        self.sync_running = False
        self.sync_thread = None
        
        # 콜백 함수들
        self.data_callback = None
        
        # AI 위치 데이터 (subscriber에서 받아올 예정)
        self.ai_position = {"x": 0.0, "y": 0.0}
        self.position_lock = threading.Lock()
        
        # AI Position Subscriber (선택적)
        self.ai_subscriber = None
        try:
            from ai_position_subscriber import AIPositionSubscriber
            import rclpy
            rclpy.init()
            self.ai_subscriber = AIPositionSubscriber()
            self.ai_subscriber.set_position_callback(self.update_ai_position)
            self.ai_subscriber.set_ai_data_callback(self.process_ai_command)
            logger.info("AI Position Subscriber 초기화 완료")
        except ImportError as e:
            logger.warning(f"AI Position Subscriber 초기화 실패: {e}")
            logger.info("AI 위치 데이터는 시뮬레이션으로 대체됩니다")
        except Exception as e:
            logger.warning(f"ROS2 초기화 실패: {e}")
            logger.info("AI 위치 데이터는 시뮬레이션으로 대체됩니다")
        
        # MQTT 전송 관련
        self.mqtt_transmitter = None
        self.mqtt_client = None
        if self.enable_mqtt:
            try:
                from sensor_data_transmitter import SensorDataTransmitter
                import paho.mqtt.client as mqtt
                
                # MQTT 전송 시스템
                self.mqtt_transmitter = SensorDataTransmitter(robot_id, "192.168.100.141", 1883)
                self._setup_mqtt_sensors()
                
                # MQTT 클라이언트 (명령 수신용)
                self.mqtt_client = mqtt.Client(client_id=f"amr_command_receiver_{robot_id}")
                self.mqtt_client.on_connect = self._on_mqtt_connect
                self.mqtt_client.on_message = self._on_mqtt_message
                self._setup_command_subscription()
                
                logger.info("MQTT 전송 및 수신 시스템 초기화 완료")
            except ImportError as e:
                logger.warning(f"MQTT 시스템 초기화 실패: {e}")
                self.enable_mqtt = False
        
        # AMR 센서 등록
        self._register_amr_sensors()
        
        logger.info(f"AMR 실제 데이터 동기화 시스템 초기화 완료 - Robot ID: {robot_id}")
        if self.enable_backup:
            logger.info(f"데이터 백업 기능 활성화 - 백업 디렉토리: {self.backup_dir}")
    
    def _create_backup_directory(self):
        """백업 디렉토리 생성"""
        try:
            if not os.path.exists(self.backup_dir):
                os.makedirs(self.backup_dir)
                logger.info(f"백업 디렉토리 생성: {self.backup_dir}")
        except Exception as e:
            logger.error(f"백업 디렉토리 생성 실패: {e}")
            self.enable_backup = False
    
    def _is_mqtt_connected(self) -> bool:
        """MQTT 연결 상태 확인"""
        if not self.enable_mqtt or not self.mqtt_transmitter:
            return False
        
        try:
            # MQTT 클라이언트 연결 상태 확인
            if hasattr(self.mqtt_transmitter, 'mqtt_client') and self.mqtt_transmitter.mqtt_client:
                return self.mqtt_transmitter.mqtt_client.is_connected()
            
            # MQTT 브로커 연결 상태 확인
            if hasattr(self.mqtt_transmitter, 'connected'):
                return self.mqtt_transmitter.connected
            
            return False
        except Exception as e:
            logger.debug(f"MQTT 연결 상태 확인 오류: {e}")
            return False
    
    def _backup_data(self, data: Dict):
        """데이터 백업 - MQTT 연결이 안될 때 또는 AI 상황 발생 시 백업"""
        if not self.enable_backup:
            return
        
        # 백업 조건 확인
        should_backup = False
        backup_reason = ""
        
        # 1. MQTT 연결이 없을 때 백업
        if not self._is_mqtt_connected():
            should_backup = True
            backup_reason = "MQTT 연결 없음"
        
        # 2. AI 상황이 발생했을 때 백업
        elif self.backup_triggered_by_situation:
            current_time = time.time()
            # 상황 발생 후 지정된 시간 동안 백업
            if current_time - self.situation_backup_start_time <= self.situation_backup_duration:
                should_backup = True
                backup_reason = f"AI 상황 발생: {self.current_ai_situation}"
            else:
                # 백업 시간이 지나면 상황 기반 백업 중지
                self.backup_triggered_by_situation = False
                self.current_ai_situation = ""
                logger.info("AI 상황 기반 백업 기간 종료")
        
        if not should_backup:
            return
        
        try:
            current_time = time.time()
            
            # 5초마다 백업
            if current_time - self.last_backup_time >= self.backup_interval:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{self.backup_dir}/amr_data_{timestamp}.json"
                
                # 백엔드 JSON 구조로 데이터 준비
                motor_speeds = self.motor_controller.get_speeds()
                x, y = self.get_ai_position()
                
                # 평균 속도 계산 (좌측/우측 모터의 평균)
                left_speed = abs(motor_speeds.get('left_speed', 0.0))
                right_speed = abs(motor_speeds.get('right_speed', 0.0))
                average_speed = (left_speed + right_speed) / 2.0
                
                # 새로운 백엔드 JSON 구조
                backup_data = {
                    "serial": "AMR001",
                    "state": "RUNNING",
                    "x": str(x),
                    "y": str(y),
                    "speed": str(average_speed)
                }
                
                # 파일에 저장
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(backup_data, f, indent=2, ensure_ascii=False)
                
                self.last_backup_time = current_time
                self.backup_count += 1
                
                logger.info(f"{backup_reason} - 데이터 백업 완료: {filename} (백업 #{self.backup_count})")
                
        except Exception as e:
            logger.error(f"데이터 백업 실패: {e}")
    
    def get_backup_stats(self) -> Dict:
        """백업 통계 조회"""
        if not self.enable_backup:
            return {"enabled": False}
        
        try:
            backup_files = []
            if os.path.exists(self.backup_dir):
                backup_files = [f for f in os.listdir(self.backup_dir) if f.endswith('.json')]
            
            # MQTT 연결 상태 확인
            mqtt_connected = self._is_mqtt_connected()
            
            # 상황 기반 백업 상태 조회
            situation_backup_status = self.get_situation_backup_status()
            
            return {
                "enabled": True,
                "backup_directory": self.backup_dir,
                "backup_count": self.backup_count,
                "backup_files_count": len(backup_files),
                "last_backup_time": self.last_backup_time,
                "backup_interval": self.backup_interval,
                "mqtt_connected": mqtt_connected,
                "backup_condition": "MQTT 연결 없음" if not mqtt_connected else "MQTT 연결됨 (백업 안함)",
                "mqtt_reconnect_attempts": self.mqtt_reconnect_attempts,
                "mqtt_reconnect_delay": self.mqtt_reconnect_delay,
                "mqtt_connection_attempts": self.mqtt_connection_attempts,
                "situation_backup": situation_backup_status
            }
        except Exception as e:
            logger.error(f"백업 통계 조회 실패: {e}")
            return {"enabled": False, "error": str(e)}
    
    def list_backup_files(self) -> list:
        """백업 파일 목록 조회"""
        if not self.enable_backup or not os.path.exists(self.backup_dir):
            return []
        
        try:
            backup_files = []
            for filename in os.listdir(self.backup_dir):
                if filename.endswith('.json'):
                    filepath = os.path.join(self.backup_dir, filename)
                    stat = os.stat(filepath)
                    backup_files.append({
                        "filename": filename,
                        "filepath": filepath,
                        "size": stat.st_size,
                        "modified": stat.st_mtime
                    })
            
            # 수정 시간 기준으로 정렬 (최신순)
            backup_files.sort(key=lambda x: x["modified"], reverse=True)
            return backup_files
        except Exception as e:
            logger.error(f"백업 파일 목록 조회 실패: {e}")
            return []
    
    def restore_backup_data(self, filename: str) -> Dict:
        """백업 데이터 복원 - 새로운 백엔드 JSON 구조"""
        if not self.enable_backup:
            return {"success": False, "error": "백업 기능이 비활성화되어 있습니다"}
        
        try:
            filepath = os.path.join(self.backup_dir, filename)
            if not os.path.exists(filepath):
                return {"success": False, "error": f"백업 파일을 찾을 수 없습니다: {filename}"}
            
            with open(filepath, 'r', encoding='utf-8') as f:
                backup_data = json.load(f)
            
            # 새로운 백엔드 JSON 구조 검증
            required_fields = ["serial", "state", "x", "y", "speed"]
            missing_fields = [field for field in required_fields if field not in backup_data]
            
            if missing_fields:
                return {
                    "success": False, 
                    "error": f"백업 데이터에 필수 필드가 없습니다: {missing_fields}"
                }
            
            logger.info(f"백업 데이터 복원 완료: {filename}")
            logger.info(f"복원된 데이터: serial={backup_data['serial']}, "
                       f"state={backup_data['state']}, "
                       f"position=({backup_data['x']}, {backup_data['y']}), "
                       f"speed={backup_data['speed']}")
            
            return {
                "success": True,
                "data": backup_data,
                "filename": filename
            }
        except Exception as e:
            logger.error(f"백업 데이터 복원 실패: {e}")
            return {"success": False, "error": str(e)}
    
    def cleanup_old_backups(self, keep_count: int = 10):
        """오래된 백업 파일 정리 (최신 파일 keep_count개만 유지)"""
        if not self.enable_backup:
            return
        
        try:
            backup_files = self.list_backup_files()
            if len(backup_files) > keep_count:
                files_to_delete = backup_files[keep_count:]
                deleted_count = 0
                
                for file_info in files_to_delete:
                    try:
                        os.remove(file_info["filepath"])
                        deleted_count += 1
                        logger.info(f"오래된 백업 파일 삭제: {file_info['filename']}")
                    except Exception as e:
                        logger.error(f"백업 파일 삭제 실패: {file_info['filename']} - {e}")
                
                logger.info(f"백업 파일 정리 완료: {deleted_count}개 파일 삭제")
        except Exception as e:
            logger.error(f"백업 파일 정리 실패: {e}")
    
    def _setup_mqtt_sensors(self):
        """MQTT 센서 설정"""
        if not self.mqtt_transmitter:
            return
        
        # AMR 센서 등록
        self.mqtt_transmitter.register_sensor(SensorType.SERIAL, "serial")
        self.mqtt_transmitter.register_sensor(SensorType.STATUS, "status")

        self.mqtt_transmitter.register_sensor(SensorType.POSITION, "position")
        self.mqtt_transmitter.register_sensor(SensorType.SPEED, "speed")
        
        # MQTT 연결 (재연결 시도 포함)
        if self._connect_mqtt_with_retry():
            logger.info("MQTT 브로커 연결 성공")
        else:
            logger.error(f"MQTT 브로커 연결 실패 (최대 {self.mqtt_reconnect_attempts}회 시도)")
            self.enable_mqtt = False
    
    def _connect_mqtt_with_retry(self) -> bool:
        """MQTT 연결 재시도"""
        self.mqtt_connection_attempts = 0
        
        while self.mqtt_connection_attempts <= self.mqtt_reconnect_attempts:
            try:
                if self.mqtt_transmitter.connect_mqtt():
                    logger.info(f"MQTT 브로커 연결 성공 (시도 {self.mqtt_connection_attempts + 1}/{self.mqtt_reconnect_attempts + 1})")
                    return True
                else:
                    self.mqtt_connection_attempts += 1
                    if self.mqtt_connection_attempts <= self.mqtt_reconnect_attempts:
                        logger.warning(f"MQTT 브로커 연결 실패 (시도 {self.mqtt_connection_attempts}/{self.mqtt_reconnect_attempts + 1})")
                        logger.info(f"{self.mqtt_reconnect_delay}초 후 재연결 시도...")
                        time.sleep(self.mqtt_reconnect_delay)
                    else:
                        logger.error(f"MQTT 브로커 연결 최종 실패 (최대 시도 횟수 초과)")
                        return False
            except Exception as e:
                self.mqtt_connection_attempts += 1
                logger.error(f"MQTT 연결 오류 (시도 {self.mqtt_connection_attempts}/{self.mqtt_reconnect_attempts + 1}): {e}")
                if self.mqtt_connection_attempts <= self.mqtt_reconnect_attempts:
                    logger.info(f"{self.mqtt_reconnect_delay}초 후 재연결 시도...")
                    time.sleep(self.mqtt_reconnect_delay)
                else:
                    return False
        
        return False
    
    def _connect_mqtt_command_client(self) -> bool:
        """MQTT 명령 수신 클라이언트 연결 (재연결 시도 포함)"""
        attempts = 0
        max_attempts = self.mqtt_reconnect_attempts
        
        while attempts <= max_attempts:
            try:
                self.mqtt_client.connect("192.168.100.141", 1883, 60)
                self.mqtt_client.loop_start()
                logger.info(f"MQTT 명령 수신 클라이언트 연결 성공 (시도 {attempts + 1}/{max_attempts + 1})")
                return True
            except Exception as e:
                attempts += 1
                logger.error(f"MQTT 명령 수신 클라이언트 연결 실패 (시도 {attempts}/{max_attempts + 1}): {e}")
                if attempts <= max_attempts:
                    logger.info(f"{self.mqtt_reconnect_delay}초 후 재연결 시도...")
                    time.sleep(self.mqtt_reconnect_delay)
                else:
                    logger.error(f"MQTT 명령 수신 클라이언트 연결 최종 실패 (최대 시도 횟수 초과)")
                    return False
        
        return False
    
    def _register_amr_sensors(self):
        """AMR 센서 등록"""
        self.sensor_sync.register_sensor(SensorType.SERIAL, "serial")
        self.sensor_sync.register_sensor(SensorType.STATUS, "status")

        self.sensor_sync.register_sensor(SensorType.POSITION, "position")
        self.sensor_sync.register_sensor(SensorType.SPEED, "speed")
        
        logger.info("AMR 센서 등록 완료")
    
    def _setup_command_subscription(self):
        """명령 구독 설정"""
        if not self.mqtt_client:
            return
        
        # 명령 토픽 구독
        topic = f"command/{self.robot_id}"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == 0:
            logger.info(f"명령 구독 성공: {topic}")
        else:
            logger.error(f"명령 구독 실패: {result[0]}")
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT 연결 콜백"""
        if rc == 0:
            logger.info("MQTT 명령 수신 클라이언트 연결 성공")
        else:
            logger.error(f"MQTT 명령 수신 클라이언트 연결 실패. 코드: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT 메시지 수신 콜백"""
        try:
            import json
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            logger.info(f"명령 수신: {data}")
            
            # 명령 처리
            self._process_command(data)
            
        except json.JSONDecodeError as e:
            logger.error(f"명령 JSON 파싱 오류: {e}")
        except Exception as e:
            logger.error(f"명령 처리 오류: {e}")
    
    def _process_command(self, command: Dict):
        """명령 처리"""
        action = command.get("action", "")
        
        if action == "MOVE_FORWARD":
            speed = command.get("speed", 50.0)
            self.move_forward(speed)
            logger.info(f"전진 명령 실행 - 속도: {speed}")
            
        elif action == "MOVE_BACKWARD":
            speed = command.get("speed", 50.0)
            self.move_backward(speed)
            logger.info(f"후진 명령 실행 - 속도: {speed}")
            
        elif action == "ROTATE_LEFT":
            speed = command.get("speed", 50.0)
            self.turn_left(speed)
            logger.info(f"좌회전 명령 실행 - 속도: {speed}")
            
        elif action == "ROTATE_RIGHT":
            speed = command.get("speed", 50.0)
            self.turn_right(speed)
            logger.info(f"우회전 명령 실행 - 속도: {speed}")
            
        elif action == "stop_motor":
            self.stop_motor()
            logger.info("모터 정지 명령 실행")
            
        elif action == "set_motor_speeds":
            left_speed = command.get("left_speed", 0.0)
            right_speed = command.get("right_speed", 0.0)
            self.set_motor_speeds(left_speed, right_speed)
            logger.info(f"모터 속도 설정 명령 실행 - L: {left_speed}, R: {right_speed}")
            
        else:
            logger.warning(f"알 수 없는 명령: {action}")
    
    def set_data_callback(self, callback: Callable[[Dict], None]):
        """데이터 콜백 설정"""
        self.data_callback = callback
    
    def update_ai_position(self, x: float, y: float):
        """AI에서 받은 위치 데이터 업데이트 (subscriber에서 호출)"""
        with self.position_lock:
            self.ai_position["x"] = x
            self.ai_position["y"] = y
            logger.debug(f"AI 위치 업데이트: x={x:.2f}, y={y:.2f}")
        
        # AI Subscriber가 있으면 ROS2 스핀 실행
        if self.ai_subscriber:
            try:
                import rclpy
                rclpy.spin_once(self.ai_subscriber, timeout_sec=0.001)
            except Exception as e:
                logger.debug(f"ROS2 스핀 오류 (무시): {e}")
    
    def get_ai_position(self) -> tuple:
        """AI에서 받은 위치 좌표 조회"""
        with self.position_lock:
            return self.ai_position["x"], self.ai_position["y"]
    
    def start_sync(self):
        """실제 데이터 동기화 시작"""
        if self.sync_running:
            logger.warning("동기화가 이미 실행 중입니다")
            return
        
        self.sync_running = True
        
        # 센서 동기화 시작
        self.sensor_sync.start_sync()
        
        # 모터 모니터링 시작
        self.motor_monitor.start_monitoring()
        
        # MQTT 전송 시작
        if self.enable_mqtt and self.mqtt_transmitter:
            if self.mqtt_transmitter.start_transmission():
                logger.info("MQTT 전송 시작")
            else:
                logger.error("MQTT 전송 시작 실패")
        
        # MQTT 명령 수신 시작
        if self.enable_mqtt and self.mqtt_client:
            if self._connect_mqtt_command_client():
                logger.info("MQTT 명령 수신 시작")
            else:
                logger.error("MQTT 명령 수신 시작 실패")
        
        # 동기화 스레드 시작
        self.sync_thread = threading.Thread(target=self._sync_worker, daemon=True)
        self.sync_thread.start()
        
        logger.info("실제 AMR 데이터 동기화 시작")
    
    def stop_sync(self):
        """실제 데이터 동기화 중지"""
        if not self.sync_running:
            return
        
        self.sync_running = False
        
        # 센서 동기화 중지
        self.sensor_sync.stop_sync()
        
        # 모터 모니터링 중지
        self.motor_monitor.stop_monitoring()
        
        # MQTT 전송 중지
        if self.enable_mqtt and self.mqtt_transmitter:
            self.mqtt_transmitter.stop_transmission()
            self.mqtt_transmitter.disconnect_mqtt()
        
        # MQTT 클라이언트 중지
        if self.enable_mqtt and self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        logger.info("실제 AMR 데이터 동기화 중지")
    
    def _sync_worker(self):
        """동기화 워커 스레드"""
        mqtt_check_counter = 0  # MQTT 연결 상태 확인 카운터
        
        while self.sync_running:
            try:
                # 실제 데이터 수집 및 업데이트
                self._update_real_data()
                
                # MQTT 연결 상태 주기적 확인 (10초마다)
                mqtt_check_counter += 1
                if mqtt_check_counter >= 10:  # 10초마다 확인
                    mqtt_check_counter = 0
                    if self.enable_mqtt and not self._is_mqtt_connected():
                        logger.warning("MQTT 연결이 끊어졌습니다. 재연결을 시도합니다...")
                        self._attempt_mqtt_reconnection()
                
                # MQTT로 데이터 전송
                mqtt_success = False
                if self.enable_mqtt and self.mqtt_transmitter:
                    try:
                        self._send_mqtt_data()
                        mqtt_success = True
                    except Exception as e:
                        logger.debug(f"MQTT 전송 실패: {e}")
                 
                time.sleep(1.0)  # 1Hz (1초마다)
                
            except Exception as e:
                logger.error(f"동기화 워커 오류: {e}")
                time.sleep(0.1)
    
    def _attempt_mqtt_reconnection(self):
        """MQTT 재연결 시도"""
        if not self.enable_mqtt:
            return
        
        logger.info("MQTT 재연결 시도 중...")
        
        # MQTT 전송 클라이언트 재연결
        if self.mqtt_transmitter:
            try:
                if self._connect_mqtt_with_retry():
                    logger.info("MQTT 전송 클라이언트 재연결 성공")
                else:
                    logger.error("MQTT 전송 클라이언트 재연결 실패")
            except Exception as e:
                logger.error(f"MQTT 전송 클라이언트 재연결 오류: {e}")
        
        # MQTT 명령 수신 클라이언트 재연결
        if self.mqtt_client:
            try:
                # 기존 연결 정리
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
                
                # 재연결 시도
                if self._connect_mqtt_command_client():
                    logger.info("MQTT 명령 수신 클라이언트 재연결 성공")
                else:
                    logger.error("MQTT 명령 수신 클라이언트 재연결 실패")
            except Exception as e:
                logger.error(f"MQTT 명령 수신 클라이언트 재연결 오류: {e}")
    
    def _update_real_data(self):
        """실제 데이터 업데이트"""
        self.sensor_sync.update_serial_data()
        
        self.sensor_sync.update_status_data(status="RUNNING")
        

        
        x, y = self.get_ai_position()
        self.sensor_sync.update_position_data(x=x, y=y)
        
        motor_speeds = self.motor_controller.get_speeds()
        left_speed = motor_speeds.get('left_speed', 0.0)
        right_speed = motor_speeds.get('right_speed', 0.0)
        current_speed = (abs(left_speed) + abs(right_speed)) / 2.0
        
        self.motor_monitor.update_motor_speeds(left_speed, right_speed)
        
        self.sensor_sync.update_speed_data(speed=current_speed)
        
        logger.debug(f"실제 모터 속도 업데이트: L={left_speed:.1f}, R={right_speed:.1f}, 평균={current_speed:.1f}")
    
    def _send_mqtt_data(self):
        """MQTT로 데이터 전송"""
        if not self.mqtt_transmitter:
            return
        
        try:
            # 현재 데이터 가져오기
            motor_speeds = self.motor_controller.get_speeds()
            x, y = self.get_ai_position()
            
            # 평균 속도 계산 (좌측/우측 모터의 평균)
            left_speed = abs(motor_speeds.get('left_speed', 0.0))
            right_speed = abs(motor_speeds.get('right_speed', 0.0))
            average_speed = (left_speed + right_speed) / 2.0
            
            # 새로운 백엔드 JSON 구조로 데이터 전송
            json_data = {
                "serial": "AMR001",
                "state": "RUNNING",
                "x": str(x),
                "y": str(y),
                "speed": str(average_speed)
            }
            
            # MQTT로 JSON 데이터 전송
            self.mqtt_transmitter.publish_json_data(json_data)
            
        except Exception as e:
            logger.error(f"MQTT 데이터 전송 오류: {e}")
    
    
    # 모터 제어 메서드들
    
    def move_forward(self, speed: float = 50.0):
        """전진"""
        success = self.motor_controller.set_speed(speed, speed)
        if success:
            # 실제 모터 상태에서 속도 가져오기
            motor_speeds = self.motor_controller.get_speeds()
            left_speed = motor_speeds.get('left_speed', speed)
            right_speed = motor_speeds.get('right_speed', speed)
            self.motor_monitor.update_motor_speeds(left_speed, right_speed)
            logger.info(f"전진 명령 실행 - 실제 속도: L={left_speed:.1f}, R={right_speed:.1f}")
        return success
    
    def move_backward(self, speed: float = 50.0):
        """후진"""
        success = self.motor_controller.set_speed(-speed, -speed)
        if success:
            # 실제 모터 상태에서 속도 가져오기
            motor_speeds = self.motor_controller.get_speeds()
            left_speed = motor_speeds.get('left_speed', -speed)
            right_speed = motor_speeds.get('right_speed', -speed)
            self.motor_monitor.update_motor_speeds(left_speed, right_speed)
            logger.info(f"후진 명령 실행 - 실제 속도: L={left_speed:.1f}, R={right_speed:.1f}")
        return success
    
    def turn_left(self, speed: float = 50.0):
        """좌회전"""
        left_speed = speed * 0.7
        right_speed = speed
        success = self.motor_controller.set_speed(left_speed, right_speed)
        if success:
            # 실제 모터 상태에서 속도 가져오기
            motor_speeds = self.motor_controller.get_speeds()
            actual_left = motor_speeds.get('left_speed', left_speed)
            actual_right = motor_speeds.get('right_speed', right_speed)
            self.motor_monitor.update_motor_speeds(actual_left, actual_right)
            logger.info(f"좌회전 명령 실행 - 실제 속도: L={actual_left:.1f}, R={actual_right:.1f}")
        return success
    
    def turn_right(self, speed: float = 50.0):
        """우회전"""
        left_speed = speed
        right_speed = speed * 0.7
        success = self.motor_controller.set_speed(left_speed, right_speed)
        if success:
            # 실제 모터 상태에서 속도 가져오기
            motor_speeds = self.motor_controller.get_speeds()
            actual_left = motor_speeds.get('left_speed', left_speed)
            actual_right = motor_speeds.get('right_speed', right_speed)
            self.motor_monitor.update_motor_speeds(actual_left, actual_right)
            logger.info(f"우회전 명령 실행 - 실제 속도: L={actual_left:.1f}, R={actual_right:.1f}")
        return success
    
    def stop_motor(self):
        """모터 정지"""
        success = self.motor_controller.stop()
        if success:
            # 실제 모터 상태에서 속도 가져오기
            motor_speeds = self.motor_controller.get_speeds()
            left_speed = motor_speeds.get('left_speed', 0.0)
            right_speed = motor_speeds.get('right_speed', 0.0)
            self.motor_monitor.update_motor_speeds(left_speed, right_speed)
            logger.info(f"모터 정지 명령 실행 - 실제 속도: L={left_speed:.1f}, R={right_speed:.1f}")
        return success
    
    def set_motor_speeds(self, left_speed: float, right_speed: float):
        """모터 속도 직접 설정"""
        success = self.motor_controller.set_speed(left_speed, right_speed)
        if success:
            # 실제 모터 상태에서 속도 가져오기
            motor_speeds = self.motor_controller.get_speeds()
            actual_left = motor_speeds.get('left_speed', left_speed)
            actual_right = motor_speeds.get('right_speed', right_speed)
            self.motor_monitor.update_motor_speeds(actual_left, actual_right)
            logger.info(f"모터 속도 설정 - 실제 속도: L={actual_left:.1f}, R={actual_right:.1f}")
        return success
    
    # 데이터 조회 메서드들
    

    
    def get_motor_speeds(self) -> Dict:
        """모터 속도 조회"""
        return self.motor_monitor.get_current_speeds()
    
    def get_sync_stats(self) -> Dict:
        """동기화 통계 조회"""
        return {
            "registered_sensors": self.sensor_sync.get_registered_sensor_count(),
            "active_sensors": self.sensor_sync.get_active_sensor_count(),
            "sync_rate": self.sensor_sync.get_sync_rate(),
            "data_loss_rate": self.sensor_sync.get_data_loss_rate(),

            "motor_status": self.get_motor_speeds()['motor_status']
        }

    def process_ai_command(self, ai_data: Dict):
        """AI 명령 데이터 처리"""
        try:
            command = ai_data.get("MOVING_FORWARD") or ai_data.get("ROTATE_LEFT") or \
                     ai_data.get("ROTATE_RIGHT") or ai_data.get("MOVING_BACKWARD") or \
                     ai_data.get("STOP")
            
            if command:
                logger.info(f"AI 명령 수신: {command}")
                self._execute_ai_command(command)
            
            # 상황 정보 로깅 및 백업 트리거
            situation = ai_data.get("situation", "")
            if situation:
                logger.info(f"AI 상황 감지: {situation}")
                self._trigger_situation_backup(situation)
            
            # 이미지 정보 로깅
            img = ai_data.get("img", "")
            if img:
                logger.info(f"AI 이미지: {img}")
                
        except Exception as e:
            logger.error(f"AI 명령 처리 오류: {e}")
    
    def _execute_ai_command(self, command: str):
        """AI 명령 실행"""
        try:
            if command == "MOVING_FORWARD":
                self.move_forward(50.0)
                logger.info("AI 명령 실행: 전진")
            elif command == "MOVING_BACKWARD":
                self.move_backward(50.0)
                logger.info("AI 명령 실행: 후진")
            elif command == "ROTATE_LEFT":
                self.turn_left(50.0)
                logger.info("AI 명령 실행: 좌회전")
            elif command == "ROTATE_RIGHT":
                self.turn_right(50.0)
                logger.info("AI 명령 실행: 우회전")
            elif command == "STOP":
                self.stop_motor()
                logger.info("AI 명령 실행: 정지")
            else:
                logger.warning(f"알 수 없는 AI 명령: {command}")
        except Exception as e:
            logger.error(f"AI 명령 실행 오류: {e}")
    
    def get_ai_command(self) -> str:
        """현재 AI 명령 조회"""
        if self.ai_subscriber:
            return self.ai_subscriber.get_ai_command()
        return "STOP"
    
    def get_ai_situation(self) -> str:
        """현재 AI 상황 조회"""
        if self.ai_subscriber:
            return self.ai_subscriber.get_ai_situation()
        return ""
    
    def get_ai_image(self) -> str:
        """현재 AI 이미지 파일명 조회"""
        if self.ai_subscriber:
            return self.ai_subscriber.get_ai_image()
        return ""
    
    def _trigger_situation_backup(self, situation: str):
        """AI 상황 발생 시 백업 트리거"""
        if not self.enable_backup:
            return
        
        # 새로운 상황이 발생했거나 기존 상황과 다른 경우
        if situation != self.current_ai_situation:
            self.current_ai_situation = situation
            self.backup_triggered_by_situation = True
            self.situation_backup_start_time = time.time()
            
            logger.info(f"AI 상황 기반 백업 트리거: {situation}")
            logger.info(f"백업 기간: {self.situation_backup_duration}초")
        
        # 기존 상황이 계속되는 경우 백업 시간 갱신
        else:
            self.situation_backup_start_time = time.time()
            logger.debug(f"AI 상황 백업 시간 갱신: {situation}")
    
    def get_situation_backup_status(self) -> Dict:
        """상황 기반 백업 상태 조회"""
        if not self.enable_backup:
            return {"enabled": False}
        
        current_time = time.time()
        remaining_time = 0
        
        if self.backup_triggered_by_situation:
            elapsed_time = current_time - self.situation_backup_start_time
            remaining_time = max(0, self.situation_backup_duration - elapsed_time)
        
        return {
            "enabled": self.enable_backup,
            "situation_backup_triggered": self.backup_triggered_by_situation,
            "current_situation": self.current_ai_situation,
            "backup_duration": self.situation_backup_duration,
            "remaining_time": remaining_time,
            "situation_backup_start_time": self.situation_backup_start_time
        }

def test_amr_real_data_sync():
    """실제 AMR 데이터 동기화 테스트"""
    print("=== 실제 AMR 데이터 동기화 테스트 ===")
    print("전진 → 정지 → 좌회전 → 정지 → 우회전 → 정지 순서로 동작합니다.")
    print("센서 데이터가 JSON 형식으로 백엔드(192.168.100.141:1883)로 전송됩니다.")
    print("MQTT 토픽: status/AMR001")
    print("전송 주기: 1초마다 (1Hz)")
    print("백업 기능: MQTT 연결 없을 때 또는 AI 상황 발생 시")
    print("상황 백업 기간: 30초")
    print("=" * 80)
    
    # AMR 실제 데이터 동기화 시스템 생성 (MQTT 활성화)
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True)
    
    # 데이터 콜백 설정 - 센서 데이터를 실시간으로 출력
    def data_callback(data):
        motor_speeds = data['motor_speeds']
        # 평균 속도 계산
        left_speed = abs(motor_speeds['left_speed'])
        right_speed = abs(motor_speeds['right_speed'])
        average_speed = (left_speed + right_speed) / 2.0
        
        mqtt_status = "✅ MQTT" if amr_sync.enable_mqtt else "❌ MQTT"
        print(f"\r실시간 센서 데이터: "
              f"속도 {average_speed:.1f} | "
              f"위치 ({data['position'][0]:.1f}, {data['position'][1]:.1f}) | "
              f"모터 상태: L={motor_speeds['left_speed']:.1f}, R={motor_speeds['right_speed']:.1f} | "
              f"{mqtt_status}", end="")
    
    amr_sync.set_data_callback(data_callback)
    
    # 동기화 시작
    amr_sync.start_sync()
    
    print("\n모터 동작 테스트 시작...")
    print(f"MQTT 전송 상태: {'활성화' if amr_sync.enable_mqtt else '비활성화'}")
    
        # AI 위치 데이터 시뮬레이션 (AI Subscriber가 없을 때만 사용)
    def simulate_ai_position():
        import random
        while True:
            x = 10.0 + random.uniform(-2.0, 2.0)
            y = 20.0 + random.uniform(-2.0, 2.0)
            amr_sync.update_ai_position(x, y)
            time.sleep(0.3)
    
    # AI Subscriber가 없으면 시뮬레이션 스레드 시작
    if not amr_sync.ai_subscriber:
        ai_thread = threading.Thread(target=simulate_ai_position, daemon=True)
        ai_thread.start()
        print("AI 위치 데이터 시뮬레이션 활성화")
    else:
        print("AI Position Subscriber 활성화 - 실제 AI 데이터 사용")
    
    try:
        # 1. 전진 (3초)
        print("\n\n1. 전진 (3초)")
        print("   속도: 50% (좌측/우측 모터)")
        print("   JSON 데이터 전송 중...")
        amr_sync.move_forward(50.0)
        time.sleep(3)
        
        # 2. 정지 (2초)
        print("\n2. 정지 (2초)")
        print("   모터 정지")
        print("   JSON 데이터 전송 중...")
        amr_sync.stop_motor()
        time.sleep(2)
        
        # 3. 좌회전 (3초)
        print("\n3. 좌회전 (3초)")
        print("   속도: 좌측 35%, 우측 50%")
        print("   JSON 데이터 전송 중...")
        amr_sync.turn_left(50.0)
        time.sleep(3)
        
        # 4. 정지 (2초)
        print("\n4. 정지 (2초)")
        print("   모터 정지")
        print("   JSON 데이터 전송 중...")
        amr_sync.stop_motor()
        time.sleep(2)
        
        # 5. 우회전 (3초)
        print("\n5. 우회전 (3초)")
        print("   속도: 좌측 50%, 우측 35%")
        print("   JSON 데이터 전송 중...")
        amr_sync.turn_right(50.0)
        time.sleep(3)
        
        # 6. 최종 정지 (2초)
        print("\n6. 최종 정지 (2초)")
        print("   모터 정지")
        print("   JSON 데이터 전송 중...")
        amr_sync.stop_motor()
        time.sleep(2)
        
        print("\n" + "=" * 80)
        print("=== 테스트 완료 ===")
        print("=" * 80)
        
        # 최종 통계 출력
        stats = amr_sync.get_sync_stats()
        print(f"\n📊 최종 통계:")
        print(f"  - 등록된 센서 수: {stats['registered_sensors']}")
        print(f"  - 활성 센서 수: {stats['active_sensors']}")
        print(f"  - 동기화 속도: {stats['sync_rate']:.2f} Hz (1초마다)")
        print(f"  - 데이터 손실률: {stats['data_loss_rate']:.2f}%")

        print(f"  - 모터 상태: {stats['motor_status']}")
        
        # MQTT 전송 통계
        if amr_sync.enable_mqtt and amr_sync.mqtt_transmitter:
            mqtt_stats = amr_sync.mqtt_transmitter.get_transmission_stats()
            print(f"\n📡 MQTT 전송 통계:")
            for key, value in mqtt_stats.items():
                print(f"  - {key}: {value}")
        
        # 최종 모터 상태 출력
        motor_speeds = amr_sync.get_motor_speeds()
        print(f"\n🔧 최종 모터 상태:")
        print(f"  - 좌측 모터: {motor_speeds['left_speed']:.1f}")
        print(f"  - 우측 모터: {motor_speeds['right_speed']:.1f}")
        print(f"  - 평균 속도: {motor_speeds['current_speed']:.1f}")
        print(f"  - 모터 상태: {motor_speeds['motor_status']}")
        
        # 실제 모터 컨트롤러 상태 확인
        real_speeds = amr_sync.motor_controller.get_speeds()
        print(f"\n⚙️  실제 모터 컨트롤러 상태:")
        print(f"  - 좌측 모터: {real_speeds['left_speed']:.1f}")
        print(f"  - 우측 모터: {real_speeds['right_speed']:.1f}")
        print(f"  - 동작 중: {real_speeds['is_running']}")
        print(f"  - 초기화됨: {real_speeds.get('is_initialized', False)}")
        
        # 백엔드 JSON 데이터 예시 출력
        if amr_sync.enable_mqtt:
            print(f"\n📋 백엔드로 전송되는 JSON 데이터 예시:")
            sample_data = {
                "serial": "AMR001",
                "state": "RUNNING",
                "x": "10.0",
                "y": "20.0",
                "speed": "25.0"
            }
            print(json.dumps(sample_data, indent=2, ensure_ascii=False))
        
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
        amr_sync.stop_motor()
    finally:
        # 동기화 중지
        amr_sync.stop_sync()
        print("\n✅ 시스템 정리 완료")

if __name__ == "__main__":
    test_amr_real_data_sync() 