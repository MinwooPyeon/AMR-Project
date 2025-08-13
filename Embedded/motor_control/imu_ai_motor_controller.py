#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU + AI 통합 모터 제어 시스템
GY-BN008x IMU를 이용한 정확한 각도 제어와 AI 주행 기능 통합
"""

import time
import math
import smbus
import json
import base64
import requests
import sys
import os
import threading
from datetime import datetime

# PCA9685 모듈 import (현재 디렉터리에서)
from PCA9685 import PCA9685

# 프로젝트 루트 경로 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from mqtt.sensor_data_transmitter import SensorDataTransmitter

# 명령 테이블 정의
COMMAND_TABLE = {
    0: 'STOP',
    1: 'MOVING_FORWARD', 
    2: 'MOVING_BACKWARD',
    3: 'ROTATE_LEFT',
    4: 'ROTATE_RIGHT'
}

class IMUAIMotorController:
    
    # 모터 방향 정의
    FORWARD = 'forward'
    BACKWARD = 'backward'
    
    def __init__(self, i2c_address=0x40, i2c_bus=1, debug=True, api_url=None, 
                 backend_broker="192.168.100.141", backend_port=1883):
        """
        Args:
            i2c_address (int): I2C 주소 (기본값: 0x40)
            i2c_bus (int): I2C 버스 번호 (기본값: 1)
            debug (bool): 디버그 모드 (기본값: True)
            api_url (str): AI API URL (기본값: None)
            backend_broker (str): Backend MQTT 브로커 주소 (기본값: "192.168.100.141")
            backend_port (int): Backend MQTT 포트 (기본값: 1883)
        """
        self.debug = debug
        self.i2c_address = i2c_address
        self.i2c_bus = i2c_bus
        self.api_url = api_url
        self.backend_broker = backend_broker
        self.backend_port = backend_port
        
        # 환경 변수로 IMU 버스/주소 강제 설정 가능
        try:
            env_bus = os.getenv('IMU_BUS')
            if env_bus is not None:
                self.imu_bus = int(env_bus)
        except Exception:
            pass
        try:
            env_addr = os.getenv('IMU_ADDR')
            if env_addr is not None:
                self.imu_address = int(env_addr, 16) if isinstance(env_addr, str) else int(env_addr)
        except Exception:
            pass

        # 서보 모터 주소 설정
        self.servo_address = 0x60  # 서보 모터용 PCA9685 주소
        # 서보 3채널 구성 (MG996R): 50Hz, 퍼센트/펄스 기반 제어
        self.servo_channels = [0, 1, 2]
        self.servo_center_us = 1500  # 중립(정지/중앙)
        self.servo_min_us = 1000     # 안전 최소 펄스
        self.servo_max_us = 2000     # 안전 최대 펄스 (MG996R은 2400us까지도 동작 가능, 필요 시 조정)
        self.servo_trim_us = {0: 0, 1: 0, 2: 0}  # 채널별 트림(미세 오프셋)
        # 90도 펄스 기반 제어 설정
        self.use_servo_pulse_turn = True
        self.steering_servo_channel = 0
        self.servo_turn_hold_time = 0.4  # 초, 서보가 목표 위치에 도달할 시간
        self.servo_up_us = 2000     # UP 명령 시 펄스 (90도 위치)
        self.servo_down_us = 1000   # DOWN 명령 시 펄스 (90도 아래 위치)
        self.servo_is_up = False    # 현재 서보 상태 (UP/DOWN)
        
        # 모터 핀 정의
        self.PWMA = 0  # 모터 A PWM
        self.AIN1 = 1  # 모터 A 방향 1
        self.AIN2 = 2  # 모터 A 방향 2
        self.PWMB = 5  # 모터 B PWM
        self.BIN1 = 3  # 모터 B 방향 1
        self.BIN2 = 4  # 모터 B 방향 2
        
        # 모터 상태
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        self.motor_a_direction = self.FORWARD
        self.motor_b_direction = self.FORWARD
        
        # AI 통신 상태
        self.last_ai_data = None
        self.serial_number = "AMR001"  # 기본 시리얼 번호
        
        # Backend 통신
        self.backend_transmitter = None
        self.backend_connected = False
        
        # IMU 관련 변수
        self.imu_bus = 1  # 기본 IMU 버스: 1
        self.imu_address = 0x4B  # 기본값: BNO08x ADR=HIGH
        self.imu_type = 'AUTO'   # AUTO | MPU6050 | BNO08X
        self.bno = None          # BNO08x 인스턴스 (Adafruit 라이브러리)
        self.initial_angle = 0.0
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.angle_offset = 0.0  # 각도 오프셋 (정중앙 맞추기용)
        self.is_turning = False
        self.turn_direction = 0  # 1: 좌회전, -1: 우회전
        self.imu_available = False  # IMU 사용 가능 여부
        
        # PID 제어 변수
        self.kp = 2.0  # 비례 게인
        self.ki = 0.1  # 적분 게인
        self.kd = 0.5  # 미분 게인
        self.prev_error = 0.0
        self.integral = 0.0
        
        # 모터 속도 설정 (무거운 무게 대응 - 높은 토크)
        self.motor_speeds = {
            'forward': 300,    # 전진 속도 (300% - 절대 최대 모드)
            'backward': 300,   # 후진 속도 (300% - 절대 최대 모드)
            'left': 300,       # 좌회전 속도 (300% - 절대 최대 모드)
            'right': 300,      # 우회전 속도 (300% - 절대 최대 모드)
            'stop': 0,         # 정지 속도
            'custom': 300      # 커스텀 기본 속도 (300% - 절대 최대 모드)
        }
        # 주행 부스트 설정: 절대 최대 모드
        self.boost_speed = 300    # 부스트 속도 (300% - 절대 최대 모드)
        self.cruise_speed = 300   # 크루즈 속도 (300% - 절대 최대 모드)
        self.move_boost_duration = 5.0  # 부스트 시간 5.0초로 극대화 (극한 짐 옮기기용)
        self.move_boost_timer = None
        self.current_motion = 'stop'  # 'stop' | 'forward' | 'backward' | 'turn' | 'custom'
        
        # 스레드 제어
        self.control_thread = None
        self.running = False
        self.thread_lock = threading.Lock()
        
        try:
            # PCA9685 모터 드라이버 초기화
            self.pwm = PCA9685(i2c_address, debug=debug)
            self.pwm.setPWMFreq(20)  # 20Hz PWM 주파수 (안정 모드)
            
            # PCA9685 서보 모터 드라이버 초기화 (선택사항)
            try:
                self.servo_pwm = PCA9685(self.servo_address, debug=debug)
                self.servo_pwm.setPWMFreq(50)  # 50Hz PWM 주파수 (서보용)
                if self.debug:
                    print(f"   서보 모터 드라이버 초기화 성공 - 주소: 0x{self.servo_address:02X}")
            except Exception as e:
                if self.debug:
                    print(f"   서보 모터 드라이버 초기화 실패: {e}")
                self.servo_pwm = None
            
            # IMU 초기화 (실패해도 계속 진행)
            self.initialize_imu()
            
            # 모든 모터 정지
            self.stop_all()
            
            if self.debug:
                imu_addr_str = f"0x{self.imu_address:02X}" if isinstance(self.imu_address, int) else "N/A"
                print(f"   모터 드라이버 주소: 0x{i2c_address:02X}")
                print(f"   서보 드라이버 주소: 0x{self.servo_address:02X}")
                print(f"   I2C 버스(모터): {self.i2c_bus}")
                print(f"   I2C 버스(IMU): {self.imu_bus if self.imu_bus is not None else 'N/A'}")
                print(f"   PWM 주파수: 20Hz (모터), 50Hz (서보)")
                print(f"   IMU 주소: {imu_addr_str}")
                print(f"   IMU 사용 가능: {self.imu_available}")
                if api_url:
                    print(f"   AI API URL: {api_url}")
                print(f"   Backend MQTT: {backend_broker}:{backend_port}")
                print(f"   PID 게인: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
                print(f"   하드코딩된 모터 속도: {self.motor_speeds}")
                
        except Exception as e:
            print(f"IMU AI 모터 컨트롤러 초기화 실패: {e}")
            raise

    def initialize_imu(self):
        """IMU 초기화 및 캘리브레이션"""
        try:
            # I2C 버스 자동 감지 (이미 설정된 경우 유지)
            if self.imu_bus is None:
                self.imu_bus = self._find_imu_bus()
                if self.imu_bus is None:
                    raise Exception("사용 가능한 I2C 버스를 찾을 수 없습니다")
            
            # IMU 주소 자동 감지
            self.imu_address = self._find_imu_address()
            if self.imu_address is None:
                raise Exception("IMU 센서를 찾을 수 없습니다")
            
            # 센서 타입에 따른 초기화
            if self.imu_type == 'BNO08X':
                # BNO08x: Adafruit CircuitPython 라이브러리로 간단 연동
                try:
                    # 버스 번호 지정이 필요하므로 ExtendedI2C 우선 사용
                    from adafruit_extended_bus import ExtendedI2C  # pip3 install adafruit-extended-bus
                    from adafruit_bno08x.i2c import BNO08X_I2C
                    from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR
                    i2c = ExtendedI2C(self.imu_bus)
                    self.bno = BNO08X_I2C(i2c, address=self.imu_address)
                    # 20Hz = 50,000us (안정성을 위해 더 느리게)
                    self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, report_interval_us=50000)
                    self.imu_available = True
                    if self.debug:
                        print("BNO08x 초기화 성공 (게임 회전벡터 활성화, 20Hz)")
                except Exception as e:
                    print(f"BNO08x 초기화 실패: {e}")
                    print("pip3 install adafruit-circuitpython-bno08x adafruit-blinka adafruit-extended-bus 로 설치 후 재시도하세요.")
                    self.imu_available = False
            else:
                # MPU6050 초기화
                self.imu_bus.write_byte_data(self.imu_address, 0x6B, 0)  # PWR_MGMT_1 레지스터
            
            # 초기 각도 캘리브레이션
            print("IMU 캘리브레이션 중...")
            angles = []
            for i in range(100):
                angle = self.read_imu_yaw() if self.imu_available else 0.0
                angles.append(angle)
                time.sleep(0.01)
            
            # 초기 각도 평균 계산
            raw_initial_angle = sum(angles) / len(angles)
            
            # 정중앙(0도) 기준으로 설정
            self.initial_angle = 0.0
            self.current_angle = 0.0
            self.target_angle = 0.0
            self.angle_offset = raw_initial_angle  # 오프셋 저장
            
            # BNO08x는 현재 각도 읽기가 없으므로 IMU 사용 불가로 둠
            if self.imu_type == 'BNO08X':
                self.imu_available = False
            else:
                self.imu_available = True
            
            if self.debug:
                print(f"IMU 초기화 완료")
                print(f"  - I2C 버스: {self.imu_bus}")
                print(f"  - IMU 주소: 0x{self.imu_address:02X}")
                print(f"  - IMU 타입: {self.imu_type}")
                print(f"  - 원시 초기 각도: {raw_initial_angle:.2f}도")
                print(f"  - 보정된 초기 각도: {self.initial_angle:.2f}도 (정중앙)")
                print(f"  - 각도 오프셋: {self.angle_offset:.2f}도")
                
        except Exception as e:
            print(f"IMU 초기화 실패: {e}")
            print("IMU 없이 모터 제어만 사용합니다.")
            self.imu_available = False

    def _find_imu_bus(self):
        """사용 가능한 I2C 버스 찾기"""
        import os
        import subprocess
        # 우선 버스 1이 존재하면 그대로 사용 (일반적 기본 버스)
        if os.path.exists("/dev/i2c-1"):
            return 1
        # 1) i2cdetect 결과로 BNO08x(0x4A/0x4B)가 보이는 버스 우선 선택
        for bus_num in range(10):
            bus_path = f"/dev/i2c-{bus_num}"
            if not os.path.exists(bus_path):
                continue
            try:
                out = subprocess.run(["i2cdetect", "-y", "-r", str(bus_num)], capture_output=True, text=True, timeout=3)
                txt = out.stdout.lower()
                if " 4b" in txt or " 4a" in txt:
                    if self.debug:
                        print(f"사용 가능한 I2C 버스 발견: {bus_num} (i2cdetect에 BNO08x 표시)")
                    return bus_num
            except Exception:
                pass
        # 2) 그 외에는 첫 번째로 열리는 버스를 후보로
        candidate_bus = None
        for bus_num in range(10):
            bus_path = f"/dev/i2c-{bus_num}"
            if not os.path.exists(bus_path):
                continue
            try:
                bus = smbus.SMBus(bus_num)
                bus.close()
                candidate_bus = bus_num
                if self.debug:
                    print(f"사용 가능한 I2C 버스 발견: {bus_num}")
                break
            except Exception:
                continue
        return candidate_bus

    def _find_imu_address(self):
        """IMU 센서 주소 찾기 (BNO08x 우선, 그 다음 MPU6050)"""
        import subprocess
        bus = None
        try:
            bus = smbus.SMBus(self.imu_bus)
            # 0) i2cdetect 결과로 주소 우선 파악
            try:
                out = subprocess.run(["i2cdetect", "-y", "-r", str(self.imu_bus)], capture_output=True, text=True, timeout=3)
                txt = out.stdout.lower()
                if " 4b" in txt:
                    self.imu_type = 'BNO08X'
                    return 0x4B
                if " 4a" in txt:
                    self.imu_type = 'BNO08X'
                    return 0x4A
            except Exception:
                pass
            # 1) BNO08x 후보 먼저 확인
            for addr in [0x4B, 0x4A]:
                try:
                    bus.read_byte_data(addr, 0x00)
                    if self.debug:
                        print(f"BNO08x 추정 장치 발견: 0x{addr:02X}")
                    self.imu_type = 'BNO08X'
                    return addr
                except Exception:
                    continue
            # 2) MPU6050 후보 확인
            for addr in [0x68, 0x69]:
                try:
                    bus.write_byte_data(addr, 0x6B, 0)
                    time.sleep(0.05)
                    who_am_i = bus.read_byte_data(addr, 0x75)
                    if who_am_i == 0x68:
                        if self.debug:
                            print(f"MPU6050 발견: 0x{addr:02X}")
                        self.imu_type = 'MPU6050'
                        return addr
                except Exception:
                    continue
        except Exception as e:
            if self.debug:
                print(f"IMU 주소 탐지 중 오류: {e}")
        finally:
            try:
                if bus is not None:
                    bus.close()
            except Exception:
                pass
        return None

    def read_imu_yaw(self):
        """IMU에서 Yaw 각도 읽기 (정중앙 기준)"""
        if not self.imu_available or self.imu_bus is None:
            return 0.0
            
        try:
            # BNO08x 우선
            if self.imu_type == 'BNO08X' and self.bno is not None:
                # 게임 회전벡터(쿼터니언) 읽기 → yaw 추출
                q = self.bno.quaternion  # (w, x, y, z)
                if q is None:
                    # 연결이 흔들릴 수 있으므로 1회 재초기화 후 재시도
                    if self._reinit_bno08x():
                        q = self.bno.quaternion
                        if q is None:
                            return 0.0
                    else:
                        return 0.0
                w, x, y, z = q
                # yaw = atan2(2(wz+xy), 1-2(y^2+z^2)) (Tait-Bryan ZYX)
                siny_cosp = 2.0 * (w * z + x * y)
                cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
                yaw_rad = math.atan2(siny_cosp, cosy_cosp)
                yaw_deg = yaw_rad * 180.0 / math.pi
                self.current_angle = self.normalize_angle(yaw_deg)
                corrected_angle = self.current_angle - self.angle_offset
                return self.normalize_angle(corrected_angle)

            # MPU6050 경로(이전 로직 유지)
            bus = smbus.SMBus(self.imu_bus)
            gyro_x = bus.read_word_data(self.imu_address, 0x43)
            gyro_y = bus.read_word_data(self.imu_address, 0x45)
            gyro_z = bus.read_word_data(self.imu_address, 0x47)
            gyro_x = self.convert_to_signed(gyro_x)
            gyro_y = self.convert_to_signed(gyro_y)
            gyro_z = self.convert_to_signed(gyro_z)
            gyro_scale = 250.0 / 32768.0
            yaw_rate = gyro_z * gyro_scale
            self.current_angle += yaw_rate * 0.01
            self.current_angle = self.normalize_angle(self.current_angle)
            corrected_angle = self.current_angle - self.angle_offset
            result = self.normalize_angle(corrected_angle)
            bus.close()
            return result
            
        except Exception as e:
            if self.debug:
                print(f"IMU 읽기 오류: {e}")
            return 0.0

    def _reinit_bno08x(self):
        """BNO08x를 재초기화하고 성공 여부 반환"""
        try:
            # 기존 연결 정리
            if hasattr(self, 'bno') and self.bno is not None:
                try:
                    del self.bno
                except:
                    pass
            
            # 잠시 대기 후 재초기화
            time.sleep(0.1)
            
            from adafruit_extended_bus import ExtendedI2C
            from adafruit_bno08x.i2c import BNO08X_I2C
            from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR
            i2c = ExtendedI2C(self.imu_bus)
            self.bno = BNO08X_I2C(i2c, address=self.imu_address)
            
            # 더 느린 속도로 재설정 (안정성 향상)
            self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, report_interval_us=50000)  # 20Hz로 낮춤
            
            if self.debug:
                print("BNO08x 재초기화 완료 (20Hz)")
            return True
        except Exception as e:
            if self.debug:
                print(f"BNO08x 재초기화 실패: {e}")
            self.bno = None
            return False

    def center_robot(self):
        """로봇을 정중앙(0도)으로 맞추기"""
        if self.is_turning:
            if self.debug:
                print("이미 회전 중입니다. 정중앙 맞추기를 건너뜁니다.")
            return False
            
        current_angle = self.read_imu_yaw()
        angle_error = abs(current_angle)
        
        if angle_error < 1.0:  # 1도 이내면 이미 중앙에 있음
            if self.debug:
                print(f"로봇이 이미 정중앙에 있습니다. (각도: {current_angle:.2f}도)")
            return True
            
        if self.debug:
            print(f"정중앙 맞추기 시작 - 현재 각도: {current_angle:.2f}도")
        
        # 0도로 회전
        self.turn_to_angle(0.0)
        return True

    def get_center_status(self):
        """정중앙 상태 확인"""
        current_angle = self.read_imu_yaw()
        is_centered = abs(current_angle) < 1.0
        
        status = {
            "current_angle": current_angle,
            "is_centered": is_centered,
            "angle_offset": self.angle_offset,
            "initial_angle": self.initial_angle
        }
        
        if self.debug:
            print(f"정중앙 상태: {status}")
            
        return status

    def convert_to_signed(self, value):
        """16비트 값을 부호 있는 정수로 변환"""
        if value > 32767:
            value -= 65536
        return value

    def normalize_angle(self, angle):
        """각도를 -180 ~ 180도 범위로 정규화"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    def calculate_angle_error(self, target, current):
        """목표 각도와 현재 각도 간의 최단 거리 오차 계산"""
        error = target - current
        
        # 최단 거리 계산
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        return error

    def pid_control(self, error):
        """PID 제어 계산"""
        with self.thread_lock:
            # 적분 항
            self.integral += error
            
            # 미분 항
            derivative = error - self.prev_error
            
            # PID 출력
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            
            # 이전 오차 저장
            self.prev_error = error
            
            # 출력 제한
            output = max(-100, min(100, output))
            
            return output

    def start_control_loop(self):
        """IMU 제어 루프 시작"""
        if self.control_thread and self.control_thread.is_alive():
            return
            
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        if self.debug:
            print("IMU 제어 루프 시작")

    def stop_control_loop(self):
        """IMU 제어 루프 정지"""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        
        if self.debug:
            print("IMU 제어 루프 정지")

    def control_loop(self):
        """IMU 기반 모터 제어 루프"""
        while self.running:
            try:
                if self.is_turning:
                    # 현재 각도 읽기
                    current_angle = self.read_imu_yaw()
                    
                    # 각도 오차 계산
                    error = self.calculate_angle_error(self.target_angle, current_angle)
                    
                    # PID 제어
                    motor_speed = self.pid_control(error)
                    
                    # 회전 방향에 따른 모터 제어 (좌우 방향 수정)
                    if abs(error) > 1.0:  # 1도 이상 오차가 있을 때만 회전
                        if self.turn_direction == 1:  # 좌회전 (방향 수정)
                            self.differential_drive(abs(motor_speed), -abs(motor_speed))
                        elif self.turn_direction == -1:  # 우회전 (방향 수정)
                            self.differential_drive(-abs(motor_speed), abs(motor_speed))
                    else:
                        # 목표 각도에 도달
                        self.stop_all()
                        self.is_turning = False
                        if self.debug:
                            print(f"목표 각도 도달: {current_angle:.2f}도")
                
                time.sleep(0.02)  # 20ms 간격 (50Hz)
                
            except Exception as e:
                if self.debug:
                    print(f"제어 루프 오류: {e}")
                time.sleep(0.1)

    def turn_left_90(self):
        """90도 좌회전"""
        if self.is_turning:
            if self.debug:
                print("이미 회전 중입니다.")
            return False
            
        if self.use_servo_pulse_turn and hasattr(self, 'servo_pwm') and self.servo_pwm is not None:
            # 서보를 90도 올렸다가(최대 펄스) 다시 90도 내리는(중립 복귀) 동작
            return self._servo_pulse_90_cycle()

        if self.imu_available:
            # IMU 기반 정확한 회전
            self.target_angle = self.normalize_angle(self.current_angle + 90)
            self.turn_direction = 1
            self.is_turning = True
            self.integral = 0.0  # 적분 항 리셋
            
            if self.debug:
                print(f"90도 좌회전 시작 (IMU 기반): 현재 {self.current_angle:.2f}도 -> 목표 {self.target_angle:.2f}도")
        else:
            # IMU 없이 시간 기반 회전
            if self.debug:
                print("90도 좌회전 시작 (시간 기반)")
            
            # 좌회전: 왼쪽 모터 후진, 오른쪽 모터 전진
            self.differential_drive(-40, 40)
            time.sleep(1.5)  # 약 1.5초 회전 (실제 테스트로 조정 필요)
            self.stop_all()
            
            if self.debug:
                print("90도 좌회전 완료 (시간 기반)")
        
        return True

    def turn_right_90(self):
        """90도 우회전"""
        if self.is_turning:
            if self.debug:
                print("이미 회전 중입니다.")
            return False
            
        if self.use_servo_pulse_turn and hasattr(self, 'servo_pwm') and self.servo_pwm is not None:
            # 좌/우 구분 없이 동일하게 90도 올렸다가 내리는 동작
            return self._servo_pulse_90_cycle()

        if self.imu_available:
            # IMU 기반 정확한 회전
            self.target_angle = self.normalize_angle(self.current_angle - 90)
            self.turn_direction = -1
            self.is_turning = True
            self.integral = 0.0  # 적분 항 리셋
            
            if self.debug:
                print(f"90도 우회전 시작 (IMU 기반): 현재 {self.current_angle:.2f}도 -> 목표 {self.target_angle:.2f}도")
        else:
            # IMU 없이 시간 기반 회전
            if self.debug:
                print("90도 우회전 시작 (시간 기반)")
            
            # 우회전: 왼쪽 모터 전진, 오른쪽 모터 후진
            self.differential_drive(40, -40)
            time.sleep(1.5)  # 약 1.5초 회전 (실제 테스트로 조정 필요)
            self.stop_all()
            
            if self.debug:
                print("90도 우회전 완료 (시간 기반)")
        
        return True

    def servo_up(self):
        """서보 모터를 UP 위치(90도)로 이동"""
        if hasattr(self, 'servo_pwm') and self.servo_pwm is not None:
            self.set_servo_us(self.steering_servo_channel, self.servo_up_us)
            self.servo_is_up = True
            if self.debug:
                print(f"서보 UP: ch={self.steering_servo_channel}, us={self.servo_up_us}")
            return True
        return False

    def servo_down(self):
        """서보 모터를 DOWN 위치(90도 아래)로 이동"""
        if hasattr(self, 'servo_pwm') and self.servo_pwm is not None:
            self.set_servo_us(self.steering_servo_channel, self.servo_down_us)
            self.servo_is_up = False
            if self.debug:
                print(f"서보 DOWN: ch={self.steering_servo_channel}, us={self.servo_down_us}")
            return True
        return False

    def _servo_pulse_90_cycle(self):
        """기존 90도 회전용 (현재는 서보 UP/DOWN으로 대체)"""
        # 좌/우회전에서 서보 기반이 활성화된 경우 UP으로 처리
        return self.servo_up()

        if self.imu_available:
            # IMU 기반 정확한 회전
            self.target_angle = self.normalize_angle(self.current_angle - 90)
            self.turn_direction = -1
            self.is_turning = True
            self.integral = 0.0  # 적분 항 리셋
            
            if self.debug:
                print(f"90도 우회전 시작 (IMU 기반): 현재 {self.current_angle:.2f}도 -> 목표 {self.target_angle:.2f}도")
        else:
            # IMU 없이 시간 기반 회전
            if self.debug:
                print("90도 우회전 시작 (시간 기반)")
            
            # 우회전: 왼쪽 모터 전진, 오른쪽 모터 후진
            self.differential_drive(40, -40)
            time.sleep(1.5)  # 약 1.5초 회전 (실제 테스트로 조정 필요)
            self.stop_all()
            
            if self.debug:
                print("90도 우회전 완료 (시간 기반)")
        
        return True

    def turn_to_angle(self, target_angle):
        """지정된 각도로 회전"""
        if self.is_turning:
            if self.debug:
                print("이미 회전 중입니다.")
            return False
            
        self.target_angle = self.normalize_angle(target_angle)
        error = self.calculate_angle_error(self.target_angle, self.current_angle)
        
        if error > 0:
            self.turn_direction = 1  # 좌회전
        else:
            self.turn_direction = -1  # 우회전
            
        self.is_turning = True
        self.integral = 0.0  # 적분 항 리셋
        
        if self.debug:
            print(f"각도 회전 시작: 현재 {self.current_angle:.2f}도 -> 목표 {self.target_angle:.2f}도")
        
        return True

    def is_turning_now(self):
        """현재 회전 중인지 확인"""
        return self.is_turning

    def get_current_angle(self):
        """현재 각도 반환"""
        return self.current_angle

    def set_serial_number(self, serial):
        self.serial_number = serial
        if self.debug:
            print(f"시리얼 번호 설정: {serial}")

    def connect_backend(self):
        """Backend MQTT 연결"""
        try:
            if self.debug:
                print(f"Backend 연결 시도: {self.backend_broker}:{self.backend_port}")
            
            self.backend_transmitter = SensorDataTransmitter(
                robot_id=self.serial_number,
                mqtt_broker=self.backend_broker,
                mqtt_port=self.backend_port
            )
            
            if self.backend_transmitter.connect_mqtt():
                self.backend_connected = True
                if self.debug:
                    print("Backend 연결 성공")
                    print(f"연결된 브로커: {self.backend_broker}:{self.backend_port}")
                    print(f"로봇 ID: {self.serial_number}")
                return True
            else:
                if self.debug:
                    print("Backend 연결 실패")
                return False
                
        except Exception as e:
            if self.debug:
                print(f"Backend 연결 오류: {e}")
            return False

    def disconnect_backend(self):
        """Backend MQTT 연결 해제"""
        if self.backend_transmitter:
            self.backend_transmitter.disconnect_mqtt()
            self.backend_connected = False
            if self.debug:
                print("Backend 연결 해제")

    def send_to_backend(self, ai_data=None, motor_status=None):
        if not self.backend_connected or not self.backend_transmitter:
            if self.debug:
                print("Backend가 연결되지 않았습니다.")
            return False
        
        try:
            if self.debug:
                print("Backend 데이터 전송 시작")
            
            state = "RUNNING"
            x = 0.0
            y = 0.0
            speed = 40.0  # 기본 속도 40%
            angle = self.current_angle  # IMU 각도 사용
            
            if ai_data:
                x = float(ai_data.get('x', 0))
                y = float(ai_data.get('y', 0))
                
                motor_case = self.determine_case_from_coordinates(x, y)
                
                if motor_case in ['forward', '전진']:
                    speed = self.get_motor_speed('forward')
                elif motor_case in ['backward', '후진']:
                    speed = self.get_motor_speed('backward')
                elif motor_case in ['left', '좌회전']:
                    speed = self.get_motor_speed('left')
                elif motor_case in ['right', '우회전']:
                    speed = self.get_motor_speed('right')
                elif motor_case in ['stop', '정지']:
                    speed = self.get_motor_speed('stop')
                elif motor_case in ['custom', '커스텀']:
                    speed = self.get_motor_speed('custom')
            
            if motor_status:
                motor_a_speed = motor_status.get('motor_a', {}).get('speed', 0)
                motor_b_speed = motor_status.get('motor_b', {}).get('speed', 0)
                if motor_a_speed > 0 or motor_b_speed > 0:
                    speed = max(motor_a_speed, motor_b_speed)
            
            if self.debug:
                print(f"전송할 데이터: state={state}, x={x}, y={y}, speed={speed}, angle={angle}")
            
            self.backend_transmitter.update_embedded_data(
                serial=self.serial_number,
                state=state,
                x=x,
                y=y,
                speed=speed,
                angle=angle
            )
            
            if self.backend_transmitter.send_embedded_data():
                if self.debug:
                    print(f"Backend 전송 성공: {state}, 속도: {speed}%, 각도: {angle:.2f}도")
                return True
            else:
                if self.debug:
                    print("Backend 전송 실패")
                return False
                
        except Exception as e:
            if self.debug:
                print(f"Backend 전송 오류: {e}")
            return False

    def get_ai_data(self):
        if not self.api_url:
            if self.debug:
                print("API URL이 설정되지 않았습니다.")
            return None
            
        try:
            # AI 명령 API 엔드포인트 사용
            command_url = self.api_url
            response = requests.get(command_url, timeout=5)
            if response.status_code == 200:
                data = response.json()
                self.last_ai_data = data
                if self.debug:
                    print(f"AI 명령 수신: {data}")
                return data
            else:
                if self.debug:
                    print(f"AI 명령 API 요청 실패: {response.status_code}")
                return None
        except Exception as e:
            if self.debug:
                print(f"AI 명령 가져오기 오류: {e}")
            return None

    def process_ai_command(self, ai_data):
        if not ai_data:
            return
            
        try:
            # AI 명령 구조에 맞게 수정
            command_code = ai_data.get('code', 0)
            command_name = ai_data.get('name', 'UNKNOWN')
            
            if self.debug:
                print(f"AI 명령 처리:")
                print(f"  명령 코드: {command_code}")
                print(f"  명령 이름: {command_name}")
                print(f"  현재 각도: {self.current_angle:.2f}도")
            
            # 명령 코드에 따른 동작 실행
            self.execute_ai_command(command_code, command_name)
            
            motor_status = self.get_motor_status()
            self.send_to_backend(ai_data, motor_status)
            
        except Exception as e:
            if self.debug:
                print(f"AI 명령 처리 오류: {e}")

    def execute_ai_command(self, command_code, command_name):
        """AI 명령 코드에 따른 동작 실행"""
        speed = self.get_motor_speed('custom')  # 기본 속도 사용
        
        if command_code == 0 or command_name == 'STOP':
            # 정지
            self.stop_all()
            self._cancel_move_boost()
            if self.debug:
                print(f"정지 명령 실행")
                
        elif command_code == 1 or command_name == 'MOVING_FORWARD':
            # 전진
            self.activate_move_boost(forward=True)
            if self.debug:
                print(f"전진 명령 실행: 속도 {speed}%")
                
        elif command_code == 2 or command_name == 'MOVING_BACKWARD':
            # 후진
            self.activate_move_boost(forward=False)
            if self.debug:
                print(f"후진 명령 실행: 속도 {speed}%")
                
        elif command_code == 3 or command_name == 'ROTATE_LEFT':
            # IMU를 이용한 정확한 90도 좌회전
            self._cancel_move_boost()
            if not self.is_turning:
                self.turn_left_90()
            if self.debug:
                print(f"좌회전 명령 실행 (IMU 기반 90도)")
                
        elif command_code == 4 or command_name == 'ROTATE_RIGHT':
            # IMU를 이용한 정확한 90도 우회전
            self._cancel_move_boost()
            if not self.is_turning:
                self.turn_right_90()
            if self.debug:
                print(f"우회전 명령 실행 (IMU 기반 90도)")
                
        else:
            if self.debug:
                print(f"알 수 없는 명령: 코드 {command_code}, 이름 {command_name}")
            self.stop_all()

    def determine_case_from_coordinates(self, x, y):
        abs_x = abs(x)
        abs_y = abs(y)
        
        threshold = 0.3
        
        if abs_x <= threshold and abs_y <= threshold:
            return 'stop'
        
        if abs_y > abs_x and abs_y > threshold:
            if y > 0:
                return 'forward'
            else:
                return 'backward'
        
        if abs_x >= abs_y and abs_x > threshold:
            if x > 0:
                return 'right'
            else:
                return 'left'
        
        return 'custom'

    def execute_ai_case(self, case, x, y):
        case = case.lower()
        
        speed = self.get_motor_speed(case)
        
        if case == 'forward' or case == '전진':
            # 전진: 양쪽 모터 동일 속도
            self.activate_move_boost(forward=True)
            if self.debug:
                print(f"전진 명령 실행: 속도 {speed}%")
                
        elif case == 'backward' or case == '후진':
            # 후진: 양쪽 모터 동일 속도
            self.activate_move_boost(forward=False)
            if self.debug:
                print(f"후진 명령 실행: 속도 {speed}%")
                
        elif case == 'stop' or case == '정지':
            self.stop_all()
            self._cancel_move_boost()
            if self.debug:
                print("정지 명령 실행")
                
        elif case == 'left' or case == '좌회전':
            # IMU를 이용한 정확한 90도 좌회전
            if not self.is_turning:
                self.turn_left_90()
            if self.debug:
                print(f"좌회전 명령 실행 (IMU 기반)")
                
        elif case == 'right' or case == '우회전':
            # IMU를 이용한 정확한 90도 우회전
            if not self.is_turning:
                self.turn_right_90()
            if self.debug:
                print(f"우회전 명령 실행 (IMU 기반)")
                
        elif case == 'custom' or case == '커스텀':
            left_speed = int(y * 100)  
            right_speed = int(x * 100) 
            
            left_speed = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))
            
            self.differential_drive(left_speed, right_speed)
            if self.debug:
                print(f"커스텀 제어: L={left_speed}, R={right_speed}")
                
        else:
            if self.debug:
                print(f"알 수 없는 케이스: {case}")
            self.stop_all()

    def send_status_to_ai(self, status_data=None):
        if not self.api_url:
            return
        
        # 테스트 시 상태 전송 비활성화 옵션 (로그 줄이기)
        if hasattr(self, 'disable_status_send') and self.disable_status_send:
            return
            
        try:
            status = {
                "serial": self.serial_number,
                "motor_a_speed": self.motor_a_speed,
                "motor_b_speed": self.motor_b_speed,
                "motor_a_direction": self.motor_a_direction,
                "motor_b_direction": self.motor_b_direction,
                "current_angle": self.current_angle,
                "is_turning": self.is_turning,
                "timestamp": datetime.now().isoformat()
            }
            
            if status_data:
                status.update(status_data)
                
            # 임베디드 장치는 상태 전송하지 않음 (AI 서버 단순화)
            # 필요시 MQTT나 별도 엔드포인트로 전송
            if self.debug and hasattr(self, '_status_debug_counter'):
                self._status_debug_counter = getattr(self, '_status_debug_counter', 0) + 1
                if self._status_debug_counter % 10 == 0:  # 10번마다 한 번만 출력
                    print(f"[DEBUG] 로봇 상태: 속도=({status['motor_a_speed']}, {status['motor_b_speed']}), 각도={status['current_angle']:.1f}°")
            return  # 상태 전송 비활성화
            
        except Exception as e:
            if self.debug:
                print(f"상태 처리 오류: {e}")

    def run_ai_control_loop(self, interval=1.0):
        if not self.api_url:
            print("API URL이 설정되지 않았습니다.")
            return
            
        print(f"IMU AI 제어 루프 시작 (간격: {interval}초)")
        
        # IMU 제어 루프 시작
        self.start_control_loop()
        
        try:
            while True:
                ai_data = self.get_ai_data()
                
                if ai_data:
                    self.process_ai_command(ai_data)
                    self.send_status_to_ai()
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\nIMU AI 제어 루프 중단")
            self.stop_all()
            self.stop_control_loop()
        except Exception as e:
            print(f"IMU AI 제어 루프 오류: {e}")
            self.stop_all()
            self.stop_control_loop()

    def set_motor_speed(self, motor, direction, speed):
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
            
        if motor == 0:  # 모터 A (왼쪽 앞+뒤 모터 그룹)
            self.pwm.setDutycycle(self.PWMA, speed)
            if direction == self.FORWARD:
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:  # backward
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
            
            self.motor_a_speed = speed
            self.motor_a_direction = direction
            
            if self.debug:
                print(f"모터 A: {direction}, 속도: {speed}%")
                
        elif motor == 1:  # 모터 B
            self.pwm.setDutycycle(self.PWMB, speed)
            if direction == self.FORWARD:
                self.pwm.setLevel(self.BIN1, 1)  # 반전: 0→1
                self.pwm.setLevel(self.BIN2, 0)  # 반전: 1→0
            else:  # backward
                self.pwm.setLevel(self.BIN1, 0)  # 반전: 1→0
                self.pwm.setLevel(self.BIN2, 1)  # 반전: 0→1
            
            self.motor_b_speed = speed
            self.motor_b_direction = direction
            
            if self.debug:
                print(f"모터 B: {direction}, 속도: {speed}%")
    
    def stop_motor(self, motor):
        if motor == 0:
            self.pwm.setDutycycle(self.PWMA, 0)
            self.motor_a_speed = 0
            if self.debug:
                print("모터 A 정지")
        elif motor == 1:
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
            if self.debug:
                print("모터 B 정지")
    
    def stop_all(self):
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        if self.debug:
            print("모든 모터 정지")

    # --- 부스트 주행 로직 ---
    def activate_move_boost(self, forward=True):
        """극한 부스트: 진동 모드로 정적 마찰력 극복"""
        # 진동 주행으로 정적 마찰력 극복
        self._vibration_start(forward)
        
        # 방향 설정  
        left = self.boost_speed if forward else -self.boost_speed
        right = self.boost_speed if forward else -self.boost_speed
        self.differential_drive(left, right)
        self.current_motion = 'forward' if forward else 'backward'
        # 타이머 재설정
        self._cancel_move_boost()
    
    def _vibration_start(self, forward=True):
        """진동 모드: 짧은 펄스로 정적 마찰력 극복"""
        direction = 1 if forward else -1
        
        # 3번의 절대 최대 강도 펄스로 시작
        for i in range(3):
            # 300% 절대 최대 펄스 (극한 정적 마찰력 극복)
            self.differential_drive(300 * direction, 300 * direction)
            time.sleep(0.1)
            # 잠깐 정지
            self.differential_drive(0, 0)
            time.sleep(0.05)
            
        if self.debug:
            print(f"안정 진동 모드 완료: {'전진' if forward else '후진'}")
        self.move_boost_timer = threading.Timer(self.move_boost_duration, self._switch_to_cruise)
        self.move_boost_timer.daemon = True
        self.move_boost_timer.start()

    def _switch_to_cruise(self):
        if self.current_motion in ['forward', 'backward']:
            speed = self.cruise_speed if self.current_motion == 'forward' else -self.cruise_speed
            self.differential_drive(speed, speed)

    def _cancel_move_boost(self):
        if self.move_boost_timer is not None:
            try:
                self.move_boost_timer.cancel()
            except Exception:
                pass
            self.move_boost_timer = None
        self.current_motion = 'stop'
    
    def get_motor_status(self):
        return {
            'motor_a': {
                'speed': self.motor_a_speed,
                'direction': self.motor_a_direction
            },
            'motor_b': {
                'speed': self.motor_b_speed,
                'direction': self.motor_b_direction
            }
        }
    
    def test_imu_motors(self):
        """IMU 모터 통합 테스트"""
        print("\n=== IMU 모터 통합 테스트 시작 ===")
        
        # IMU 제어 루프 시작
        self.start_control_loop()
        
        try:
            # 0. 정중앙 맞추기 테스트
            print("\n0. 정중앙 맞추기 테스트")
            initial_status = self.get_center_status()
            print(f"초기 상태: {initial_status}")
            
            if self.center_robot():
                while self.is_turning:
                    print(f"정중앙 맞추기 중... 현재 각도: {self.current_angle:.2f}도")
                    time.sleep(0.1)
                final_status = self.get_center_status()
                print(f"정중앙 맞추기 완료! 최종 상태: {final_status}")
            else:
                print("정중앙 맞추기 실패")
            
            time.sleep(2)
            
            # 1. 90도 좌회전 테스트
            print("\n1. 90도 좌회전 테스트")
            if self.turn_left_90():
                while self.is_turning:
                    print(f"회전 중... 현재 각도: {self.current_angle:.2f}도")
                    time.sleep(0.1)
                print("좌회전 완료!")
            
            time.sleep(2)
            
            # 2. 90도 우회전 테스트
            print("\n2. 90도 우회전 테스트")
            if self.turn_right_90():
                while self.is_turning:
                    print(f"회전 중... 현재 각도: {self.current_angle:.2f}도")
                    time.sleep(0.1)
                print("우회전 완료!")
            
            time.sleep(2)
            
            # 3. 전진 테스트 (5초)
            print("\n3. 전진 테스트 (5초)")
            self.differential_drive(30, 30)
            time.sleep(5)
            self.stop_all()
            
            # 4. 후진 테스트 (5초)
            print("\n4. 후진 테스트 (5초)")
            self.differential_drive(-30, -30)
            time.sleep(5)
            self.stop_all()
            
            print("\n=== IMU 모터 통합 테스트 완료 ===")
            
        except KeyboardInterrupt:
            print("\n테스트 중단")
        finally:
            self.stop_all()
            self.stop_control_loop()
    
    def test_motors_only(self):
        """모터만 테스트 (IMU 없이)"""
        print("\n=== 모터 전용 테스트 시작 (IMU 없이) ===")
        
        try:
            # 1. 전진 테스트 (3초)
            print("\n1. 전진 테스트 (3초)")
            self.differential_drive(30, 30)
            time.sleep(3)
            self.stop_all()
            
            time.sleep(1)
            
            # 2. 후진 테스트 (3초)
            print("\n2. 후진 테스트 (3초)")
            self.differential_drive(-30, -30)
            time.sleep(3)
            self.stop_all()
            
            time.sleep(1)
            
            # 3. 좌회전 테스트 (시간 기반)
            print("\n3. 좌회전 테스트 (시간 기반)")
            self.differential_drive(-40, 40)
            time.sleep(1.5)
            self.stop_all()
            
            time.sleep(1)
            
            # 4. 우회전 테스트 (시간 기반)
            print("\n4. 우회전 테스트 (시간 기반)")
            self.differential_drive(40, -40)
            time.sleep(1.5)
            self.stop_all()
            
            time.sleep(1)
            
            # 5. 차동 구동 테스트
            print("\n5. 차동 구동 테스트 (왼쪽 빠르게)")
            self.differential_drive(50, 20)
            time.sleep(2)
            self.stop_all()
            
            print("\n=== 모터 전용 테스트 완료 ===")
            
        except KeyboardInterrupt:
            print("\n테스트 중단")
        finally:
            self.stop_all()
    
    def differential_drive(self, left_speed, right_speed):
        """
        차동 구동 (로봇 이동) - 모터 A와 B 동시 제어
        
        Args:
            left_speed (int): 왼쪽 모터 속도 (-300 ~ 300)
            right_speed (int): 오른쪽 모터 속도 (-300 ~ 300)
        """
        # 속도 제한 (300%까지 허용 - 절대 최대 모드)
        left_speed = max(-300, min(300, left_speed))
        right_speed = max(-300, min(300, right_speed))
        
        # 모터 A (왼쪽 앞+뒤 모터 그룹) 설정
        if left_speed > 0:
            # 모터 A 전진
            self.pwm.setDutycycle(self.PWMA, abs(left_speed))
            self.pwm.setLevel(self.AIN1, 0)
            self.pwm.setLevel(self.AIN2, 1)
            self.motor_a_speed = abs(left_speed)
            self.motor_a_direction = self.FORWARD
        elif left_speed < 0:
            # 모터 A 후진
            self.pwm.setDutycycle(self.PWMA, abs(left_speed))
            self.pwm.setLevel(self.AIN1, 1)
            self.pwm.setLevel(self.AIN2, 0)
            self.motor_a_speed = abs(left_speed)
            self.motor_a_direction = self.BACKWARD
        else:
            # 모터 A 정지
            self.pwm.setDutycycle(self.PWMA, 0)
            self.motor_a_speed = 0
        
        # 모터 B (오른쪽 앞+뒤 모터 그룹) 설정 - 방향 반전
        if right_speed > 0:
            # 모터 B 전진 (방향 반전)
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 1)  # 반전: 0→1
            self.pwm.setLevel(self.BIN2, 0)  # 반전: 1→0
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.FORWARD
        elif right_speed < 0:
            # 모터 B 후진 (방향 반전)
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 0)  # 반전: 1→0
            self.pwm.setLevel(self.BIN2, 1)  # 반전: 0→1
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.BACKWARD
        else:
            # 모터 B 정지
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
        
        if self.debug:
            print(f"차동 구동: L={left_speed}, R={right_speed}")

    def set_motor_speed_config(self, speeds):
        self.motor_speeds.update(speeds)
        if self.debug:
            print(f"모터 속도 설정 업데이트: {self.motor_speeds}")

    def get_motor_speed(self, case):
        case = case.lower()
        return self.motor_speeds.get(case, self.motor_speeds['custom'])

    def set_pid_gains(self, kp, ki, kd):
        """PID 게인 설정"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        if self.debug:
            print(f"PID 게인 설정: Kp={kp}, Ki={ki}, Kd={kd}")

    def set_servo_angle(self, channel, angle):
        """서보 모터 각도 설정"""
        if not hasattr(self, 'servo_pwm') or self.servo_pwm is None:
            if self.debug:
                print("서보 모터 드라이버가 초기화되지 않았습니다.")
            return False
        
        try:
            # 각도를 펄스 폭으로 변환 (0도=1ms, 180도=2ms)
            # 50Hz에서 1ms = 50 펄스, 2ms = 100 펄스
            pulse_width = int(50 + (angle / 180.0) * 50)  # 50-100 펄스 범위
            
            self.servo_pwm.setPWM(channel, 0, pulse_width)
            
            if self.debug:
                print(f"서보 모터 {channel}번 채널: {angle}도 (펄스: {pulse_width})")
            
            return True
            
        except Exception as e:
            if self.debug:
                print(f"서보 모터 제어 실패: {e}")
            return False

    def set_servo_pulse(self, channel, pulse_width):
        """서보 모터 펄스 폭 직접 설정"""
        if not hasattr(self, 'servo_pwm') or self.servo_pwm is None:
            if self.debug:
                print("서보 모터 드라이버가 초기화되지 않았습니다.")
            return False
        
        try:
            self.servo_pwm.setPWM(channel, 0, pulse_width)
            
            if self.debug:
                print(f"서보 모터 {channel}번 채널: 펄스 폭 {pulse_width}")
            
            return True
            
        except Exception as e:
            if self.debug:
                print(f"서보 모터 제어 실패: {e}")
            return False

    def set_servo_us(self, channel, microseconds):
        """서보 모터를 펄스 폭(마이크로초)으로 제어"""
        if not hasattr(self, 'servo_pwm') or self.servo_pwm is None:
            if self.debug:
                print("서보 모터 드라이버가 초기화되지 않았습니다.")
            return False
        try:
            # 트림 반영 및 안전 클램프
            microseconds = int(microseconds) + int(self.servo_trim_us.get(channel, 0))
            microseconds = max(self.servo_min_us, min(self.servo_max_us, microseconds))
            # 50Hz → 20,000us 주기, 12-bit(4096) 분해능
            counts = int(microseconds * 4096 / 20000)
            counts = max(0, min(4095, counts))
            self.servo_pwm.setPWM(channel, 0, counts)
            if self.debug:
                print(f"서보 {channel}: {microseconds}us -> {counts}cnt")
            return True
        except Exception as e:
            if self.debug:
                print(f"서보 us 제어 실패: {e}")
            return False

    def set_servo_percent(self, channel, percent):
        """서보 모터를 퍼센트(-100~100)로 제어 (중립 0%)"""
        try:
            percent = max(-100, min(100, float(percent)))
            span = self.servo_max_us - self.servo_min_us
            half = span / 2.0
            microseconds = int(self.servo_center_us + (percent / 100.0) * half)
            return self.set_servo_us(channel, microseconds)
        except Exception as e:
            if self.debug:
                print(f"서보 percent 제어 실패: {e}")
            return False

    def stop_servo(self, channel):
        """서보 모터 정지"""
        if not hasattr(self, 'servo_pwm') or self.servo_pwm is None:
            return False
        
        try:
            self.servo_pwm.setPWM(channel, 0, 0)
            
            if self.debug:
                print(f"서보 모터 {channel}번 채널 정지")
            
            return True
            
        except Exception as e:
            if self.debug:
                print(f"서보 모터 정지 실패: {e}")
            return False

    def test_servos(self):
        """서보 모터 테스트"""
        if not hasattr(self, 'servo_pwm') or self.servo_pwm is None:
            print("서보 모터 드라이버가 초기화되지 않았습니다.")
            return
        
        print("=== 서보 모터 테스트 ===")
        
        # 테스트할 채널들
        channels = [0, 1, 2, 3]  # 0-3번 채널 테스트
        
        for channel in channels:
            print(f"\n채널 {channel} 테스트:")
            
            # 0도
            self.set_servo_angle(channel, 0)
            time.sleep(1)
            
            # 90도
            self.set_servo_angle(channel, 90)
            time.sleep(1)
            
            # 180도
            self.set_servo_angle(channel, 180)
            time.sleep(1)
            
            # 90도로 복귀
            self.set_servo_angle(channel, 90)
            time.sleep(1)
            
            # 정지
            self.stop_servo(channel)
        
        print("\n서보 모터 테스트 완료")

    def start_command_processor(self):
        """명령 처리 스레드 시작"""
        if self.debug:
            print("명령 처리 스레드 시작")
        # 여기에 명령 처리 로직 추가 가능

    def get_current_command(self):
        """현재 명령 상태 반환"""
        return {
            "code": 0,
            "name": "STOP",
            "timestamp": datetime.now().isoformat()
        }

def main():
    print("=" * 60)
    print("IMU AI 모터 컨트롤러 테스트")
    print("=" * 60)
    
    ai_api_url = "http://localhost:5001/command"  # AI 서버 주소
    controller = None
    
    try:
        controller = IMUAIMotorController(
            debug=True, 
            api_url=ai_api_url,
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        controller.set_serial_number("AMR001")
        
        if controller.connect_backend():
            print("Backend 연결 성공")
        else:
            print("Backend 연결 실패")
        
        print("\n임베디드 장치 모드를 선택하세요:")
        print("1. 전체 IMU 모터 통합 테스트 (정중앙 맞추기 포함)")
        print("2. 정중앙 맞추기만 실행")
        print("3. AI 제어 루프 실행 (AI 서버에서 명령 수신)")
        print("4. 모터 전용 테스트 (IMU 없이)")
        print("5. Flask 웹 서버 + 명령 처리 시작 (사용 안 함)")
        print("6. 서보 모터 테스트")
        
        choice = input("\n선택 (1-6): ").strip()
        
        if choice == "1":
            # IMU 모터 통합 테스트 실행
            controller.test_imu_motors()
        elif choice == "2":
            # 정중앙 맞추기만 실행
            print("\n=== 정중앙 맞추기만 실행 ===")
            controller.start_control_loop()
            try:
                initial_status = controller.get_center_status()
                print(f"초기 상태: {initial_status}")
                
                if controller.center_robot():
                    while controller.is_turning:
                        print(f"정중앙 맞추기 중... 현재 각도: {controller.current_angle:.2f}도")
                        time.sleep(0.1)
                    final_status = controller.get_center_status()
                    print(f"정중앙 맞추기 완료! 최종 상태: {final_status}")
                else:
                    print("정중앙 맞추기 실패")
            finally:
                controller.stop_all()
                controller.stop_control_loop()
        elif choice == "3":
            # AI 제어 루프 실행
            controller.run_ai_control_loop(interval=1.0)
        elif choice == "4":
            # 모터 전용 테스트 (IMU 없이)
            controller.test_motors_only()
        elif choice == "5":
            # Flask 웹 서버 기능은 제거됨 (AI 서버로 분리)
            print("\n⚠️  옵션 5는 더 이상 사용되지 않습니다.")
            print("AI 서버는 별도로 실행하세요:")
            print("  python3 ai_client.py")
            print("\n현재 장치는 임베디드 모드로만 동작합니다.")
        elif choice == "6":
            # 서보 모터 테스트
            controller.test_servos()
        else:
            print("잘못된 선택입니다.")
        
        # AI 제어 루프 실행 (테스트 후 주석 해제)
        # controller.run_ai_control_loop(interval=1.0)
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단")
        if controller:
            controller.stop_all()
            controller.stop_control_loop()
            controller.disconnect_backend()
    except Exception as e:
        print(f"\n오류 발생: {e}")
        if controller:
            controller.stop_all()
            controller.stop_control_loop()
            controller.disconnect_backend()
    finally:
        print("정리 완료")

if __name__ == "__main__":
    main()
