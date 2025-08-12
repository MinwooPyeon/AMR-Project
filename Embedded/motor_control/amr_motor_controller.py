#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
통합 AMR 모터 컨트롤러
IMU 기반 정확한 각도 제어와 AI 통신을 통합한 완전한 AMR 제어 시스템
"""

import time
import math
import smbus
import json
import requests
import threading
from datetime import datetime
from PCA9685 import PCA9685

# 명령 테이블 정의
COMMAND_TABLE = {
    0: 'STOP',
    1: 'MOVING_FORWARD', 
    2: 'MOVING_BACKWARD',
    3: 'ROTATE_LEFT',
    4: 'ROTATE_RIGHT'
}

class AMRMotorController:
    """
    통합 AMR 모터 컨트롤러
    
    기능:
    - IMU 기반 정확한 90도 회전
    - AI 명령 처리
    - Backend MQTT 통신
    - 서보 모터 제어
    - PID 제어를 통한 정밀 각도 제어
    """
    
    # 모터 방향 정의
    FORWARD = 'forward'
    BACKWARD = 'backward'
    
    def __init__(self, 
                 motor_i2c_address=0x40, 
                 servo_i2c_address=0x60,
                 imu_i2c_address=0x68,
                 i2c_bus=None,
                 debug=True, 
                 api_url=None, 
                 backend_broker="192.168.100.141", 
                 backend_port=1883):
        """
        Args:
            motor_i2c_address (int): 모터 드라이버 I2C 주소 (기본값: 0x40)
            servo_i2c_address (int): 서보 드라이버 I2C 주소 (기본값: 0x60)
            imu_i2c_address (int): IMU 센서 I2C 주소 (기본값: 0x68)
            i2c_bus (int): I2C 버스 번호 (기본값: None, 자동 감지)
            debug (bool): 디버그 모드 (기본값: True)
            api_url (str): AI API URL (기본값: None)
            backend_broker (str): Backend MQTT 브로커 주소
            backend_port (int): Backend MQTT 포트
        """
        self.debug = debug
        self.motor_i2c_address = motor_i2c_address
        self.servo_i2c_address = servo_i2c_address
        self.imu_i2c_address = imu_i2c_address
        self.i2c_bus = i2c_bus
        self.api_url = api_url
        self.backend_broker = backend_broker
        self.backend_port = backend_port
        
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
        self.serial_number = "AMR001"
        
        # Backend 통신
        self.backend_transmitter = None
        self.backend_connected = False
        
        # IMU 관련 변수
        self.imu_bus = None
        self.initial_angle = 0.0
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.angle_offset = 0.0
        self.is_turning = False
        self.turn_direction = 0
        self.imu_available = False
        
        # PID 제어 변수
        self.kp = 2.0
        self.ki = 0.1
        self.kd = 0.5
        self.prev_error = 0.0
        self.integral = 0.0
        
        # 모터 속도 설정
        self.motor_speeds = {
            'forward': 100,
            'backward': 100,
            'left': 100,
            'right': 100,
            'stop': 0,
            'custom': 100
        }
        
        # 스레드 제어
        self.control_thread = None
        self.running = False
        self.thread_lock = threading.Lock()
        
        # 초기화
        self._initialize_hardware()
        
    def _initialize_hardware(self):
        """하드웨어 초기화"""
        try:
            # PCA9685 모터 드라이버 초기화
            self.pwm = PCA9685(self.motor_i2c_address, debug=self.debug, i2c_bus=self.i2c_bus)
            self.pwm.setPWMFreq(20)  # 20Hz PWM 주파수
            
            # PCA9685 서보 모터 드라이버 초기화
            try:
                self.servo_pwm = PCA9685(self.servo_i2c_address, debug=self.debug, i2c_bus=self.i2c_bus)
                self.servo_pwm.setPWMFreq(50)  # 50Hz PWM 주파수
                if self.debug:
                    print(f"서보 모터 드라이버 초기화 성공 - 주소: 0x{self.servo_i2c_address:02X}")
            except Exception as e:
                if self.debug:
                    print(f"서보 모터 드라이버 초기화 실패: {e}")
                self.servo_pwm = None
            
            # IMU 초기화
            self._initialize_imu()
            
            # 모든 모터 정지
            self.stop_all()
            
            if self.debug:
                self._print_initialization_info()
                
        except Exception as e:
            print(f"하드웨어 초기화 실패: {e}")
            raise
            
    def _initialize_imu(self):
        """IMU 초기화 및 캘리브레이션"""
        try:
            # I2C 버스 자동 감지
            self.imu_bus = self._find_imu_bus()
            if not self.imu_bus:
                raise Exception("사용 가능한 I2C 버스를 찾을 수 없습니다")
            
            # IMU 주소 자동 감지
            detected_address = self._find_imu_address()
            if detected_address:
                self.imu_i2c_address = detected_address
            
            # MPU6050 초기화
            bus = smbus.SMBus(self.imu_bus)
            bus.write_byte_data(self.imu_i2c_address, 0x6B, 0)
            
            # 초기 각도 캘리브레이션
            print("IMU 캘리브레이션 중...")
            angles = []
            for i in range(100):
                angle = self._read_imu_yaw()
                angles.append(angle)
                time.sleep(0.01)
            
            raw_initial_angle = sum(angles) / len(angles)
            self.initial_angle = 0.0
            self.current_angle = 0.0
            self.target_angle = 0.0
            self.angle_offset = raw_initial_angle
            self.imu_available = True
            
            if self.debug:
                print(f"IMU 초기화 완료")
                print(f"  - I2C 버스: {self.imu_bus}")
                print(f"  - IMU 주소: 0x{self.imu_i2c_address:02X}")
                print(f"  - 각도 오프셋: {self.angle_offset:.2f}도")
                
        except Exception as e:
            print(f"IMU 초기화 실패: {e}")
            print("IMU 없이 모터 제어만 사용합니다.")
            self.imu_available = False
            
    def _find_imu_bus(self):
        """사용 가능한 I2C 버스 찾기"""
        import os
        
        for bus_num in range(10):
            bus_path = f"/dev/i2c-{bus_num}"
            if os.path.exists(bus_path):
                try:
                    bus = smbus.SMBus(bus_num)
                    bus.close()
                    if self.debug:
                        print(f"사용 가능한 I2C 버스 발견: {bus_num}")
                    return bus_num
                except Exception as e:
                    if self.debug:
                        print(f"버스 {bus_num} 테스트 실패: {e}")
                    continue
        return None
        
    def _find_imu_address(self):
        """IMU 센서 주소 찾기"""
        imu_addresses = [0x68, 0x69]
        
        for addr in imu_addresses:
            try:
                bus = smbus.SMBus(self.imu_bus)
                bus.write_byte_data(addr, 0x6B, 0)
                time.sleep(0.1)
                
                who_am_i = bus.read_byte_data(addr, 0x75)
                
                if who_am_i == 0x68:
                    if self.debug:
                        print(f"IMU 센서 발견: 주소 0x{addr:02X}")
                    return addr
                else:
                    if self.debug:
                        print(f"주소 0x{addr:02X} WHO_AM_I: 0x{who_am_i:02X} (예상: 0x68)")
                        
            except Exception as e:
                if self.debug:
                    print(f"주소 0x{addr:02X} 접근 실패: {e}")
                continue
        
        return None
        
    def _read_imu_yaw(self):
        """IMU에서 Yaw 각도 읽기"""
        if not self.imu_available or not self.imu_bus:
            return 0.0
            
        try:
            bus = smbus.SMBus(self.imu_bus)
            
            gyro_x = bus.read_word_data(self.imu_i2c_address, 0x43)
            gyro_y = bus.read_word_data(self.imu_i2c_address, 0x45)
            gyro_z = bus.read_word_data(self.imu_i2c_address, 0x47)
            
            gyro_x = self._convert_to_signed(gyro_x)
            gyro_y = self._convert_to_signed(gyro_y)
            gyro_z = self._convert_to_signed(gyro_z)
            
            gyro_scale = 250.0 / 32768.0
            yaw_rate = gyro_z * gyro_scale
            
            self.current_angle += yaw_rate * 0.01
            self.current_angle = self._normalize_angle(self.current_angle)
            
            corrected_angle = self.current_angle - self.angle_offset
            result = self._normalize_angle(corrected_angle)
            
            bus.close()
            return result
            
        except Exception as e:
            if self.debug:
                print(f"IMU 읽기 오류: {e}")
            return 0.0
            
    def _convert_to_signed(self, value):
        """16비트 값을 부호 있는 정수로 변환"""
        if value > 32767:
            value -= 65536
        return value
        
    def _normalize_angle(self, angle):
        """각도를 -180 ~ 180도 범위로 정규화"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
        
    def _calculate_angle_error(self, target, current):
        """목표 각도와 현재 각도 간의 최단 거리 오차 계산"""
        error = target - current
        
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        return error
        
    def _pid_control(self, error):
        """PID 제어 계산"""
        with self.thread_lock:
            self.integral += error
            derivative = error - self.prev_error
            
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            
            output = max(-100, min(100, output))
            return output
            
    def _print_initialization_info(self):
        """초기화 정보 출력"""
        print(f"모터 드라이버 주소: 0x{self.motor_i2c_address:02X}")
        print(f"서보 드라이버 주소: 0x{self.servo_i2c_address:02X}")
        print(f"I2C 버스: {self.i2c_bus or '자동 감지'}")
        print(f"PWM 주파수: 20Hz (모터), 50Hz (서보)")
        print(f"IMU 주소: 0x{self.imu_i2c_address:02X}")
        print(f"IMU 사용 가능: {self.imu_available}")
        if self.api_url:
            print(f"AI API URL: {self.api_url}")
        print(f"Backend MQTT: {self.backend_broker}:{self.backend_port}")
        print(f"PID 게인: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        
    # 공개 API 메서드들
    def start_control_loop(self):
        """IMU 제어 루프 시작"""
        if self.control_thread and self.control_thread.is_alive():
            return
            
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
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
            
    def _control_loop(self):
        """IMU 기반 모터 제어 루프"""
        while self.running:
            try:
                if self.is_turning:
                    current_angle = self._read_imu_yaw()
                    error = self._calculate_angle_error(self.target_angle, current_angle)
                    motor_speed = self._pid_control(error)
                    
                    if abs(error) > 1.0:
                        if self.turn_direction == 1:
                            self.differential_drive(-abs(motor_speed), abs(motor_speed))
                        elif self.turn_direction == -1:
                            self.differential_drive(abs(motor_speed), -abs(motor_speed))
                    else:
                        self.stop_all()
                        self.is_turning = False
                        if self.debug:
                            print(f"목표 각도 도달: {current_angle:.2f}도")
                
                time.sleep(0.01)
                
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
            
        if self.imu_available:
            self.target_angle = self._normalize_angle(self.current_angle + 90)
            self.turn_direction = 1
            self.is_turning = True
            self.integral = 0.0
            
            if self.debug:
                print(f"90도 좌회전 시작 (IMU 기반): 현재 {self.current_angle:.2f}도 -> 목표 {self.target_angle:.2f}도")
        else:
            if self.debug:
                print("90도 좌회전 시작 (시간 기반)")
            
            self.differential_drive(-40, 40)
            time.sleep(1.5)
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
            
        if self.imu_available:
            self.target_angle = self._normalize_angle(self.current_angle - 90)
            self.turn_direction = -1
            self.is_turning = True
            self.integral = 0.0
            
            if self.debug:
                print(f"90도 우회전 시작 (IMU 기반): 현재 {self.current_angle:.2f}도 -> 목표 {self.target_angle:.2f}도")
        else:
            if self.debug:
                print("90도 우회전 시작 (시간 기반)")
            
            self.differential_drive(40, -40)
            time.sleep(1.5)
            self.stop_all()
            
            if self.debug:
                print("90도 우회전 완료 (시간 기반)")
        
        return True
        
    def differential_drive(self, left_speed, right_speed):
        """차동 구동"""
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # 모터 A (왼쪽 모터)
        if left_speed > 0:
            self.pwm.setDutycycle(self.PWMA, abs(left_speed))
            self.pwm.setLevel(self.AIN1, 0)
            self.pwm.setLevel(self.AIN2, 1)
            self.motor_a_speed = abs(left_speed)
            self.motor_a_direction = self.FORWARD
        elif left_speed < 0:
            self.pwm.setDutycycle(self.PWMA, abs(left_speed))
            self.pwm.setLevel(self.AIN1, 1)
            self.pwm.setLevel(self.AIN2, 0)
            self.motor_a_speed = abs(left_speed)
            self.motor_a_direction = self.BACKWARD
        else:
            self.pwm.setDutycycle(self.PWMA, 0)
            self.motor_a_speed = 0
        
        # 모터 B (오른쪽 모터)
        if right_speed > 0:
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 0)
            self.pwm.setLevel(self.BIN2, 1)
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.FORWARD
        elif right_speed < 0:
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 1)
            self.pwm.setLevel(self.BIN2, 0)
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.BACKWARD
        else:
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
        
        if self.debug:
            print(f"차동 구동: L={left_speed}, R={right_speed}")
            
    def stop_all(self):
        """모든 모터 정지"""
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        if self.debug:
            print("모든 모터 정지")
            
    def set_servo_angle(self, channel, angle):
        """서보 모터 각도 설정"""
        if not hasattr(self, 'servo_pwm') or self.servo_pwm is None:
            if self.debug:
                print("서보 모터 드라이버가 초기화되지 않았습니다.")
            return False
        
        try:
            pulse_width = int(50 + (angle / 180.0) * 50)
            self.servo_pwm.setPWM(channel, 0, pulse_width)
            
            if self.debug:
                print(f"서보 모터 {channel}번 채널: {angle}도 (펄스: {pulse_width})")
            
            return True
            
        except Exception as e:
            if self.debug:
                print(f"서보 모터 제어 실패: {e}")
            return False
            
    def get_motor_status(self):
        """모터 상태 반환"""
        return {
            'motor_a': {
                'speed': self.motor_a_speed,
                'direction': self.motor_a_direction
            },
            'motor_b': {
                'speed': self.motor_b_speed,
                'direction': self.motor_b_direction
            },
            'current_angle': self.current_angle,
            'is_turning': self.is_turning
        }
        
    def set_pid_gains(self, kp, ki, kd):
        """PID 게인 설정"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        if self.debug:
            print(f"PID 게인 설정: Kp={kp}, Ki={ki}, Kd={kd}")
            
    def set_motor_speeds(self, speeds):
        """모터 속도 설정"""
        self.motor_speeds.update(speeds)
        if self.debug:
            print(f"모터 속도 설정 업데이트: {self.motor_speeds}")
            
    def get_current_angle(self):
        """현재 각도 반환"""
        return self.current_angle
        
    def is_turning_now(self):
        """현재 회전 중인지 확인"""
        return self.is_turning
        
    def center_robot(self):
        """로봇을 정중앙(0도)으로 맞추기"""
        if self.is_turning:
            if self.debug:
                print("이미 회전 중입니다. 정중앙 맞추기를 건너뜁니다.")
            return False
            
        current_angle = self._read_imu_yaw()
        angle_error = abs(current_angle)
        
        if angle_error < 1.0:
            if self.debug:
                print(f"로봇이 이미 정중앙에 있습니다. (각도: {current_angle:.2f}도)")
            return True
            
        if self.debug:
            print(f"정중앙 맞추기 시작 - 현재 각도: {current_angle:.2f}도")
        
        self.target_angle = 0.0
        error = self._calculate_angle_error(self.target_angle, current_angle)
        
        if error > 0:
            self.turn_direction = 1
        else:
            self.turn_direction = -1
            
        self.is_turning = True
        self.integral = 0.0
        
        return True

def main():
    """테스트 메인 함수"""
    print("=" * 60)
    print("통합 AMR 모터 컨트롤러 테스트")
    print("=" * 60)
    
    controller = None
    
    try:
        controller = AMRMotorController(debug=True)
        
        print("\n테스트 옵션을 선택하세요:")
        print("1. 90도 좌회전 테스트")
        print("2. 90도 우회전 테스트")
        print("3. 정중앙 맞추기 테스트")
        print("4. 전진/후진 테스트")
        print("5. 서보 모터 테스트")
        
        choice = input("\n선택 (1-5): ").strip()
        
        if choice == "1":
            controller.start_control_loop()
            controller.turn_left_90()
            while controller.is_turning:
                print(f"회전 중... 현재 각도: {controller.get_current_angle():.2f}도")
                time.sleep(0.1)
            print("좌회전 완료!")
            
        elif choice == "2":
            controller.start_control_loop()
            controller.turn_right_90()
            while controller.is_turning:
                print(f"회전 중... 현재 각도: {controller.get_current_angle():.2f}도")
                time.sleep(0.1)
            print("우회전 완료!")
            
        elif choice == "3":
            controller.start_control_loop()
            controller.center_robot()
            while controller.is_turning:
                print(f"정중앙 맞추기 중... 현재 각도: {controller.get_current_angle():.2f}도")
                time.sleep(0.1)
            print("정중앙 맞추기 완료!")
            
        elif choice == "4":
            print("전진 테스트 (3초)")
            controller.differential_drive(30, 30)
            time.sleep(3)
            controller.stop_all()
            
            time.sleep(1)
            
            print("후진 테스트 (3초)")
            controller.differential_drive(-30, -30)
            time.sleep(3)
            controller.stop_all()
            
        elif choice == "5":
            if hasattr(controller, 'servo_pwm') and controller.servo_pwm:
                print("서보 모터 테스트")
                for channel in [0, 1, 2, 3]:
                    print(f"채널 {channel} 테스트")
                    controller.set_servo_angle(channel, 0)
                    time.sleep(1)
                    controller.set_servo_angle(channel, 90)
                    time.sleep(1)
                    controller.set_servo_angle(channel, 180)
                    time.sleep(1)
                    controller.set_servo_angle(channel, 90)
                    time.sleep(1)
            else:
                print("서보 모터 드라이버가 초기화되지 않았습니다.")
                
        else:
            print("잘못된 선택입니다.")
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단")
    except Exception as e:
        print(f"\n오류 발생: {e}")
    finally:
        if controller:
            controller.stop_all()
            controller.stop_control_loop()
        print("정리 완료")

if __name__ == "__main__":
    main() 