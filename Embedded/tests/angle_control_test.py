#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 기반 회전 제어 테스트 스크립트
"""

import smbus
import time
import math
import numpy as np
from collections import deque

class RotationControlTest:
    def __init__(self, i2c_bus=1, imu_address=0x68):
        self.bus = smbus.SMBus(i2c_bus)
        self.imu_address = imu_address
        
        # MPU9250 레지스터 주소
        self.MPU9250_ADDR = imu_address
        self.PWR_MGMT_1 = 0x6B
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        self.TEMP_OUT_H = 0x41
        
        # AK8963 (자기계) 레지스터 주소
        self.AK8963_ADDR = 0x0C
        self.AK8963_ST1 = 0x02
        self.AK8963_XOUT_L = 0x03
        self.AK8963_CNTL1 = 0x0A
        self.AK8963_ASAX = 0x10
        
        # 각도 계산 변수
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()
        
        # 스무딩을 위한 변수
        self.angle_history = deque(maxlen=10)
        self.smoothing_factor = 0.8
        
        # PID 제어 변수
        self.pid_kp = 2.0
        self.pid_ki = 0.1
        self.pid_kd = 0.5
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        
        # 회전 제어 설정
        self.angle_tolerance = 2.0
        self.target_angle = 0.0
        self.current_angle = 0.0
        self.angle_error = 0.0
        self.initial_angle = 0.0
        
        print("회전 제어 테스트 초기화 완료")
    
    def initialize_mpu9250(self):
        """MPU9250 초기화"""
        try:
            # 전원 관리 레지스터 설정
            self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            # 설정 레지스터
            self.bus.write_byte_data(self.MPU9250_ADDR, self.CONFIG, 0x06)
            
            # 자이로스코프 설정 (±250°/s)
            self.bus.write_byte_data(self.MPU9250_ADDR, self.GYRO_CONFIG, 0x00)
            
            # 가속도계 설정 (±2g)
            self.bus.write_byte_data(self.MPU9250_ADDR, self.ACCEL_CONFIG, 0x00)
            
            print("MPU9250 초기화 완료")
            return True
            
        except Exception as e:
            print(f"MPU9250 초기화 실패: {e}")
            return False
    
    def initialize_ak8963(self):
        """AK8963 (자기계) 초기화"""
        try:
            # AK8963 활성화
            self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL1, 0x00)
            time.sleep(0.1)
            self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL1, 0x16)
            time.sleep(0.1)
            
            print("AK8963 초기화 완료")
            return True
            
        except Exception as e:
            print(f"AK8963 초기화 실패: {e}")
            return False
    
    def read_raw_data(self):
        """원시 센서 데이터 읽기"""
        try:
            # 가속도 데이터 읽기
            accel_data = self.bus.read_i2c_block_data(self.MPU9250_ADDR, self.ACCEL_XOUT_H, 6)
            accel_x = (accel_data[0] << 8) | accel_data[1]
            accel_y = (accel_data[2] << 8) | accel_data[3]
            accel_z = (accel_data[4] << 8) | accel_data[5]
            
            # 자이로 데이터 읽기
            gyro_data = self.bus.read_i2c_block_data(self.MPU9250_ADDR, self.GYRO_XOUT_H, 6)
            gyro_x = (gyro_data[0] << 8) | gyro_data[1]
            gyro_y = (gyro_data[2] << 8) | gyro_data[3]
            gyro_z = (gyro_data[4] << 8) | gyro_data[5]
            
            # 온도 데이터 읽기
            temp_data = self.bus.read_i2c_block_data(self.MPU9250_ADDR, self.TEMP_OUT_H, 2)
            temp = (temp_data[0] << 8) | temp_data[1]
            
            # 자기계 데이터 읽기
            mag_data = self.bus.read_i2c_block_data(self.AK8963_ADDR, self.AK8963_XOUT_L, 7)
            mag_x = (mag_data[1] << 8) | mag_data[0]
            mag_y = (mag_data[3] << 8) | mag_data[2]
            mag_z = (mag_data[5] << 8) | mag_data[4]
            
            return {
                'accel': (accel_x, accel_y, accel_z),
                'gyro': (gyro_x, gyro_y, gyro_z),
                'temp': temp,
                'mag': (mag_x, mag_y, mag_z)
            }
            
        except Exception as e:
            print(f"센서 데이터 읽기 실패: {e}")
            return None
    
    def convert_to_physical_units(self, raw_data):
        """원시 데이터를 물리적 단위로 변환"""
        if raw_data is None:
            return None
        
        # 가속도 변환 (16-bit, ±2g)
        accel_scale = 2.0 / 32768.0
        accel_x = raw_data['accel'][0] * accel_scale
        accel_y = raw_data['accel'][1] * accel_scale
        accel_z = raw_data['accel'][2] * accel_scale
        
        # 자이로 변환 (16-bit, ±250°/s)
        gyro_scale = 250.0 / 32768.0
        gyro_x = raw_data['gyro'][0] * gyro_scale
        gyro_y = raw_data['gyro'][1] * gyro_scale
        gyro_z = raw_data['gyro'][2] * gyro_scale
        
        # 온도 변환
        temp = raw_data['temp'] / 340.0 + 36.53
        
        # 자기계 변환 (16-bit)
        mag_scale = 4912.0 / 32768.0
        mag_x = raw_data['mag'][0] * mag_scale
        mag_y = raw_data['mag'][1] * mag_scale
        mag_z = raw_data['mag'][2] * mag_scale
        
        return {
            'accel': (accel_x, accel_y, accel_z),
            'gyro': (gyro_x, gyro_y, gyro_z),
            'temp': temp,
            'mag': (mag_x, mag_y, mag_z)
        }
    
    def calculate_angles(self, data):
        """가속도와 자이로 데이터로부터 각도 계산"""
        if data is None:
            return None
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 가속도로부터 롤과 피치 계산
        accel_x, accel_y, accel_z = data['accel']
        
        # 중력 가속도로 정규화
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if accel_magnitude > 0:
            accel_x /= accel_magnitude
            accel_y /= accel_magnitude
            accel_z /= accel_magnitude
        
        # 롤과 피치 계산 (가속도 기반)
        roll_accel = math.atan2(accel_y, accel_z) * 180.0 / math.pi
        pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180.0 / math.pi
        
        # 자이로 데이터로 각속도 계산
        gyro_x, gyro_y, gyro_z = data['gyro']
        
        # 자이로 적분 (간단한 적분)
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt
        
        # 각도 정규화 (-180 ~ 180)
        self.yaw = self.normalize_angle(self.yaw)
        self.roll = self.normalize_angle(self.roll)
        self.pitch = self.normalize_angle(self.pitch)
        
        return {
            'yaw': self.yaw,
            'pitch': self.pitch,
            'roll': self.roll,
            'dt': dt
        }
    
    def normalize_angle(self, angle):
        """각도를 -180 ~ 180 범위로 정규화"""
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle
    
    def smooth_angle(self, new_angle):
        """각도 스무딩"""
        if len(self.angle_history) == 0:
            self.angle_history.append(new_angle)
            return new_angle
        
        smoothed = self.smoothing_factor * self.angle_history[-1] + (1 - self.smoothing_factor) * new_angle
        self.angle_history.append(smoothed)
        return smoothed
    
    def calculate_angle_error(self, current, target):
        """각도 오차 계산"""
        error = target - current
        
        # 각도 차이를 -180 ~ 180 범위로 정규화
        while error > 180.0:
            error -= 360.0
        while error < -180.0:
            error += 360.0
        
        return error
    
    def calculate_pid_output(self, error, dt):
        """PID 제어 출력 계산"""
        # 적분 항
        self.pid_integral += error * dt
        
        # 미분 항
        derivative = (error - self.pid_previous_error) / dt if dt > 0 else 0
        
        # PID 출력 계산
        output = (self.pid_kp * error + 
                 self.pid_ki * self.pid_integral + 
                 self.pid_kd * derivative)
        
        # 적분 와인드업 방지
        self.pid_integral = max(-100.0, min(100.0, self.pid_integral))
        
        # 출력 제한
        output = max(-100.0, min(100.0, output))
        
        self.pid_previous_error = error
        return output
    
    def test_rotation_control(self):
        """회전 제어 테스트"""
        print("회전 제어 테스트 시작...")
        print("초기화 중...")
        
        if not self.initialize_mpu9250():
            print("MPU9250 초기화 실패")
            return False
        
        if not self.initialize_ak8963():
            print("AK8963 초기화 실패 (자기계 기능 제한)")
        
        print("센서 초기화 완료")
        print("회전 제어 테스트를 시작합니다.")
        print("명령어:")
        print("  'left_90' - 좌회전 90도 (휠 1개)")
        print("  'right_90' - 우회전 90도 (휠 1개)")
        print("  'turn_180' - 회전 180도 (휠 1개)")
        print("  'target <angle>' - 특정 각도로 이동")
        print("  'quit' - 종료")
        print()
        
        try:
            while True:
                # 센서 데이터 읽기
                raw_data = self.read_raw_data()
                if raw_data is None:
                    continue
                
                # 물리적 단위로 변환
                data = self.convert_to_physical_units(raw_data)
                if data is None:
                    continue
                
                # 각도 계산
                angles = self.calculate_angles(data)
                if angles is None:
                    continue
                
                # 각도 스무딩
                self.current_angle = self.smooth_angle(angles['yaw'])
                
                # 각도 오차 계산
                self.angle_error = self.calculate_angle_error(self.current_angle, self.target_angle)
                
                # PID 제어 출력 계산
                control_output = self.calculate_pid_output(self.angle_error, angles['dt'])
                
                # 상태 출력
                print(f"\r현재 각도: {self.current_angle:6.2f}° | "
                      f"목표 각도: {self.target_angle:6.2f}° | "
                      f"오차: {self.angle_error:6.2f}° | "
                      f"제어 출력: {control_output:6.2f} | "
                      f"휠 1개 회전 모드", end="")
                
                # 사용자 입력 확인
                if self.check_user_input():
                    break
                
                time.sleep(0.02)  # 50Hz 제어 주파수
                
        except KeyboardInterrupt:
            print("\n테스트 종료")
        
        return True
    
    def check_user_input(self):
        """사용자 입력 확인 (비동기)"""
        import select
        import sys
        
        if select.select([sys.stdin], [], [], 0.0)[0]:
            command = input().strip().lower()
            
            if command == 'left_90':
                self.initial_angle = self.current_angle
                self.target_angle = self.normalize_angle(self.current_angle + 90.0)
                print(f"\n좌회전 90도 명령: 초기 {self.initial_angle:.2f}° → 목표 {self.target_angle:.2f}°")
                
            elif command == 'right_90':
                self.initial_angle = self.current_angle
                self.target_angle = self.normalize_angle(self.current_angle - 90.0)
                print(f"\n우회전 90도 명령: 초기 {self.initial_angle:.2f}° → 목표 {self.target_angle:.2f}°")
                
            elif command == 'turn_180':
                self.initial_angle = self.current_angle
                self.target_angle = self.normalize_angle(self.current_angle + 180.0)
                print(f"\n회전 180도 명령: 초기 {self.initial_angle:.2f}° → 목표 {self.target_angle:.2f}°")
                
            elif command.startswith('target '):
                try:
                    angle = float(command.split()[1])
                    self.initial_angle = self.current_angle
                    self.target_angle = self.normalize_angle(angle)
                    print(f"\n특정 각도 이동: 초기 {self.initial_angle:.2f}° → 목표 {self.target_angle:.2f}°")
                except (ValueError, IndexError):
                    print("\n잘못된 명령입니다. 'target <angle>' 형식으로 입력하세요.")
                
            elif command == 'quit':
                return True
                
            else:
                print(f"\n알 수 없는 명령: {command}")
        
        return False
    
    def test_rotation_sequence(self):
        """회전 시퀀스 테스트"""
        print("회전 시퀀스 테스트 시작...")
        
        if not self.initialize_mpu9250():
            return False
        
        # 초기 각도 설정
        print("초기 각도를 0도로 설정합니다. 로봇을 정면으로 향하게 하세요.")
        input("준비되면 Enter를 누르세요...")
        
        # 초기 각도 읽기
        for _ in range(50):  # 1초간 평균
            raw_data = self.read_raw_data()
            if raw_data:
                data = self.convert_to_physical_units(raw_data)
                if data:
                    angles = self.calculate_angles(data)
                    if angles:
                        self.current_angle = angles['yaw']
            time.sleep(0.02)
        
        self.initial_angle = self.current_angle
        print(f"초기 각도 설정: {self.current_angle:.2f}°")
        
        # 회전 시퀀스 테스트
        rotations = [
            ("좌회전 90도", 90.0),
            ("우회전 90도", -90.0),
            ("회전 180도", 180.0),
            ("우회전 90도", -90.0),
            ("좌회전 90도", 90.0)
        ]
        
        for rotation_name, angle in rotations:
            print(f"\n{rotation_name} 테스트")
            self.initial_angle = self.current_angle
            self.target_angle = self.normalize_angle(self.initial_angle + angle)
            print(f"목표 각도: {self.target_angle:.2f}°")
            
            start_time = time.time()
            while abs(self.angle_error) > self.angle_tolerance:
                raw_data = self.read_raw_data()
                if raw_data:
                    data = self.convert_to_physical_units(raw_data)
                    if data:
                        angles = self.calculate_angles(data)
                        if angles:
                            self.current_angle = self.smooth_angle(angles['yaw'])
                            self.angle_error = self.calculate_angle_error(self.current_angle, self.target_angle)
                            
                            print(f"\r현재: {self.current_angle:.2f}° | 목표: {self.target_angle:.2f}° | 오차: {self.angle_error:.2f}°", end="")
                            
                            if time.time() - start_time > 10.0:  # 10초 타임아웃
                                print("\n타임아웃!")
                                break
                
                time.sleep(0.02)
            
            print(f"\n{rotation_name} 완료! 최종 각도: {self.current_angle:.2f}°")
            time.sleep(1)  # 다음 회전 전 대기
        
        print("회전 시퀀스 테스트 완료!")
        return True

def main():
    print("IMU 기반 회전 제어 테스트")
    print("=" * 50)
    
    # 테스트 모드 선택
    print("테스트 모드를 선택하세요:")
    print("1. 대화형 회전 제어 테스트")
    print("2. 회전 시퀀스 테스트")
    
    try:
        choice = input("선택 (1 또는 2): ").strip()
        
        test = RotationControlTest()
        
        if choice == "1":
            test.test_rotation_control()
        elif choice == "2":
            test.test_rotation_sequence()
        else:
            print("잘못된 선택입니다.")
            
    except KeyboardInterrupt:
        print("\n테스트 중단")
    except Exception as e:
        print(f"오류 발생: {e}")

if __name__ == "__main__":
    main() 