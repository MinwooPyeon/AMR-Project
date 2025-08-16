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
        
        self.MPU9250_ADDR = imu_address
        self.PWR_MGMT_1 = 0x6B
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        self.TEMP_OUT_H = 0x41
        
        self.AK8963_ADDR = 0x0C
        self.AK8963_ST1 = 0x02
        self.AK8963_XOUT_L = 0x03
        self.AK8963_CNTL1 = 0x0A
        self.AK8963_ASAX = 0x10
        
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()
        
        self.angle_history = deque(maxlen=10)
        self.smoothing_factor = 0.8
        
        self.pid_kp = 2.0
        self.pid_ki = 0.1
        self.pid_kd = 0.5
        self.pid_integral = 0.0
        self.pid_previous_error = 0.0
        
        self.angle_tolerance = 2.0
        self.target_angle = 0.0
        self.current_angle = 0.0
        self.angle_error = 0.0
        self.initial_angle = 0.0
        
        print("Rotation control test initialization complete")
    
    def initialize_mpu9250(self):
        try:
            self.bus.write_byte_data(self.MPU9250_ADDR, self.PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.CONFIG, 0x06)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.GYRO_CONFIG, 0x00)
            
            self.bus.write_byte_data(self.MPU9250_ADDR, self.ACCEL_CONFIG, 0x00)
            
            print("MPU9250 initialization complete")
            return True
            
        except Exception as e:
            print(f"MPU9250 initialization failed: {e}")
            return False
    
    def initialize_ak8963(self):
        try:
            self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL1, 0x00)
            time.sleep(0.1)
            self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL1, 0x16)
            time.sleep(0.1)
            
            print("AK8963 initialization complete")
            return True
            
        except Exception as e:
            print(f"AK8963 initialization failed: {e}")
            return False
    
    def read_raw_data(self):
        try:
            accel_data = self.bus.read_i2c_block_data(self.MPU9250_ADDR, self.ACCEL_XOUT_H, 6)
            accel_x = (accel_data[0] << 8) | accel_data[1]
            accel_y = (accel_data[2] << 8) | accel_data[3]
            accel_z = (accel_data[4] << 8) | accel_data[5]
            
            gyro_data = self.bus.read_i2c_block_data(self.MPU9250_ADDR, self.GYRO_XOUT_H, 6)
            gyro_x = (gyro_data[0] << 8) | gyro_data[1]
            gyro_y = (gyro_data[2] << 8) | gyro_data[3]
            gyro_z = (gyro_data[4] << 8) | gyro_data[5]
            
            temp_data = self.bus.read_i2c_block_data(self.MPU9250_ADDR, self.TEMP_OUT_H, 2)
            temp = (temp_data[0] << 8) | temp_data[1]
            
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
            print(f"Sensor data reading failed: {e}")
            return None
    
    def convert_to_physical_units(self, raw_data):
        if raw_data is None:
            return None
        
        accel_scale = 2.0 / 32768.0
        accel_x = raw_data['accel'][0] * accel_scale
        accel_y = raw_data['accel'][1] * accel_scale
        accel_z = raw_data['accel'][2] * accel_scale
        
        gyro_scale = 250.0 / 32768.0
        gyro_x = raw_data['gyro'][0] * gyro_scale
        gyro_y = raw_data['gyro'][1] * gyro_scale
        gyro_z = raw_data['gyro'][2] * gyro_scale
        
        temp = raw_data['temp'] / 340.0 + 36.53
        
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
        if data is None:
            return None
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        accel_x, accel_y, accel_z = data['accel']
        
        accel_magnitude = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        if accel_magnitude > 0:
            accel_x /= accel_magnitude
            accel_y /= accel_magnitude
            accel_z /= accel_magnitude
        
        roll_accel = math.atan2(accel_y, accel_z) * 180.0 / math.pi
        pitch_accel = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180.0 / math.pi
        
        gyro_x, gyro_y, gyro_z = data['gyro']
        
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt
        
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
        error = target - current
        
        while error > 180.0:
            error -= 360.0
        while error < -180.0:
            error += 360.0
        
        return error
    
    def calculate_pid_output(self, error, dt):
        self.pid_integral += error * dt
        
        derivative = (error - self.pid_previous_error) / dt if dt > 0 else 0
        
        output = (self.pid_kp * error + 
                 self.pid_ki * self.pid_integral + 
                 self.pid_kd * derivative)
        
        self.pid_integral = max(-100.0, min(100.0, self.pid_integral))
        
        output = max(-100.0, min(100.0, output))
        
        self.pid_previous_error = error
        return output
    
    def test_rotation_control(self):
        print("Rotation control test started...")
        print("Initializing...")
        
        if not self.initialize_mpu9250():
            print("MPU9250 initialization failed")
            return False
        
        if not self.initialize_ak8963():
            print("AK8963 initialization failed (magnetometer function limited)")
        
        print("Sensor initialization complete")
        print("Starting rotation control test.")
        print("Commands:")
        print("  'left_90' - Turn left 90 degrees (single wheel)")
        print("  'right_90' - Turn right 90 degrees (single wheel)")
        print("  'turn_180' - Turn 180 degrees (single wheel)")
        print("  'target <angle>' - Move to specific angle")
        print("  'quit' - Exit")
        print()
        
        try:
            while True:
                raw_data = self.read_raw_data()
                if raw_data is None:
                    continue
                
                data = self.convert_to_physical_units(raw_data)
                if data is None:
                    continue
                
                angles = self.calculate_angles(data)
                if angles is None:
                    continue
                
                self.current_angle = self.smooth_angle(angles['yaw'])
                
                self.angle_error = self.calculate_angle_error(self.current_angle, self.target_angle)
                
                control_output = self.calculate_pid_output(self.angle_error, angles['dt'])
                
                print(f"\rCurrent angle: {self.current_angle:6.2f}° | "
                      f"Target angle: {self.target_angle:6.2f}° | "
                      f"Error: {self.angle_error:6.2f}° | "
                      f"Control output: {control_output:6.2f} | "
                      f"Single wheel rotation mode", end="")
                
                if self.check_user_input():
                    break
                
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\nTest terminated")
        
        return True
    
    def check_user_input(self):
        import select
        import sys
        
        if select.select([sys.stdin], [], [], 0.0)[0]:
            command = input().strip().lower()
            
            if command == 'left_90':
                self.initial_angle = self.current_angle
                self.target_angle = self.normalize_angle(self.current_angle + 90.0)
                print(f"\nLeft turn 90 degrees command: Initial {self.initial_angle:.2f}° → Target {self.target_angle:.2f}°")
                
            elif command == 'right_90':
                self.initial_angle = self.current_angle
                self.target_angle = self.normalize_angle(self.current_angle - 90.0)
                print(f"\nRight turn 90 degrees command: Initial {self.initial_angle:.2f}° → Target {self.target_angle:.2f}°")
                
            elif command == 'turn_180':
                self.initial_angle = self.current_angle
                self.target_angle = self.normalize_angle(self.current_angle + 180.0)
                print(f"\nTurn 180 degrees command: Initial {self.initial_angle:.2f}° → Target {self.target_angle:.2f}°")
                
            elif command.startswith('target '):
                try:
                    angle = float(command.split()[1])
                    self.initial_angle = self.current_angle
                    self.target_angle = self.normalize_angle(angle)
                    print(f"\nMove to specific angle: Initial {self.initial_angle:.2f}° → Target {self.target_angle:.2f}°")
                except (ValueError, IndexError):
                    print("\nInvalid command. Please enter in 'target <angle>' format.")
                
            elif command == 'quit':
                return True
                
            else:
                print(f"\nUnknown command: {command}")
        
        return False
    
    def test_rotation_sequence(self):
        print("Rotation sequence test started...")
        
        if not self.initialize_mpu9250():
            return False
        
        print("Setting initial angle to 0 degrees. Please face the robot forward.")
        input("Press Enter when ready...")
        
        for _ in range(50):
            raw_data = self.read_raw_data()
            if raw_data:
                data = self.convert_to_physical_units(raw_data)
                if data:
                    angles = self.calculate_angles(data)
                    if angles:
                        self.current_angle = angles['yaw']
            time.sleep(0.02)
        
        self.initial_angle = self.current_angle
        print(f"Initial angle set: {self.current_angle:.2f}°")
        
        rotations = [
            ("Left turn 90 degrees", 90.0),
            ("Right turn 90 degrees", -90.0),
            ("Turn 180 degrees", 180.0),
            ("Right turn 90 degrees", -90.0),
            ("Left turn 90 degrees", 90.0)
        ]
        
        for rotation_name, angle in rotations:
            print(f"\n{rotation_name} test")
            self.initial_angle = self.current_angle
            self.target_angle = self.normalize_angle(self.initial_angle + angle)
            print(f"Target angle: {self.target_angle:.2f}°")
            
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
                            
                            print(f"\rCurrent: {self.current_angle:.2f}° | Target: {self.target_angle:.2f}° | Error: {self.angle_error:.2f}°", end="")
                            
                            if time.time() - start_time > 10.0:
                                print("\nTimeout!")
                                break
                
                time.sleep(0.02)
            
            print(f"\n{rotation_name} complete! Final angle: {self.current_angle:.2f}°")
            time.sleep(1)
        
        print("Rotation sequence test complete!")
        return True

def main():
    print("IMU-based Rotation Control Test")
    print("=" * 50)
    
    print("Select test mode:")
    print("1. Interactive rotation control test")
    print("2. Rotation sequence test")
    
    try:
        choice = input("Select (1 or 2): ").strip()
        
        test = RotationControlTest()
        
        if choice == "1":
            test.test_rotation_control()
        elif choice == "2":
            test.test_rotation_sequence()
        else:
            print("Invalid selection.")
            
    except KeyboardInterrupt:
        print("\nTest interrupted")
    except Exception as e:
        print(f"Error occurred: {e}")

if __name__ == "__main__":
    main() 