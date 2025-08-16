#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 센서 90도 회전 감지 테스트 스크립트
"""

import smbus
import time
import math
import numpy as np
from collections import deque

class RotationDetectionTest:
    def __init__(self, bus_num=1, mpu_address=0x68):
        self.bus = smbus.SMBus(bus_num)
        self.mpu_address = mpu_address
        
        self.PWR_MGMT_1 = 0x6B
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        self.TEMP_OUT_H = 0x41
        
        self.target_angle = 90.0
        self.angle_tolerance = 2.0
        self.rotation_threshold = 5.0
        self.stable_threshold = 1.0
        
        self.yaw_history = deque(maxlen=10)
        self.yaw_rate_history = deque(maxlen=10)
        
        self.rotation_in_progress = False
        self.rotation_start_angle = 0.0
        self.rotation_start_time = 0
        self.total_rotation = 0.0
        self.yaw_reference = 0.0
        
    def initialize(self):
        try:
            who_am_i = self.bus.read_byte_data(self.mpu_address, 0x75)
            if who_am_i != 0x68:
                print(f"WHO_AM_I error: 0x{who_am_i:02X} (expected: 0x68)")
                return False
            
            self.bus.write_byte_data(self.mpu_address, self.PWR_MGMT_1, 0x00)
            
            self.bus.write_byte_data(self.mpu_address, 0x19, 0x07)
            
            self.bus.write_byte_data(self.mpu_address, self.CONFIG, 0x03)
            
            self.bus.write_byte_data(self.mpu_address, self.GYRO_CONFIG, 0x00)
            
            self.bus.write_byte_data(self.mpu_address, self.ACCEL_CONFIG, 0x00)
            
            print("IMU sensor initialization complete")
            return True
            
        except Exception as e:
            print(f"IMU initialization error: {e}")
            return False
    
    def read_raw_data(self):
        try:
            data = self.bus.read_i2c_block_data(self.mpu_address, self.ACCEL_XOUT_H, 14)
            
            accel_x = (data[0] << 8) | data[1]
            accel_y = (data[2] << 8) | data[3]
            accel_z = (data[4] << 8) | data[5]
            
            temp = (data[6] << 8) | data[7]
            
            gyro_x = (data[8] << 8) | data[9]
            gyro_y = (data[10] << 8) | data[11]
            gyro_z = (data[12] << 8) | data[13]
            
            return {
                'accel': (accel_x, accel_y, accel_z),
                'temp': temp,
                'gyro': (gyro_x, gyro_y, gyro_z)
            }
            
        except Exception as e:
            print(f"Data reading error: {e}")
            return None
    
    def convert_data(self, raw_data):
        if raw_data is None:
            return None
        
        accel_scale = 16384.0
        accel_x = raw_data['accel'][0] / accel_scale
        accel_y = raw_data['accel'][1] / accel_scale
        accel_z = raw_data['accel'][2] / accel_scale
        
        temp = raw_data['temp'] / 340.0 + 36.53
        
        gyro_scale = 131.0
        gyro_x = raw_data['gyro'][0] / gyro_scale
        gyro_y = raw_data['gyro'][1] / gyro_scale
        gyro_z = raw_data['gyro'][2] / gyro_scale
        
        return {
            'accel': (accel_x, accel_y, accel_z),
            'temp': temp,
            'gyro': (gyro_x, gyro_y, gyro_z)
        }
    
    def calculate_yaw(self, accel_data, gyro_data):
        if len(self.yaw_history) > 0:
            dt = 0.02
            yaw_rate = gyro_data[2]
            prev_yaw = self.yaw_history[-1]
            current_yaw = prev_yaw + yaw_rate * dt
            
            while current_yaw > 180:
                current_yaw -= 360
            while current_yaw < -180:
                current_yaw += 360
        else:
            current_yaw = 0.0
        
        return current_yaw
    
    def detect_rotation(self, yaw, yaw_rate):
        current_time = time.time() * 1000
        
        if abs(yaw_rate) > self.rotation_threshold and not self.rotation_in_progress:
            self.rotation_in_progress = True
            self.rotation_start_angle = yaw
            self.rotation_start_time = current_time
            print(f"Rotation started: {yaw:.1f} degrees, angular velocity: {yaw_rate:.1f} deg/s")
            
        elif abs(yaw_rate) < self.stable_threshold and self.rotation_in_progress:
            self.rotation_in_progress = False
            rotation_end_angle = yaw
            rotation_duration = current_time - self.rotation_start_time
            
            angle_diff = rotation_end_angle - self.rotation_start_angle
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360
            
            self.total_rotation = abs(angle_diff)
            
            print(f"Rotation ended: {rotation_end_angle:.1f} degrees")
            print(f"Total rotation angle: {self.total_rotation:.1f} degrees")
            print(f"Rotation time: {rotation_duration:.0f}ms")
            
            if abs(self.total_rotation - self.target_angle) <= self.angle_tolerance:
                print(f"Target angle ({self.target_angle} degrees) reached successfully!")
                print(f"Actual rotation: {self.total_rotation:.1f} degrees")
                return True
            else:
                print(f"Target angle not reached. Target: {self.target_angle} degrees, Actual: {self.total_rotation:.1f} degrees")
                return False
        
        return False
    
    def calibrate_yaw(self):
        print("Yaw angle calibration started...")
        
        samples = []
        for i in range(20):
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                if converted_data:
                    yaw = self.calculate_yaw(converted_data['accel'], converted_data['gyro'])
                    samples.append(yaw)
            time.sleep(0.05)
        
        if samples:
            self.yaw_reference = np.mean(samples)
            print(f"Calibration complete: {self.yaw_reference:.1f} degrees")
            return True
        else:
            print("Calibration failed")
            return False
    
    def continuous_monitoring(self, duration=60):
        print(f"Continuous monitoring started ({duration} seconds)")
        print("Perform 90 degree rotation...")
        
        start_time = time.time()
        target_reached = False
        
        while time.time() - start_time < duration:
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                if converted_data:
                    yaw = self.calculate_yaw(converted_data['accel'], converted_data['gyro'])
                    yaw_rate = converted_data['gyro'][2]
                    
                    self.yaw_history.append(yaw)
                    self.yaw_rate_history.append(yaw_rate)
                    
                    if self.detect_rotation(yaw, yaw_rate):
                        target_reached = True
                        break
                    
                    if len(self.yaw_history) % 50 == 0:
                        print(f"Current yaw: {yaw:.1f} degrees, angular velocity: {yaw_rate:.1f} deg/s")
            
            time.sleep(0.02)  # 50Hz
        
        if target_reached:
            print("🎉 90도 회전 감지 성공!")
        else:
            print("⏰ 시간 초과")
        
        return target_reached
    
    def test_rotation_accuracy(self):
        """회전 정확도 테스트"""
        print("회전 정확도 테스트 시작")
        print("정확히 90도 회전을 여러 번 수행하세요...")
        
        results = []
        test_count = 5
        
        for i in range(test_count):
            print(f"\n--- 테스트 {i+1}/{test_count} ---")
            print("90도 회전을 수행하세요...")
            
            while not self.rotation_in_progress:
                raw_data = self.read_raw_data()
                if raw_data:
                    converted_data = self.convert_data(raw_data)
                    if converted_data:
                        yaw = self.calculate_yaw(converted_data['accel'], converted_data['gyro'])
                        yaw_rate = converted_data['gyro'][2]
                        self.detect_rotation(yaw, yaw_rate)
                time.sleep(0.02)
            
            while self.rotation_in_progress:
                raw_data = self.read_raw_data()
                if raw_data:
                    converted_data = self.convert_data(raw_data)
                    if converted_data:
                        yaw = self.calculate_yaw(converted_data['accel'], converted_data['gyro'])
                        yaw_rate = converted_data['gyro'][2]
                        if self.detect_rotation(yaw, yaw_rate):
                            results.append(self.total_rotation)
                            break
                time.sleep(0.02)
            
            time.sleep(2)
        
        if results:
            mean_rotation = np.mean(results)
            std_rotation = np.std(results)
            accuracy = 1.0 - abs(mean_rotation - 90.0) / 90.0
            
            print(f"\n=== Accuracy Test Results ===")
            print(f"Average rotation angle: {mean_rotation:.1f} degrees")
            print(f"Standard deviation: {std_rotation:.1f} degrees")
            print(f"Accuracy: {accuracy:.1%}")
            print(f"Individual results: {[f'{r:.1f} degrees' for r in results]}")
        
        return results
    
    def print_data(self, data):
        if data:
            accel = data['accel']
            gyro = data['gyro']
            temp = data['temp']
            
            print(f"Acceleration: X={accel[0]:6.2f}, Y={accel[1]:6.2f}, Z={accel[2]:6.2f} g")
            print(f"Gyroscope:  X={gyro[0]:6.2f}, Y={gyro[1]:6.2f}, Z={gyro[2]:6.2f} °/s")
            print(f"Temperature: {temp:.1f}°C")
    
    def run_all_tests(self):
        print("=== IMU 90 Degree Rotation Detection Test ===")
        
        print("\n1. I2C bus scan...")
        for addr in range(0x08, 0x78):
            try:
                self.bus.read_byte_data(addr, 0)
                print(f"Found device: 0x{addr:02X}")
            except:
                pass
        
        print("\n2. IMU sensor initialization...")
        if not self.initialize():
            print("IMU initialization failed")
            return
        
        print("\n3. Yaw angle calibration...")
        if not self.calibrate_yaw():
            print("Calibration failed")
            return
        
        print("\n4. Sensor data reading test...")
        for i in range(5):
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                print(f"Sample {i+1}:")
                self.print_data(converted_data)
            time.sleep(0.5)
        
        print("\n5. 90 degree rotation detection test...")
        self.continuous_monitoring(30)
        
        print("\n6. Rotation accuracy test...")
        self.test_rotation_accuracy()
        
        print("\n=== Test Complete ===")

def main():
    try:
        test = RotationDetectionTest()
        test.run_all_tests()
    except PermissionError:
        print("I2C permission error. Please run the following command:")
        print("sudo chmod 666 /dev/i2c-1")
    except Exception as e:
        print(f"Error occurred: {e}")

if __name__ == "__main__":
    main() 