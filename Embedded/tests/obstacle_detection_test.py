#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 기반 장애물 감지 테스트 스크립트
"""

import smbus
import time
import math
import numpy as np
from collections import deque
import threading
import signal
import sys

class ObstacleDetectionTest:
    def __init__(self, bus_num=1, mpu9250_addr=0x68, ak8963_addr=0x0C):
        self.bus = smbus.SMBus(bus_num)
        self.mpu9250_addr = mpu9250_addr
        self.ak8963_addr = ak8963_addr
        
        self.WHO_AM_I = 0x75
        self.PWR_MGMT_1 = 0x6B
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.SMPLRT_DIV = 0x19
        self.INT_ENABLE = 0x38
        self.INT_STATUS = 0x3A
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        self.TEMP_OUT_H = 0x41
        self.INT_PIN_CFG = 0x37
        
        self.AK8963_WHO_AM_I = 0x00
        self.AK8963_ST1 = 0x02
        self.AK8963_XOUT_L = 0x03
        self.AK8963_CNTL = 0x0A
        self.AK8963_ASAX = 0x10
        
        self.acceleration_threshold = 2.0
        self.gyro_threshold = 50.0
        self.magnetic_threshold = 100.0
        self.window_size = 10
        self.detection_confidence = 0.7
        
        self.data_history = deque(maxlen=self.window_size)
        self.mag_calibration = [1.0, 1.0, 1.0]
        
        self.detection_count = 0
        self.total_samples = 0
        
        self.accel_scale = 16384.0
        self.gyro_scale = 131.0
        self.mag_scale = 10.0 * 4912.0 / 32760.0
        
        self.running = False
        
    def initialize(self):
        try:
            print("Initializing IMU sensor...")
            
            who_am_i = self.bus.read_byte_data(self.mpu9250_addr, self.WHO_AM_I)
            print(f"MPU9250 WHO_AM_I: 0x{who_am_i:02X} (expected: 0x71)")
            
            if who_am_i != 0x71:
                print("MPU9250 connection failed!")
                return False
            
            self.bus.write_byte_data(self.mpu9250_addr, self.PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            self.bus.write_byte_data(self.mpu9250_addr, self.PWR_MGMT_1, 0x01)
            time.sleep(0.1)
            
            self.bus.write_byte_data(self.mpu9250_addr, self.SMPLRT_DIV, 0x04)
            
            self.bus.write_byte_data(self.mpu9250_addr, self.CONFIG, 0x03)
            
            self.bus.write_byte_data(self.mpu9250_addr, self.GYRO_CONFIG, 0x00)
            
            self.bus.write_byte_data(self.mpu9250_addr, self.ACCEL_CONFIG, 0x00)
            
            self.bus.write_byte_data(self.mpu9250_addr, self.INT_PIN_CFG, 0x22)
            self.bus.write_byte_data(self.mpu9250_addr, self.INT_ENABLE, 0x01)
            
            if not self.initialize_ak8963():
                print("AK8963 initialization failed, proceeding without magnetometer")
            
            print("IMU sensor initialization complete")
            return True
            
        except Exception as e:
            print(f"Initialization error: {e}")
            return False
    
    def initialize_ak8963(self):
        """AK8963 자력계 초기화"""
        try:
            who_am_i = self.bus.read_byte_data(self.ak8963_addr, self.AK8963_WHO_AM_I)
            print(f"AK8963 WHO_AM_I: 0x{who_am_i:02X} (expected: 0x48)")
            
            if who_am_i != 0x48:
                return False
            
            self.bus.write_byte_data(self.ak8963_addr, self.AK8963_CNTL, 0x00)
            time.sleep(0.01)
            
            self.bus.write_byte_data(self.ak8963_addr, self.AK8963_CNTL, 0x0F)
            time.sleep(0.01)
            
            asa = []
            for i in range(3):
                asa_val = self.bus.read_byte_data(self.ak8963_addr, self.AK8963_ASAX + i)
                asa.append(asa_val)
            
            for i in range(3):
                self.mag_calibration[i] = (asa[i] - 128) / 256.0 + 1.0
            
            print(f"Magnetometer calibration: {self.mag_calibration}")
            
            self.bus.write_byte_data(self.ak8963_addr, self.AK8963_CNTL, 0x00)
            time.sleep(0.01)
            
            self.bus.write_byte_data(self.ak8963_addr, self.AK8963_CNTL, 0x16)
            
            print("AK8963 magnetometer initialization complete")
            return True
            
        except Exception as e:
            print(f"AK8963 initialization error: {e}")
            return False
    
    def read_raw_data(self):
        try:
            data = self.bus.read_i2c_block_data(self.mpu9250_addr, self.ACCEL_XOUT_H, 14)
            
            accel_x = (data[0] << 8) | data[1]
            accel_y = (data[2] << 8) | data[3]
            accel_z = (data[4] << 8) | data[5]
            temp = (data[6] << 8) | data[7]
            gyro_x = (data[8] << 8) | data[9]
            gyro_y = (data[10] << 8) | data[11]
            gyro_z = (data[12] << 8) | data[13]
            
            mag_x = mag_y = mag_z = 0
            try:
                st1 = self.bus.read_byte_data(self.ak8963_addr, self.AK8963_ST1)
                if st1 & 0x01:
                    mag_data = self.bus.read_i2c_block_data(self.ak8963_addr, self.AK8963_XOUT_L, 7)
                    st2 = mag_data[6]
                    if not (st2 & 0x08):
                        mag_x = (mag_data[1] << 8) | mag_data[0]
                        mag_y = (mag_data[3] << 8) | mag_data[2]
                        mag_z = (mag_data[5] << 8) | mag_data[4]
            except:
                pass
            
            return {
                'accel': [accel_x, accel_y, accel_z],
                'gyro': [gyro_x, gyro_y, gyro_z],
                'mag': [mag_x, mag_y, mag_z],
                'temp': temp
            }
            
        except Exception as e:
            print(f"Data reading error: {e}")
            return None
    
    def convert_data(self, raw_data):
        if raw_data is None:
            return None
        
        accel_x = raw_data['accel'][0] / self.accel_scale * 9.81
        accel_y = raw_data['accel'][1] / self.accel_scale * 9.81
        accel_z = raw_data['accel'][2] / self.accel_scale * 9.81
        
        gyro_x = raw_data['gyro'][0] / self.gyro_scale
        gyro_y = raw_data['gyro'][1] / self.gyro_scale
        gyro_z = raw_data['gyro'][2] / self.gyro_scale
        
        mag_x = raw_data['mag'][0] * self.mag_scale * self.mag_calibration[0]
        mag_y = raw_data['mag'][1] * self.mag_scale * self.mag_calibration[1]
        mag_z = raw_data['mag'][2] * self.mag_scale * self.mag_calibration[2]
        
        temp = raw_data['temp'] / 340.0 + 36.53
        
        return {
            'accel': [accel_x, accel_y, accel_z],
            'gyro': [gyro_x, gyro_y, gyro_z],
            'mag': [mag_x, mag_y, mag_z],
            'temp': temp,
            'timestamp': time.time()
        }
    
    def calculate_magnitude(self, data):
        """벡터 크기 계산"""
        return math.sqrt(sum(x*x for x in data))
    
    def detect_obstacle(self, data):
        if len(self.data_history) < self.window_size:
            return False, 0.0, 0.0
        
        accel_mag = self.calculate_magnitude(data['accel'])
        gyro_mag = self.calculate_magnitude(data['gyro'])
        mag_mag = self.calculate_magnitude(data['mag'])
        
        accel_history = [self.calculate_magnitude(d['accel']) for d in self.data_history]
        gyro_history = [self.calculate_magnitude(d['gyro']) for d in self.data_history]
        mag_history = [self.calculate_magnitude(d['mag']) for d in self.data_history]
        
        accel_avg = np.mean(accel_history)
        gyro_avg = np.mean(gyro_history)
        mag_avg = np.mean(mag_history)
        
        accel_anomaly = abs(accel_mag - accel_avg) > self.acceleration_threshold
        gyro_anomaly = abs(gyro_mag - gyro_avg) > self.gyro_threshold
        mag_anomaly = abs(mag_mag - mag_avg) > self.magnetic_threshold
        
        obstacle_detected = accel_anomaly or gyro_anomaly or mag_anomaly
        
        confidence = 0.0
        if accel_anomaly: confidence += 0.4
        if gyro_anomaly: confidence += 0.3
        if mag_anomaly: confidence += 0.3
        
        distance = 0.0
        if obstacle_detected:
            accel_distance = accel_mag * 0.1
            gyro_distance = gyro_mag * 0.01
            mag_distance = mag_mag / 1000.0
            
            distance = (accel_distance * 0.5 + gyro_distance * 0.3 + mag_distance * 0.2)
        
        return obstacle_detected, confidence, distance
    
    def print_data(self, data, obstacle_info):
        """데이터 출력"""
        detected, confidence, distance = obstacle_info
        
        print(f"\n{'='*60}")
        print(f"Time: {time.strftime('%H:%M:%S')}")
        print(f"Temperature: {data['temp']:.1f}°C")
        
        print(f"\nAcceleration (m/s²):")
        print(f"  X: {data['accel'][0]:8.3f}  Y: {data['accel'][1]:8.3f}  Z: {data['accel'][2]:8.3f}")
        print(f"  Magnitude: {self.calculate_magnitude(data['accel']):.3f}")
        
        print(f"\nGyroscope (deg/s):")
        print(f"  X: {data['gyro'][0]:8.3f}  Y: {data['gyro'][1]:8.3f}  Z: {data['gyro'][2]:8.3f}")
        print(f"  Magnitude: {self.calculate_magnitude(data['gyro']):.3f}")
        
        print(f"\nMagnetometer (μT):")
        print(f"  X: {data['mag'][0]:8.3f}  Y: {data['mag'][1]:8.3f}  Z: {data['mag'][2]:8.3f}")
        print(f"  Magnitude: {self.calculate_magnitude(data['mag']):.3f}")
        
        if detected:
            print(f"\nObstacle detected!")
            print(f"  Confidence: {confidence:.2f}")
            print(f"  Estimated distance: {distance:.3f}m")
            self.detection_count += 1
        
        print(f"\nStatistics: Detected {self.detection_count} times / Total {self.total_samples} times")
    
    def continuous_monitoring(self):
        print("Obstacle detection monitoring started... (Press Ctrl+C to exit)")
        print(f"Thresholds: Acceleration {self.acceleration_threshold} m/s², Gyro {self.gyro_threshold} deg/s, Magnetic {self.magnetic_threshold} μT")
        
        self.running = True
        
        try:
            while self.running:
                raw_data = self.read_raw_data()
                if raw_data is None:
                    continue
                
                data = self.convert_data(raw_data)
                if data is None:
                    continue
                
                self.data_history.append(data)
                self.total_samples += 1
                
                obstacle_info = self.detect_obstacle(data)
                
                self.print_data(data, obstacle_info)
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nMonitoring terminated")
        except Exception as e:
            print(f"Monitoring error: {e}")
        finally:
            self.running = False
    
    def calibration_test(self):
        print("Calibration test started...")
        print("Please keep the sensor stable.")
        
        samples = []
        for i in range(100):
            raw_data = self.read_raw_data()
            if raw_data:
                data = self.convert_data(raw_data)
                if data:
                    samples.append(data)
            time.sleep(0.01)
        
        if samples:
            accel_means = np.mean([s['accel'] for s in samples], axis=0)
            gyro_means = np.mean([s['gyro'] for s in samples], axis=0)
            mag_means = np.mean([s['mag'] for s in samples], axis=0)
            
            accel_stds = np.std([s['accel'] for s in samples], axis=0)
            gyro_stds = np.std([s['gyro'] for s in samples], axis=0)
            mag_stds = np.std([s['mag'] for s in samples], axis=0)
            
            print(f"\nCalibration results:")
            print(f"Acceleration mean: {accel_means}")
            print(f"Acceleration std: {accel_stds}")
            print(f"Gyro mean: {gyro_means}")
            print(f"Gyro std: {gyro_stds}")
            print(f"Magnetic mean: {mag_means}")
            print(f"Magnetic std: {mag_stds}")
    
    def motion_test(self):
        print("Motion test started...")
        print("Move the sensor in various directions.")
        
        for i in range(50):
            raw_data = self.read_raw_data()
            if raw_data:
                data = self.convert_data(raw_data)
                if data:
                    accel_mag = self.calculate_magnitude(data['accel'])
                    gyro_mag = self.calculate_magnitude(data['gyro'])
                    mag_mag = self.calculate_magnitude(data['mag'])
                    
                    print(f"Acceleration: {accel_mag:.3f} m/s², Gyro: {gyro_mag:.3f} deg/s, Magnetic: {mag_mag:.3f} μT")
            
            time.sleep(0.1)
    
    def run_all_tests(self):
        if not self.initialize():
            print("Initialization failed!")
            return
        
        print("\n1. Calibration test")
        self.calibration_test()
        
        print("\n2. Motion test")
        self.motion_test()
        
        print("\n3. Continuous monitoring")
        self.continuous_monitoring()

def signal_handler(sig, frame):
    print('\nProgram terminated')
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    print("IMU-based Obstacle Detection Test")
    print("=" * 50)
    
    print("Scanning I2C bus...")
    bus = smbus.SMBus(1)
    devices = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            devices.append(addr)
        except:
            pass
    
    print(f"Found I2C devices: {[hex(addr) for addr in devices]}")
    
    detector = ObstacleDetectionTest()
    detector.run_all_tests()

if __name__ == "__main__":
    main() 