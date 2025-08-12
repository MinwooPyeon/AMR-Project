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
        
        # MPU9250 레지스터 정의
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
        
        # AK8963 레지스터 정의
        self.AK8963_WHO_AM_I = 0x00
        self.AK8963_ST1 = 0x02
        self.AK8963_XOUT_L = 0x03
        self.AK8963_CNTL = 0x0A
        self.AK8963_ASAX = 0x10
        
        # 장애물 감지 설정
        self.acceleration_threshold = 2.0  # m/s²
        self.gyro_threshold = 50.0        # deg/s
        self.magnetic_threshold = 100.0    # μT
        self.window_size = 10
        self.detection_confidence = 0.7
        
        # 데이터 히스토리
        self.data_history = deque(maxlen=self.window_size)
        self.mag_calibration = [1.0, 1.0, 1.0]
        
        # 통계
        self.detection_count = 0
        self.total_samples = 0
        
        # 스케일 팩터
        self.accel_scale = 16384.0  # ±2g
        self.gyro_scale = 131.0     # ±250°/s
        self.mag_scale = 10.0 * 4912.0 / 32760.0  # 16-bit, ±4900 μT
        
        self.running = False
        
    def initialize(self):
        """MPU9250 및 AK8963 초기화"""
        try:
            print("IMU 센서 초기화 중...")
            
            # MPU9250 WHO_AM_I 확인
            who_am_i = self.bus.read_byte_data(self.mpu9250_addr, self.WHO_AM_I)
            print(f"MPU9250 WHO_AM_I: 0x{who_am_i:02X} (예상: 0x71)")
            
            if who_am_i != 0x71:
                print("MPU9250 연결 실패!")
                return False
            
            # MPU9250 초기화
            self.bus.write_byte_data(self.mpu9250_addr, self.PWR_MGMT_1, 0x80)  # 리셋
            time.sleep(0.1)
            self.bus.write_byte_data(self.mpu9250_addr, self.PWR_MGMT_1, 0x01)  # 클록 소스 설정
            time.sleep(0.1)
            
            # 샘플레이트 설정
            self.bus.write_byte_data(self.mpu9250_addr, self.SMPLRT_DIV, 0x04)  # 200Hz
            
            # 설정 레지스터
            self.bus.write_byte_data(self.mpu9250_addr, self.CONFIG, 0x03)  # DLPF
            
            # 자이로스코프 설정 (±250°/s)
            self.bus.write_byte_data(self.mpu9250_addr, self.GYRO_CONFIG, 0x00)
            
            # 가속도계 설정 (±2g)
            self.bus.write_byte_data(self.mpu9250_addr, self.ACCEL_CONFIG, 0x00)
            
            # 인터럽트 설정
            self.bus.write_byte_data(self.mpu9250_addr, self.INT_PIN_CFG, 0x22)
            self.bus.write_byte_data(self.mpu9250_addr, self.INT_ENABLE, 0x01)
            
            # AK8963 자력계 초기화
            if not self.initialize_ak8963():
                print("AK8963 초기화 실패, 자력계 없이 진행")
            
            print("IMU 센서 초기화 완료")
            return True
            
        except Exception as e:
            print(f"초기화 오류: {e}")
            return False
    
    def initialize_ak8963(self):
        """AK8963 자력계 초기화"""
        try:
            # WHO_AM_I 확인
            who_am_i = self.bus.read_byte_data(self.ak8963_addr, self.AK8963_WHO_AM_I)
            print(f"AK8963 WHO_AM_I: 0x{who_am_i:02X} (예상: 0x48)")
            
            if who_am_i != 0x48:
                return False
            
            # 파워 다운
            self.bus.write_byte_data(self.ak8963_addr, self.AK8963_CNTL, 0x00)
            time.sleep(0.01)
            
            # Fuse ROM 액세스
            self.bus.write_byte_data(self.ak8963_addr, self.AK8963_CNTL, 0x0F)
            time.sleep(0.01)
            
            # 캘리브레이션 값 읽기
            asa = []
            for i in range(3):
                asa_val = self.bus.read_byte_data(self.ak8963_addr, self.AK8963_ASAX + i)
                asa.append(asa_val)
            
            # 캘리브레이션 값 계산
            for i in range(3):
                self.mag_calibration[i] = (asa[i] - 128) / 256.0 + 1.0
            
            print(f"자력계 캘리브레이션: {self.mag_calibration}")
            
            # 파워 다운
            self.bus.write_byte_data(self.ak8963_addr, self.AK8963_CNTL, 0x00)
            time.sleep(0.01)
            
            # 16비트 해상도, 8Hz로 설정
            self.bus.write_byte_data(self.ak8963_addr, self.AK8963_CNTL, 0x16)
            
            print("AK8963 자력계 초기화 완료")
            return True
            
        except Exception as e:
            print(f"AK8963 초기화 오류: {e}")
            return False
    
    def read_raw_data(self):
        """원시 센서 데이터 읽기"""
        try:
            # 14바이트 읽기 (가속도 6바이트 + 온도 2바이트 + 자이로 6바이트)
            data = self.bus.read_i2c_block_data(self.mpu9250_addr, self.ACCEL_XOUT_H, 14)
            
            # 데이터 변환
            accel_x = (data[0] << 8) | data[1]
            accel_y = (data[2] << 8) | data[3]
            accel_z = (data[4] << 8) | data[5]
            temp = (data[6] << 8) | data[7]
            gyro_x = (data[8] << 8) | data[9]
            gyro_y = (data[10] << 8) | data[11]
            gyro_z = (data[12] << 8) | data[13]
            
            # 자력계 데이터 읽기
            mag_x = mag_y = mag_z = 0
            try:
                st1 = self.bus.read_byte_data(self.ak8963_addr, self.AK8963_ST1)
                if st1 & 0x01:  # 데이터 준비
                    mag_data = self.bus.read_i2c_block_data(self.ak8963_addr, self.AK8963_XOUT_L, 7)
                    st2 = mag_data[6]
                    if not (st2 & 0x08):  # 오버플로우 없음
                        mag_x = (mag_data[1] << 8) | mag_data[0]
                        mag_y = (mag_data[3] << 8) | mag_data[2]
                        mag_z = (mag_data[5] << 8) | mag_data[4]
            except:
                pass  # 자력계 읽기 실패 시 무시
            
            return {
                'accel': [accel_x, accel_y, accel_z],
                'gyro': [gyro_x, gyro_y, gyro_z],
                'mag': [mag_x, mag_y, mag_z],
                'temp': temp
            }
            
        except Exception as e:
            print(f"데이터 읽기 오류: {e}")
            return None
    
    def convert_data(self, raw_data):
        """원시 데이터를 물리적 단위로 변환"""
        if raw_data is None:
            return None
        
        # 가속도 변환 (m/s²)
        accel_x = raw_data['accel'][0] / self.accel_scale * 9.81
        accel_y = raw_data['accel'][1] / self.accel_scale * 9.81
        accel_z = raw_data['accel'][2] / self.accel_scale * 9.81
        
        # 자이로스코프 변환 (deg/s)
        gyro_x = raw_data['gyro'][0] / self.gyro_scale
        gyro_y = raw_data['gyro'][1] / self.gyro_scale
        gyro_z = raw_data['gyro'][2] / self.gyro_scale
        
        # 자력계 변환 (μT)
        mag_x = raw_data['mag'][0] * self.mag_scale * self.mag_calibration[0]
        mag_y = raw_data['mag'][1] * self.mag_scale * self.mag_calibration[1]
        mag_z = raw_data['mag'][2] * self.mag_scale * self.mag_calibration[2]
        
        # 온도 변환 (°C)
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
        """장애물 감지"""
        if len(self.data_history) < self.window_size:
            return False, 0.0, 0.0
        
        # 현재 데이터
        accel_mag = self.calculate_magnitude(data['accel'])
        gyro_mag = self.calculate_magnitude(data['gyro'])
        mag_mag = self.calculate_magnitude(data['mag'])
        
        # 히스토리에서 평균 계산
        accel_history = [self.calculate_magnitude(d['accel']) for d in self.data_history]
        gyro_history = [self.calculate_magnitude(d['gyro']) for d in self.data_history]
        mag_history = [self.calculate_magnitude(d['mag']) for d in self.data_history]
        
        accel_avg = np.mean(accel_history)
        gyro_avg = np.mean(gyro_history)
        mag_avg = np.mean(mag_history)
        
        # 이상 감지
        accel_anomaly = abs(accel_mag - accel_avg) > self.acceleration_threshold
        gyro_anomaly = abs(gyro_mag - gyro_avg) > self.gyro_threshold
        mag_anomaly = abs(mag_mag - mag_avg) > self.magnetic_threshold
        
        # 장애물 감지
        obstacle_detected = accel_anomaly or gyro_anomaly or mag_anomaly
        
        # 신뢰도 계산
        confidence = 0.0
        if accel_anomaly: confidence += 0.4
        if gyro_anomaly: confidence += 0.3
        if mag_anomaly: confidence += 0.3
        
        # 거리 추정 (간단한 근사치)
        distance = 0.0
        if obstacle_detected:
            # 가속도 기반 거리 추정
            accel_distance = accel_mag * 0.1
            # 자이로 기반 거리 추정
            gyro_distance = gyro_mag * 0.01
            # 자기장 기반 거리 추정
            mag_distance = mag_mag / 1000.0
            
            distance = (accel_distance * 0.5 + gyro_distance * 0.3 + mag_distance * 0.2)
        
        return obstacle_detected, confidence, distance
    
    def print_data(self, data, obstacle_info):
        """데이터 출력"""
        detected, confidence, distance = obstacle_info
        
        print(f"\n{'='*60}")
        print(f"시간: {time.strftime('%H:%M:%S')}")
        print(f"온도: {data['temp']:.1f}°C")
        
        print(f"\n가속도 (m/s²):")
        print(f"  X: {data['accel'][0]:8.3f}  Y: {data['accel'][1]:8.3f}  Z: {data['accel'][2]:8.3f}")
        print(f"  크기: {self.calculate_magnitude(data['accel']):.3f}")
        
        print(f"\n자이로스코프 (deg/s):")
        print(f"  X: {data['gyro'][0]:8.3f}  Y: {data['gyro'][1]:8.3f}  Z: {data['gyro'][2]:8.3f}")
        print(f"  크기: {self.calculate_magnitude(data['gyro']):.3f}")
        
        print(f"\n자력계 (μT):")
        print(f"  X: {data['mag'][0]:8.3f}  Y: {data['mag'][1]:8.3f}  Z: {data['mag'][2]:8.3f}")
        print(f"  크기: {self.calculate_magnitude(data['mag']):.3f}")
        
        if detected:
            print(f"\n🚨 장애물 감지!")
            print(f"  신뢰도: {confidence:.2f}")
            print(f"  추정 거리: {distance:.3f}m")
            self.detection_count += 1
        
        print(f"\n통계: 감지 {self.detection_count}회 / 총 {self.total_samples}회")
    
    def continuous_monitoring(self):
        """연속 모니터링"""
        print("장애물 감지 모니터링 시작... (Ctrl+C로 종료)")
        print(f"임계값: 가속도 {self.acceleration_threshold} m/s², 자이로 {self.gyro_threshold} deg/s, 자기장 {self.magnetic_threshold} μT")
        
        self.running = True
        
        try:
            while self.running:
                # 데이터 읽기
                raw_data = self.read_raw_data()
                if raw_data is None:
                    continue
                
                # 데이터 변환
                data = self.convert_data(raw_data)
                if data is None:
                    continue
                
                # 히스토리 업데이트
                self.data_history.append(data)
                self.total_samples += 1
                
                # 장애물 감지
                obstacle_info = self.detect_obstacle(data)
                
                # 결과 출력
                self.print_data(data, obstacle_info)
                
                # 샘플링 간격
                time.sleep(0.1)  # 10Hz
                
        except KeyboardInterrupt:
            print("\n모니터링 종료")
        except Exception as e:
            print(f"모니터링 오류: {e}")
        finally:
            self.running = False
    
    def calibration_test(self):
        """캘리브레이션 테스트"""
        print("캘리브레이션 테스트 시작...")
        print("센서를 안정적으로 유지하세요.")
        
        samples = []
        for i in range(100):
            raw_data = self.read_raw_data()
            if raw_data:
                data = self.convert_data(raw_data)
                if data:
                    samples.append(data)
            time.sleep(0.01)
        
        if samples:
            # 평균 및 표준편차 계산
            accel_means = np.mean([s['accel'] for s in samples], axis=0)
            gyro_means = np.mean([s['gyro'] for s in samples], axis=0)
            mag_means = np.mean([s['mag'] for s in samples], axis=0)
            
            accel_stds = np.std([s['accel'] for s in samples], axis=0)
            gyro_stds = np.std([s['gyro'] for s in samples], axis=0)
            mag_stds = np.std([s['mag'] for s in samples], axis=0)
            
            print(f"\n캘리브레이션 결과:")
            print(f"가속도 평균: {accel_means}")
            print(f"가속도 표준편차: {accel_stds}")
            print(f"자이로 평균: {gyro_means}")
            print(f"자이로 표준편차: {gyro_stds}")
            print(f"자기장 평균: {mag_means}")
            print(f"자기장 표준편차: {mag_stds}")
    
    def motion_test(self):
        """모션 테스트"""
        print("모션 테스트 시작...")
        print("센서를 다양한 방향으로 움직여보세요.")
        
        for i in range(50):
            raw_data = self.read_raw_data()
            if raw_data:
                data = self.convert_data(raw_data)
                if data:
                    accel_mag = self.calculate_magnitude(data['accel'])
                    gyro_mag = self.calculate_magnitude(data['gyro'])
                    mag_mag = self.calculate_magnitude(data['mag'])
                    
                    print(f"가속도: {accel_mag:.3f} m/s², 자이로: {gyro_mag:.3f} deg/s, 자기장: {mag_mag:.3f} μT")
            
            time.sleep(0.1)
    
    def run_all_tests(self):
        """모든 테스트 실행"""
        if not self.initialize():
            print("초기화 실패!")
            return
        
        print("\n1. 캘리브레이션 테스트")
        self.calibration_test()
        
        print("\n2. 모션 테스트")
        self.motion_test()
        
        print("\n3. 연속 모니터링")
        self.continuous_monitoring()

def signal_handler(sig, frame):
    print('\n프로그램 종료')
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    print("IMU 기반 장애물 감지 테스트")
    print("=" * 50)
    
    # I2C 버스 스캔
    print("I2C 버스 스캔 중...")
    bus = smbus.SMBus(1)
    devices = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            devices.append(addr)
        except:
            pass
    
    print(f"발견된 I2C 장치: {[hex(addr) for addr in devices]}")
    
    # 테스트 실행
    detector = ObstacleDetectionTest()
    detector.run_all_tests()

if __name__ == "__main__":
    main() 