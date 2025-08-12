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
        
        # MPU6050 레지스터 정의
        self.PWR_MGMT_1 = 0x6B
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43
        self.TEMP_OUT_H = 0x41
        
        # 회전 감지 설정
        self.target_angle = 90.0  # 목표 회전 각도 (도)
        self.angle_tolerance = 2.0  # 각도 허용 오차 (도)
        self.rotation_threshold = 5.0  # 회전 감지 임계값 (도/초)
        self.stable_threshold = 1.0  # 안정 상태 임계값 (도/초)
        
        # 데이터 히스토리
        self.yaw_history = deque(maxlen=10)
        self.yaw_rate_history = deque(maxlen=10)
        
        # 회전 상태
        self.rotation_in_progress = False
        self.rotation_start_angle = 0.0
        self.rotation_start_time = 0
        self.total_rotation = 0.0
        self.yaw_reference = 0.0
        
    def initialize(self):
        """IMU 센서 초기화"""
        try:
            # WHO_AM_I 확인
            who_am_i = self.bus.read_byte_data(self.mpu_address, 0x75)
            if who_am_i != 0x68:
                print(f"WHO_AM_I 오류: 0x{who_am_i:02X} (예상: 0x68)")
                return False
            
            # 전원 관리 설정
            self.bus.write_byte_data(self.mpu_address, self.PWR_MGMT_1, 0x00)
            
            # 샘플링 레이트 설정 (1kHz)
            self.bus.write_byte_data(self.mpu_address, 0x19, 0x07)
            
            # DLPF 설정 (42Hz)
            self.bus.write_byte_data(self.mpu_address, self.CONFIG, 0x03)
            
            # 자이로스코프 설정 (±250°/s)
            self.bus.write_byte_data(self.mpu_address, self.GYRO_CONFIG, 0x00)
            
            # 가속도계 설정 (±2g)
            self.bus.write_byte_data(self.mpu_address, self.ACCEL_CONFIG, 0x00)
            
            print("IMU 센서 초기화 완료")
            return True
            
        except Exception as e:
            print(f"IMU 초기화 오류: {e}")
            return False
    
    def read_raw_data(self):
        """원시 센서 데이터 읽기"""
        try:
            # 14바이트 읽기 (가속도 6바이트 + 온도 2바이트 + 자이로 6바이트)
            data = self.bus.read_i2c_block_data(self.mpu_address, self.ACCEL_XOUT_H, 14)
            
            # 가속도 데이터 (16비트)
            accel_x = (data[0] << 8) | data[1]
            accel_y = (data[2] << 8) | data[3]
            accel_z = (data[4] << 8) | data[5]
            
            # 온도 데이터
            temp = (data[6] << 8) | data[7]
            
            # 자이로 데이터 (16비트)
            gyro_x = (data[8] << 8) | data[9]
            gyro_y = (data[10] << 8) | data[11]
            gyro_z = (data[12] << 8) | data[13]
            
            return {
                'accel': (accel_x, accel_y, accel_z),
                'temp': temp,
                'gyro': (gyro_x, gyro_y, gyro_z)
            }
            
        except Exception as e:
            print(f"데이터 읽기 오류: {e}")
            return None
    
    def convert_data(self, raw_data):
        """원시 데이터를 물리적 단위로 변환"""
        if raw_data is None:
            return None
        
        # 가속도 변환 (±2g 범위)
        accel_scale = 16384.0
        accel_x = raw_data['accel'][0] / accel_scale
        accel_y = raw_data['accel'][1] / accel_scale
        accel_z = raw_data['accel'][2] / accel_scale
        
        # 온도 변환
        temp = raw_data['temp'] / 340.0 + 36.53
        
        # 자이로 변환 (±250°/s 범위)
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
        """가속도와 자이로 데이터로부터 요 각도 계산"""
        # 간단한 자이로 적분 (실제로는 칼만 필터 사용 권장)
        if len(self.yaw_history) > 0:
            dt = 0.02  # 50Hz 샘플링
            yaw_rate = gyro_data[2]  # Z축 자이로
            prev_yaw = self.yaw_history[-1]
            current_yaw = prev_yaw + yaw_rate * dt
            
            # 각도 정규화 (-180 ~ 180)
            while current_yaw > 180:
                current_yaw -= 360
            while current_yaw < -180:
                current_yaw += 360
        else:
            current_yaw = 0.0
        
        return current_yaw
    
    def detect_rotation(self, yaw, yaw_rate):
        """회전 감지"""
        current_time = time.time() * 1000  # 밀리초
        
        # 회전 상태 판단
        if abs(yaw_rate) > self.rotation_threshold and not self.rotation_in_progress:
            # 회전 시작
            self.rotation_in_progress = True
            self.rotation_start_angle = yaw
            self.rotation_start_time = current_time
            print(f"회전 시작: {yaw:.1f}도, 각속도: {yaw_rate:.1f}도/초")
            
        elif abs(yaw_rate) < self.stable_threshold and self.rotation_in_progress:
            # 회전 종료
            self.rotation_in_progress = False
            rotation_end_angle = yaw
            rotation_duration = current_time - self.rotation_start_time
            
            # 총 회전 각도 계산
            angle_diff = rotation_end_angle - self.rotation_start_angle
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360
            
            self.total_rotation = abs(angle_diff)
            
            print(f"회전 종료: {rotation_end_angle:.1f}도")
            print(f"총 회전 각도: {self.total_rotation:.1f}도")
            print(f"회전 시간: {rotation_duration:.0f}ms")
            
            # 목표 각도 도달 확인
            if abs(self.total_rotation - self.target_angle) <= self.angle_tolerance:
                print(f"✅ 목표 각도({self.target_angle}도) 도달 성공!")
                print(f"실제 회전: {self.total_rotation:.1f}도")
                return True
            else:
                print(f"❌ 목표 각도 미도달. 목표: {self.target_angle}도, 실제: {self.total_rotation:.1f}도")
                return False
        
        return False
    
    def calibrate_yaw(self):
        """요 각도 캘리브레이션"""
        print("요 각도 캘리브레이션 시작...")
        
        # 여러 샘플 수집
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
            print(f"캘리브레이션 완료: {self.yaw_reference:.1f}도")
            return True
        else:
            print("캘리브레이션 실패")
            return False
    
    def continuous_monitoring(self, duration=60):
        """연속 모니터링"""
        print(f"연속 모니터링 시작 ({duration}초)")
        print("90도 회전을 수행하세요...")
        
        start_time = time.time()
        target_reached = False
        
        while time.time() - start_time < duration:
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                if converted_data:
                    yaw = self.calculate_yaw(converted_data['accel'], converted_data['gyro'])
                    yaw_rate = converted_data['gyro'][2]
                    
                    # 히스토리 업데이트
                    self.yaw_history.append(yaw)
                    self.yaw_rate_history.append(yaw_rate)
                    
                    # 회전 감지
                    if self.detect_rotation(yaw, yaw_rate):
                        target_reached = True
                        break
                    
                    # 현재 상태 출력
                    if len(self.yaw_history) % 50 == 0:  # 1초마다 출력
                        print(f"현재 요: {yaw:.1f}도, 각속도: {yaw_rate:.1f}도/초")
            
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
            
            # 회전 대기
            while not self.rotation_in_progress:
                raw_data = self.read_raw_data()
                if raw_data:
                    converted_data = self.convert_data(raw_data)
                    if converted_data:
                        yaw = self.calculate_yaw(converted_data['accel'], converted_data['gyro'])
                        yaw_rate = converted_data['gyro'][2]
                        self.detect_rotation(yaw, yaw_rate)
                time.sleep(0.02)
            
            # 회전 완료 대기
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
            
            time.sleep(2)  # 다음 테스트 전 대기
        
        # 결과 분석
        if results:
            mean_rotation = np.mean(results)
            std_rotation = np.std(results)
            accuracy = 1.0 - abs(mean_rotation - 90.0) / 90.0
            
            print(f"\n=== 정확도 테스트 결과 ===")
            print(f"평균 회전 각도: {mean_rotation:.1f}도")
            print(f"표준편차: {std_rotation:.1f}도")
            print(f"정확도: {accuracy:.1%}")
            print(f"개별 결과: {[f'{r:.1f}도' for r in results]}")
        
        return results
    
    def print_data(self, data):
        """센서 데이터 출력"""
        if data:
            accel = data['accel']
            gyro = data['gyro']
            temp = data['temp']
            
            print(f"가속도: X={accel[0]:6.2f}, Y={accel[1]:6.2f}, Z={accel[2]:6.2f} g")
            print(f"자이로:  X={gyro[0]:6.2f}, Y={gyro[1]:6.2f}, Z={gyro[2]:6.2f} °/s")
            print(f"온도: {temp:.1f}°C")
    
    def run_all_tests(self):
        """모든 테스트 실행"""
        print("=== IMU 90도 회전 감지 테스트 ===")
        
        # I2C 버스 스캔
        print("\n1. I2C 버스 스캔...")
        for addr in range(0x08, 0x78):
            try:
                self.bus.read_byte_data(addr, 0)
                print(f"발견된 장치: 0x{addr:02X}")
            except:
                pass
        
        # IMU 초기화
        print("\n2. IMU 센서 초기화...")
        if not self.initialize():
            print("IMU 초기화 실패")
            return
        
        # 캘리브레이션
        print("\n3. 요 각도 캘리브레이션...")
        if not self.calibrate_yaw():
            print("캘리브레이션 실패")
            return
        
        # 데이터 읽기 테스트
        print("\n4. 센서 데이터 읽기 테스트...")
        for i in range(5):
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                print(f"샘플 {i+1}:")
                self.print_data(converted_data)
            time.sleep(0.5)
        
        # 연속 모니터링
        print("\n5. 90도 회전 감지 테스트...")
        self.continuous_monitoring(30)
        
        # 정확도 테스트
        print("\n6. 회전 정확도 테스트...")
        self.test_rotation_accuracy()
        
        print("\n=== 테스트 완료 ===")

def main():
    # I2C 권한 확인
    try:
        test = RotationDetectionTest()
        test.run_all_tests()
    except PermissionError:
        print("I2C 권한 오류. 다음 명령을 실행하세요:")
        print("sudo chmod 666 /dev/i2c-1")
    except Exception as e:
        print(f"오류 발생: {e}")

if __name__ == "__main__":
    main() 