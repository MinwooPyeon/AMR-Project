#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 센서 테스트 스크립트
"""

import time
import smbus
import math
import threading
from typing import Dict, List, Tuple

class IMUTest:
    """IMU 센서 테스트 클래스"""
    
    def __init__(self, bus_num=1, address=0x68):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        self.connected = False
        
        # MPU6050 레지스터
        self.REG_PWR_MGMT_1 = 0x6B
        self.REG_WHO_AM_I = 0x75
        self.REG_CONFIG = 0x1A
        self.REG_GYRO_CONFIG = 0x1B
        self.REG_ACCEL_CONFIG = 0x1C
        self.REG_SMPLRT_DIV = 0x19
        self.REG_INT_ENABLE = 0x38
        self.REG_ACCEL_XOUT_H = 0x3B
        self.REG_GYRO_XOUT_H = 0x43
        self.REG_TEMP_OUT_H = 0x41
        
        print(f"IMU 테스트 초기화 (주소: 0x{address:02X})")
    
    def initialize(self) -> bool:
        """IMU 센서 초기화"""
        try:
            # WHO_AM_I 확인
            who_am_i = self.bus.read_byte_data(self.address, self.REG_WHO_AM_I)
            if who_am_i != 0x68:
                print(f"WHO_AM_I 불일치: 0x{who_am_i:02X} (예상: 0x68)")
                return False
            
            print("WHO_AM_I 확인 완료")
            
            # 파워 매니지먼트 리셋
            self.bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            
            # 클록 소스 설정
            self.bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x01)
            
            # 샘플레이트 설정 (125Hz)
            self.bus.write_byte_data(self.address, self.REG_SMPLRT_DIV, 0x07)
            
            # 설정 레지스터 (DLPF)
            self.bus.write_byte_data(self.address, self.REG_CONFIG, 0x06)
            
            # 자이로스코프 설정 (±250°/s)
            self.bus.write_byte_data(self.address, self.REG_GYRO_CONFIG, 0x00)
            
            # 가속도계 설정 (±2g)
            self.bus.write_byte_data(self.address, self.REG_ACCEL_CONFIG, 0x00)
            
            # 인터럽트 활성화
            self.bus.write_byte_data(self.address, self.REG_INT_ENABLE, 0x01)
            
            self.connected = True
            print("IMU 센서 초기화 완료")
            return True
            
        except Exception as e:
            print(f"IMU 초기화 실패: {e}")
            return False
    
    def read_raw_data(self) -> Dict[str, int]:
        """원시 데이터 읽기"""
        try:
            # 14바이트 데이터 읽기
            data = self.bus.read_i2c_block_data(self.address, self.REG_ACCEL_XOUT_H, 14)
            
            # 데이터 변환
            raw_data = {
                'accel_x': (data[0] << 8) | data[1],
                'accel_y': (data[2] << 8) | data[3],
                'accel_z': (data[4] << 8) | data[5],
                'temp': (data[6] << 8) | data[7],
                'gyro_x': (data[8] << 8) | data[9],
                'gyro_y': (data[10] << 8) | data[11],
                'gyro_z': (data[12] << 8) | data[13]
            }
            
            return raw_data
            
        except Exception as e:
            print(f"데이터 읽기 실패: {e}")
            return {}
    
    def convert_data(self, raw_data: Dict[str, int]) -> Dict[str, float]:
        """원시 데이터를 실제 값으로 변환"""
        if not raw_data:
            return {}
        
        # 스케일 팩터
        accel_scale = 16384.0  # ±2g
        gyro_scale = 131.0     # ±250°/s
        temp_scale = 340.0      # 온도 스케일
        
        converted = {
            'accel_x': raw_data['accel_x'] / accel_scale * 9.81,  # m/s²
            'accel_y': raw_data['accel_y'] / accel_scale * 9.81,
            'accel_z': raw_data['accel_z'] / accel_scale * 9.81,
            'gyro_x': raw_data['gyro_x'] / gyro_scale,  # 도/초
            'gyro_y': raw_data['gyro_y'] / gyro_scale,
            'gyro_z': raw_data['gyro_z'] / gyro_scale,
            'temperature': raw_data['temp'] / temp_scale + 36.53,  # 섭씨
            'accel_magnitude': math.sqrt(
                (raw_data['accel_x'] / accel_scale * 9.81) ** 2 +
                (raw_data['accel_y'] / accel_scale * 9.81) ** 2 +
                (raw_data['accel_z'] / accel_scale * 9.81) ** 2
            )
        }
        
        return converted
    
    def print_data(self, data: Dict[str, float]):
        """데이터 출력"""
        print(f"\n=== IMU 데이터 ===")
        print(f"가속도 (m/s²): X={data.get('accel_x', 0):.2f}, Y={data.get('accel_y', 0):.2f}, Z={data.get('accel_z', 0):.2f}")
        print(f"각속도 (도/초): X={data.get('gyro_x', 0):.2f}, Y={data.get('gyro_y', 0):.2f}, Z={data.get('gyro_z', 0):.2f}")
        print(f"온도: {data.get('temperature', 0):.1f}°C")
        print(f"가속도 크기: {data.get('accel_magnitude', 0):.2f} m/s²")
        print("=" * 20)
    
    def continuous_read(self, duration: int = 10):
        """지속적인 데이터 읽기"""
        print(f"\n지속적인 데이터 읽기 시작 ({duration}초)")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                self.print_data(converted_data)
            else:
                print("데이터 읽기 실패")
            
            time.sleep(0.1)  # 100ms 간격
    
    def calibration_test(self):
        """캘리브레이션 테스트"""
        print("\n=== 캘리브레이션 테스트 ===")
        print("센서를 평평한 곳에 놓고 움직이지 마세요...")
        
        # 정지 상태에서 데이터 수집
        samples = []
        for i in range(50):  # 5초간 50개 샘플
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                samples.append(converted_data)
            time.sleep(0.1)
        
        if samples:
            # 평균값 계산
            avg_accel_x = sum(s['accel_x'] for s in samples) / len(samples)
            avg_accel_y = sum(s['accel_y'] for s in samples) / len(samples)
            avg_accel_z = sum(s['accel_z'] for s in samples) / len(samples)
            avg_gyro_x = sum(s['gyro_x'] for s in samples) / len(samples)
            avg_gyro_y = sum(s['gyro_y'] for s in samples) / len(samples)
            avg_gyro_z = sum(s['gyro_z'] for s in samples) / len(samples)
            
            print(f"\n캘리브레이션 결과:")
            print(f"가속도 오프셋: X={avg_accel_x:.3f}, Y={avg_accel_y:.3f}, Z={avg_accel_z:.3f}")
            print(f"자이로 오프셋: X={avg_gyro_x:.3f}, Y={avg_gyro_y:.3f}, Z={avg_gyro_z:.3f}")
            
            # 중력 가속도 확인
            gravity = math.sqrt(avg_accel_x**2 + avg_accel_y**2 + avg_accel_z**2)
            print(f"중력 가속도: {gravity:.2f} m/s² (예상: 9.81)")
    
    def motion_test(self):
        """모션 테스트"""
        print("\n=== 모션 테스트 ===")
        print("센서를 다양한 방향으로 움직여보세요...")
        
        for i in range(30):  # 3초간
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                
                # 모션 감지
                accel_mag = converted_data.get('accel_magnitude', 0)
                gyro_mag = math.sqrt(
                    converted_data.get('gyro_x', 0)**2 +
                    converted_data.get('gyro_y', 0)**2 +
                    converted_data.get('gyro_z', 0)**2
                )
                
                if accel_mag > 12.0 or gyro_mag > 10.0:
                    print(f"모션 감지! 가속도: {accel_mag:.2f}, 각속도: {gyro_mag:.2f}")
                
                self.print_data(converted_data)
            
            time.sleep(0.1)
    
    def run_all_tests(self):
        """모든 테스트 실행"""
        print("🚀 IMU 센서 테스트 시작")
        
        try:
            # 1. 초기화 테스트
            if not self.initialize():
                print("❌ IMU 센서 초기화 실패")
                return
            
            print("✅ IMU 센서 초기화 성공")
            
            # 2. 기본 데이터 읽기 테스트
            print("\n=== 기본 데이터 읽기 테스트 ===")
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                self.print_data(converted_data)
                print("✅ 데이터 읽기 성공")
            else:
                print("❌ 데이터 읽기 실패")
                return
            
            # 3. 캘리브레이션 테스트
            self.calibration_test()
            
            # 4. 모션 테스트
            self.motion_test()
            
            # 5. 지속적인 데이터 읽기
            self.continuous_read(5)
            
            print("\n✅ 모든 테스트 완료!")
            
        except KeyboardInterrupt:
            print("\n⚠️ 테스트 중단됨")
        except Exception as e:
            print(f"\n❌ 테스트 오류: {e}")
        finally:
            print("테스트 종료")

def main():
    """메인 함수"""
    # I2C 주소 스캔
    print("I2C 장치 스캔 중...")
    bus = smbus.SMBus(1)
    found_devices = []
    
    for addr in range(0x08, 0x78):
        try:
            bus.read_byte(addr)
            found_devices.append(addr)
            print(f"발견된 장치: 0x{addr:02X}")
        except:
            pass
    
    if not found_devices:
        print("❌ I2C 장치를 찾을 수 없습니다")
        return
    
    # IMU 테스트 실행
    imu_test = IMUTest(bus_num=1, address=0x68)
    imu_test.run_all_tests()

if __name__ == "__main__":
    main() 