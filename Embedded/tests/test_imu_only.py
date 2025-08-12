#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 전용 테스트 스크립트
"""

import smbus
import time
import math

class IMUTester:
    def __init__(self, i2c_bus=None):
        self.imu_address = 0x68  # MPU6050 주소
        self.imu_bus = None
        
        # I2C 버스 찾기
        if i2c_bus is None:
            i2c_bus = self._find_imu_bus()
        
        if i2c_bus is None:
            print("❌ IMU를 찾을 수 없습니다!")
            return
        
        try:
            self.imu_bus = smbus.SMBus(i2c_bus)
            print(f"✅ IMU 연결 성공 - 버스 {i2c_bus}")
            self._initialize_imu()
        except Exception as e:
            print(f"❌ IMU 초기화 실패: {e}")
    
    def _find_imu_bus(self):
        """IMU가 연결된 I2C 버스 찾기"""
        print("🔍 IMU 연결된 I2C 버스 찾는 중...")
        
        for bus_num in range(10):
            try:
                bus = smbus.SMBus(bus_num)
                
                # MPU6050 초기화 테스트
                try:
                    bus.write_byte_data(self.imu_address, 0x6B, 0)  # PWR_MGMT_1
                    time.sleep(0.1)
                    
                    # WHO_AM_I 레지스터 읽기 (0x75)
                    who_am_i = bus.read_byte_data(self.imu_address, 0x75)
                    if who_am_i == 0x68:  # MPU6050의 WHO_AM_I 값
                        print(f"✅ IMU 발견 - 버스 {bus_num}")
                        bus.close()
                        return bus_num
                    else:
                        print(f"❌ 버스 {bus_num}: 잘못된 WHO_AM_I 값 (0x{who_am_i:02X})")
                        bus.close()
                except:
                    bus.close()
                    continue
                    
            except Exception as e:
                print(f"❌ 버스 {bus_num} 접근 실패: {e}")
                continue
        
        print("❌ IMU를 찾을 수 없습니다!")
        return None
    
    def _initialize_imu(self):
        """IMU 초기화"""
        try:
            # PWR_MGMT_1 레지스터 초기화
            self.imu_bus.write_byte_data(self.imu_address, 0x6B, 0)
            time.sleep(0.1)
            
            # WHO_AM_I 확인
            who_am_i = self.imu_bus.read_byte_data(self.imu_address, 0x75)
            print(f"WHO_AM_I: 0x{who_am_i:02X} (예상: 0x68)")
            
            if who_am_i == 0x68:
                print("✅ IMU 초기화 성공")
            else:
                print("❌ IMU 초기화 실패 - 잘못된 WHO_AM_I 값")
                
        except Exception as e:
            print(f"❌ IMU 초기화 중 오류: {e}")
    
    def read_accelerometer(self):
        """가속도계 읽기"""
        try:
            # 가속도계 데이터 읽기 (0x3B-0x40)
            accel_data = self.imu_bus.read_i2c_block_data(self.imu_address, 0x3B, 6)
            
            # 16비트 값으로 변환
            accel_x = (accel_data[0] << 8) | accel_data[1]
            accel_y = (accel_data[2] << 8) | accel_data[3]
            accel_z = (accel_data[4] << 8) | accel_data[5]
            
            # 부호 있는 16비트로 변환
            if accel_x > 32767:
                accel_x -= 65536
            if accel_y > 32767:
                accel_y -= 65536
            if accel_z > 32767:
                accel_z -= 65536
            
            # g 단위로 변환 (16384 LSB/g)
            accel_x_g = accel_x / 16384.0
            accel_y_g = accel_y / 16384.0
            accel_z_g = accel_z / 16384.0
            
            return accel_x_g, accel_y_g, accel_z_g
            
        except Exception as e:
            print(f"❌ 가속도계 읽기 실패: {e}")
            return None, None, None
    
    def read_gyroscope(self):
        """자이로스코프 읽기"""
        try:
            # 자이로스코프 데이터 읽기 (0x43-0x48)
            gyro_data = self.imu_bus.read_i2c_block_data(self.imu_address, 0x43, 6)
            
            # 16비트 값으로 변환
            gyro_x = (gyro_data[0] << 8) | gyro_data[1]
            gyro_y = (gyro_data[2] << 8) | gyro_data[3]
            gyro_z = (gyro_data[4] << 8) | gyro_data[5]
            
            # 부호 있는 16비트로 변환
            if gyro_x > 32767:
                gyro_x -= 65536
            if gyro_y > 32767:
                gyro_y -= 65536
            if gyro_z > 32767:
                gyro_z -= 65536
            
            # deg/s 단위로 변환 (131 LSB/deg/s)
            gyro_x_dps = gyro_x / 131.0
            gyro_y_dps = gyro_y / 131.0
            gyro_z_dps = gyro_z / 131.0
            
            return gyro_x_dps, gyro_y_dps, gyro_z_dps
            
        except Exception as e:
            print(f"❌ 자이로스코프 읽기 실패: {e}")
            return None, None, None
    
    def calculate_yaw(self, gyro_z_dps, dt):
        """Yaw 각도 계산 (간단한 적분)"""
        return gyro_z_dps * dt
    
    def test_imu_continuous(self, duration=10):
        """IMU 연속 테스트"""
        print(f"\n🔄 IMU 연속 테스트 시작 ({duration}초)")
        print("=" * 60)
        
        start_time = time.time()
        prev_time = start_time
        yaw_angle = 0.0
        
        try:
            while time.time() - start_time < duration:
                current_time = time.time()
                dt = current_time - prev_time
                prev_time = current_time
                
                # 센서 데이터 읽기
                accel_x, accel_y, accel_z = self.read_accelerometer()
                gyro_x, gyro_y, gyro_z = self.read_gyroscope()
                
                if accel_x is not None and gyro_x is not None:
                    # Yaw 각도 계산
                    yaw_angle += self.calculate_yaw(gyro_z, dt)
                    
                    # 출력
                    elapsed = current_time - start_time
                    print(f"\r[{elapsed:5.1f}s] "
                          f"가속도: X={accel_x:6.2f}g Y={accel_y:6.2f}g Z={accel_z:6.2f}g | "
                          f"자이로: X={gyro_x:6.1f}°/s Y={gyro_y:6.1f}°/s Z={gyro_z:6.1f}°/s | "
                          f"Yaw={yaw_angle:6.1f}°", end='', flush=True)
                
                time.sleep(0.1)  # 100ms 간격
                
        except KeyboardInterrupt:
            print("\n\n⏹️ 사용자에 의해 중단됨")
        
        print(f"\n\n✅ IMU 테스트 완료")

def main():
    print("=== IMU 전용 테스트 ===")
    
    # IMU 테스터 생성
    imu_tester = IMUTester()
    
    if imu_tester.imu_bus is None:
        print("❌ IMU 테스트를 진행할 수 없습니다.")
        return
    
    print("\n📋 테스트 옵션:")
    print("1. 단일 센서 읽기")
    print("2. 연속 센서 읽기 (10초)")
    print("3. 연속 센서 읽기 (30초)")
    
    choice = input("\n선택 (1-3): ").strip()
    
    if choice == "1":
        # 단일 읽기
        print("\n📊 단일 센서 읽기:")
        accel_x, accel_y, accel_z = imu_tester.read_accelerometer()
        gyro_x, gyro_y, gyro_z = imu_tester.read_gyroscope()
        
        if accel_x is not None:
            print(f"가속도: X={accel_x:.2f}g, Y={accel_y:.2f}g, Z={accel_z:.2f}g")
            print(f"자이로: X={gyro_x:.1f}°/s, Y={gyro_y:.1f}°/s, Z={gyro_z:.1f}°/s")
        else:
            print("❌ 센서 읽기 실패")
    
    elif choice == "2":
        # 10초 연속 테스트
        imu_tester.test_imu_continuous(10)
    
    elif choice == "3":
        # 30초 연속 테스트
        imu_tester.test_imu_continuous(30)
    
    else:
        print("❌ 잘못된 선택")

if __name__ == "__main__":
    main()
