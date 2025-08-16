import time
import smbus
import math
import threading
from typing import Dict, List, Tuple

class IMUTest:
    
    def __init__(self, bus_num=1, address=0x68):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        self.connected = False
        
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
        
        print(f"IMU test initialization (address: 0x{address:02X})")
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
        try:
            who_am_i = self.bus.read_byte_data(self.address, self.REG_WHO_AM_I)
            if who_am_i != 0x68:
                print(f"WHO_AM_I mismatch: 0x{who_am_i:02X} (expected: 0x68)")
                return False
            
            print("WHO_AM_I verification completed")
            
            self.bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x80)
            time.sleep(0.1)
            
            self.bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x01)
            
            self.bus.write_byte_data(self.address, self.REG_SMPLRT_DIV, 0x07)
            
            self.bus.write_byte_data(self.address, self.REG_CONFIG, 0x06)
            
            self.bus.write_byte_data(self.address, self.REG_GYRO_CONFIG, 0x00)
            
            self.bus.write_byte_data(self.address, self.REG_ACCEL_CONFIG, 0x00)
            
            self.bus.write_byte_data(self.address, self.REG_INT_ENABLE, 0x01)
            
            self.connected = True
            print("IMU sensor initialization completed")
            return True
            
        except Exception as e:
            print(f"IMU 초기화 실패: {e}")
            return False
    
    def read_raw_data(self) -> Dict[str, int]:
        try:
            data = self.bus.read_i2c_block_data(self.address, self.REG_ACCEL_XOUT_H, 14)
            
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
            print(f"Data reading failed: {e}")
            return {}
    
    def convert_data(self, raw_data: Dict[str, int]) -> Dict[str, float]:
        if not raw_data:
            return {}
        
        accel_scale = 16384.0
        gyro_scale = 131.0
        temp_scale = 340.0
        
        converted = {
            'accel_x': raw_data['accel_x'] / accel_scale * 9.81,
            'accel_y': raw_data['accel_y'] / accel_scale * 9.81,
            'accel_z': raw_data['accel_z'] / accel_scale * 9.81,
            'gyro_x': raw_data['gyro_x'] / gyro_scale,
            'gyro_y': raw_data['gyro_y'] / gyro_scale,
            'gyro_z': raw_data['gyro_z'] / gyro_scale,
            'temperature': raw_data['temp'] / temp_scale + 36.53,
            'accel_magnitude': math.sqrt(
                (raw_data['accel_x'] / accel_scale * 9.81) ** 2 +
                (raw_data['accel_y'] / accel_scale * 9.81) ** 2 +
                (raw_data['accel_z'] / accel_scale * 9.81) ** 2
            )
        }
        
        return converted
    
    def print_data(self, data: Dict[str, float]):
        print(f"=== IMU Data ===")
        print(f"Acceleration (m/s²): X={data.get('accel_x', 0):.2f}, Y={data.get('accel_y', 0):.2f}, Z={data.get('accel_z', 0):.2f}")
        print(f"Angular velocity (deg/s): X={data.get('gyro_x', 0):.2f}, Y={data.get('gyro_y', 0):.2f}, Z={data.get('gyro_z', 0):.2f}")
        print(f"Temperature: {data.get('temperature', 0):.1f}°C")
        print(f"Acceleration magnitude: {data.get('accel_magnitude', 0):.2f} m/s²")
        print("=" * 20)
    
    def continuous_read(self, duration: int = 10):
        print(f"Starting continuous data reading ({duration} seconds)")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                self.print_data(converted_data)
            else:
                print("Data reading failed")
            
            time.sleep(0.1)
    
    def calibration_test(self):
        print("=== Calibration Test ===")
        print("Place the sensor on a flat surface and do not move...")
        
        samples = []
        for i in range(50):
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                samples.append(converted_data)
            time.sleep(0.1)
        
        if samples:
            avg_accel_x = sum(s['accel_x'] for s in samples) / len(samples)
            avg_accel_y = sum(s['accel_y'] for s in samples) / len(samples)
            avg_accel_z = sum(s['accel_z'] for s in samples) / len(samples)
            avg_gyro_x = sum(s['gyro_x'] for s in samples) / len(samples)
            avg_gyro_y = sum(s['gyro_y'] for s in samples) / len(samples)
            avg_gyro_z = sum(s['gyro_z'] for s in samples) / len(samples)
            
            print(f"Calibration results:")
            print(f"Acceleration offset: X={avg_accel_x:.3f}, Y={avg_accel_y:.3f}, Z={avg_accel_z:.3f}")
            print(f"Gyro offset: X={avg_gyro_x:.3f}, Y={avg_gyro_y:.3f}, Z={avg_gyro_z:.3f}")
            
            gravity = math.sqrt(avg_accel_x**2 + avg_accel_y**2 + avg_accel_z**2)
            print(f"Gravity acceleration: {gravity:.2f} m/s² (expected: 9.81)")
    
    def motion_test(self):
        print("=== Motion Test ===")
        print("Move the sensor in various directions...")
        
        for i in range(30):
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                
                accel_mag = converted_data.get('accel_magnitude', 0)
                gyro_mag = math.sqrt(
                    converted_data.get('gyro_x', 0)**2 +
                    converted_data.get('gyro_y', 0)**2 +
                    converted_data.get('gyro_z', 0)**2
                )
                
                if accel_mag > 12.0 or gyro_mag > 10.0:
                    print(f"Motion detected! Acceleration: {accel_mag:.2f}, Angular velocity: {gyro_mag:.2f}")
                
                self.print_data(converted_data)
            
            time.sleep(0.1)
    
    def run_all_tests(self):
        print("IMU sensor test started")
        
        try:
            if not self.initialize():
                print("IMU sensor initialization failed")
                return
            
            print("IMU sensor initialization successful")
            
            print("=== Basic Data Reading Test ===")
            raw_data = self.read_raw_data()
            if raw_data:
                converted_data = self.convert_data(raw_data)
                self.print_data(converted_data)
                print("Data reading successful")
            else:
                print("Data reading failed")
                return
            
            self.calibration_test()
            
            self.motion_test()
            
            self.continuous_read(5)
            
            print("All tests completed!")
            
        except KeyboardInterrupt:
            print("Test interrupted")
        except Exception as e:
            print(f"Test error: {e}")
        finally:
            print("Test finished")

def main():
    print("Scanning I2C devices...")
    bus = smbus.SMBus(1)
    found_devices = []
    
    for addr in range(0x08, 0x78):
        try:
            bus.read_byte(addr)
            found_devices.append(addr)
            print(f"Found device: 0x{addr:02X}")
        except:
            pass
    
    if not found_devices:
        print("No I2C devices found")
        return
    
    imu_test = IMUTest(bus_num=1, address=0x68)
    imu_test.run_all_tests()

if __name__ == "__main__":
    main() 