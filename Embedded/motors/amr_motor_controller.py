import time
import math
import smbus
import json
import requests
import threading
from datetime import datetime
from PCA9685 import PCA9685

COMMAND_TABLE = {
    0: 'STOP',
    1: 'MOVING_FORWARD', 
    2: 'MOVING_BACKWARD',
    3: 'ROTATE_LEFT',
    4: 'ROTATE_RIGHT'
}

class AMRMotorController:
    
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
        self.debug = debug
        self.motor_i2c_address = motor_i2c_address
        self.servo_i2c_address = servo_i2c_address
        self.imu_i2c_address = imu_i2c_address
        self.i2c_bus = i2c_bus
        self.api_url = api_url
        self.backend_broker = backend_broker
        self.backend_port = backend_port
        
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4
        
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        self.motor_a_direction = self.FORWARD
        self.motor_b_direction = self.FORWARD
        
        self.last_ai_data = None
        self.serial_number = "AMR001"
        
        self.backend_transmitter = None
        self.backend_connected = False
        
        self.imu_bus = None
        self.initial_angle = 0.0
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.angle_offset = 0.0
        self.is_turning = False
        self.turn_direction = 0
        self.imu_available = False
        
        self.kp = 2.0
        self.ki = 0.1
        self.kd = 0.5
        self.prev_error = 0.0
        self.integral = 0.0
        
        self.motor_speeds = {
            'forward': 300,
            'backward': 300,
            'left': 300,
            'right': 300,
            'stop': 0,
            'custom': 300
        }
        
        self.control_thread = None
        self.running = False
        self.thread_lock = threading.Lock()
        
        self._initialize_hardware()
        
    def _initialize_hardware(self):
        try:
            self.pwm = PCA9685(self.motor_i2c_address, debug=self.debug, i2c_bus=self.i2c_bus)
            self.pwm.setPWMFreq(20)
            
            try:
                self.servo_pwm = PCA9685(self.servo_i2c_address, debug=self.debug, i2c_bus=self.i2c_bus)
                self.servo_pwm.setPWMFreq(50)
                if self.debug:
                    print(f"Servo motor driver initialization successful - Address: 0x{self.servo_i2c_address:02X}")
            except Exception as e:
                if self.debug:
                    print(f"Servo motor driver initialization failed: {e}")
                self.servo_pwm = None
            
            self._initialize_imu()
            
            self.stop_all()
            
            if self.debug:
                self._print_initialization_info()
                
        except Exception as e:
            print(f"Hardware initialization failed: {e}")
            raise
            
    def _initialize_imu(self):
        try:
            self.imu_bus = self._find_imu_bus()
            if not self.imu_bus:
                raise Exception("No available I2C bus found")
            
            detected_address = self._find_imu_address()
            if detected_address:
                self.imu_i2c_address = detected_address
            
            bus = smbus.SMBus(self.imu_bus)
            bus.write_byte_data(self.imu_i2c_address, 0x6B, 0)
            
            print("IMU calibration in progress...")
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
                print(f"IMU initialization completed")
                print(f"  - I2C Bus: {self.imu_bus}")
                print(f"  - IMU Address: 0x{self.imu_i2c_address:02X}")
                print(f"  - Angle Offset: {self.angle_offset:.2f} degrees")
                
        except Exception as e:
            print(f"IMU initialization failed: {e}")
            print("Using motor control only without IMU.")
            self.imu_available = False
            
    def _find_imu_bus(self):
        import os
        
        for bus_num in range(10):
            bus_path = f"/dev/i2c-{bus_num}"
            if os.path.exists(bus_path):
                try:
                    bus = smbus.SMBus(bus_num)
                    bus.close()
                    if self.debug:
                        print(f"Available I2C bus found: {bus_num}")
                    return bus_num
                except Exception as e:
                    if self.debug:
                        print(f"Bus {bus_num} test failed: {e}")
                    continue
        return None
        
    def _find_imu_address(self):
        imu_addresses = [0x68, 0x69]
        
        for addr in imu_addresses:
            try:
                bus = smbus.SMBus(self.imu_bus)
                bus.write_byte_data(addr, 0x6B, 0)
                time.sleep(0.1)
                
                who_am_i = bus.read_byte_data(addr, 0x75)
                
                if who_am_i == 0x68:
                    if self.debug:
                        print(f"IMU sensor found: Address 0x{addr:02X}")
                    return addr
                else:
                    if self.debug:
                        print(f"Address 0x{addr:02X} WHO_AM_I: 0x{who_am_i:02X} (Expected: 0x68)")
                        
            except Exception as e:
                if self.debug:
                    print(f"Address 0x{addr:02X} access failed: {e}")
                continue
        
        return None
        
    def _read_imu_yaw(self):
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
                print(f"IMU read error: {e}")
            return 0.0
            
    def _convert_to_signed(self, value):
        if value > 32767:
            value -= 65536
        return value
        
    def _normalize_angle(self, angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
        
    def _calculate_angle_error(self, target, current):
        error = target - current
        
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        return error
        
    def _pid_control(self, error):
        with self.thread_lock:
            self.integral += error
            derivative = error - self.prev_error
            
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            
            output = max(-100, min(100, output))
            return output
            
    def _print_initialization_info(self):
        print(f"Motor driver address: 0x{self.motor_i2c_address:02X}")
        print(f"Servo driver address: 0x{self.servo_i2c_address:02X}")
        print(f"I2C bus: {self.i2c_bus or 'Auto detect'}")
        print(f"PWM frequency: 20Hz (motor), 50Hz (servo)")
        print(f"IMU address: 0x{self.imu_i2c_address:02X}")
        print(f"IMU available: {self.imu_available}")
        if self.api_url:
            print(f"AI API URL: {self.api_url}")
        print(f"Backend MQTT: {self.backend_broker}:{self.backend_port}")
        print(f"PID gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")
        
    def start_control_loop(self):
        if self.control_thread and self.control_thread.is_alive():
            return
            
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        if self.debug:
            print("IMU control loop started")
            
    def stop_control_loop(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        
        if self.debug:
            print("IMU control loop stopped")
            
    def _control_loop(self):
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
                            print(f"Target angle reached: {current_angle:.2f} degrees")
                
                time.sleep(0.01)
                
            except Exception as e:
                if self.debug:
                    print(f"Control loop error: {e}")
                time.sleep(0.1)
                
    def turn_left_90(self):
        if self.is_turning:
            if self.debug:
                print("Already turning.")
            return False
            
        if self.imu_available:
            self.target_angle = self._normalize_angle(self.current_angle + 90)
            self.turn_direction = 1
            self.is_turning = True
            self.integral = 0.0
            
            if self.debug:
                print(f"90 degree left turn started (IMU based): Current {self.current_angle:.2f} degrees -> Target {self.target_angle:.2f} degrees")
        else:
            if self.debug:
                print("90 degree left turn started (time based)")
            
            self.differential_drive(-40, 40)
            time.sleep(1.5)
            self.stop_all()
            
            if self.debug:
                print("90 degree left turn completed (time based)")
        
        return True
        
    def turn_right_90(self):
        if self.is_turning:
            if self.debug:
                print("Already turning.")
            return False
            
        if self.imu_available:
            self.target_angle = self._normalize_angle(self.current_angle - 90)
            self.turn_direction = -1
            self.is_turning = True
            self.integral = 0.0
            
            if self.debug:
                print(f"90 degree right turn started (IMU based): Current {self.current_angle:.2f} degrees -> Target {self.target_angle:.2f} degrees")
        else:
            if self.debug:
                print("90 degree right turn started (time based)")
            
            self.differential_drive(40, -40)
            time.sleep(1.5)
            self.stop_all()
            
            if self.debug:
                print("90 degree right turn completed (time based)")
        
        return True
        
    def differential_drive(self, left_speed, right_speed):
        left_speed = max(-300, min(300, left_speed))
        right_speed = max(-300, min(300, right_speed))
        
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
        
        if right_speed > 0:
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 1)
            self.pwm.setLevel(self.BIN2, 0)
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.FORWARD
        elif right_speed < 0:
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 0)
            self.pwm.setLevel(self.BIN2, 1)
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.BACKWARD
        else:
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
        
        if self.debug:
            print(f"Differential drive: L={left_speed}, R={right_speed}")
            
    def stop_all(self):
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        if self.debug:
            print("All motors stopped")
            
    def set_servo_angle(self, channel, angle):
        if not hasattr(self, 'servo_pwm') or self.servo_pwm is None:
            if self.debug:
                print("Servo motor driver not initialized.")
            return False
        
        try:
            pulse_width = int(50 + (angle / 180.0) * 50)
            self.servo_pwm.setPWM(channel, 0, pulse_width)
            
            if self.debug:
                print(f"Servo motor channel {channel}: {angle} degrees (pulse: {pulse_width})")
            
            return True
            
        except Exception as e:
            if self.debug:
                print(f"Servo motor control failed: {e}")
            return False
            
    def get_motor_status(self):
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
        self.kp = kp
        self.ki = ki
        self.kd = kd
        if self.debug:
            print(f"PID gains set: Kp={kp}, Ki={ki}, Kd={kd}")
            
    def set_motor_speeds(self, speeds):
        self.motor_speeds.update(speeds)
        if self.debug:
            print(f"Motor speed settings updated: {self.motor_speeds}")
            
    def get_current_angle(self):
        return self.current_angle
        
    def is_turning_now(self):
        return self.is_turning
        
    def center_robot(self):
        if self.is_turning:
            if self.debug:
                print("Already turning. Skipping center alignment.")
            return False
            
        current_angle = self._read_imu_yaw()
        angle_error = abs(current_angle)
        
        if angle_error < 1.0:
            if self.debug:
                print(f"Robot is already centered. (Angle: {current_angle:.2f} degrees)")
            return True
            
        if self.debug:
            print(f"Center alignment started - Current angle: {current_angle:.2f} degrees")
        
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
    print("=" * 60)
    print("AMR Motor Controller Test")
    print("=" * 60)
    
    controller = None
    
    try:
        controller = AMRMotorController(debug=True)
        
        print("\nSelect test option:")
        print("1. 90 degree left turn test")
        print("2. 90 degree right turn test")
        print("3. Center alignment test")
        print("4. Forward/backward test")
        print("5. Servo motor test")
        
        choice = input("\nSelect (1-5): ").strip()
        
        if choice == "1":
            controller.start_control_loop()
            controller.turn_left_90()
            while controller.is_turning:
                print(f"Turning... Current angle: {controller.get_current_angle():.2f} degrees")
                time.sleep(0.1)
            print("Left turn completed!")
            
        elif choice == "2":
            controller.start_control_loop()
            controller.turn_right_90()
            while controller.is_turning:
                print(f"Turning... Current angle: {controller.get_current_angle():.2f} degrees")
                time.sleep(0.1)
            print("Right turn completed!")
            
        elif choice == "3":
            controller.start_control_loop()
            controller.center_robot()
            while controller.is_turning:
                print(f"Center alignment... Current angle: {controller.get_current_angle():.2f} degrees")
                time.sleep(0.1)
            print("Center alignment completed!")
            
        elif choice == "4":
            print("Forward test (3 seconds)")
            controller.differential_drive(30, 30)
            time.sleep(3)
            controller.stop_all()
            
            time.sleep(1)
            
            print("Backward test (3 seconds)")
            controller.differential_drive(-30, -30)
            time.sleep(3)
            controller.stop_all()
            
        elif choice == "5":
            if hasattr(controller, 'servo_pwm') and controller.servo_pwm:
                print("Servo motor test")
                for channel in [0, 1, 2, 3]:
                    print(f"Channel {channel} test")
                    controller.set_servo_angle(channel, 0)
                    time.sleep(1)
                    controller.set_servo_angle(channel, 90)
                    time.sleep(1)
                    controller.set_servo_angle(channel, 180)
                    time.sleep(1)
                    controller.set_servo_angle(channel, 90)
                    time.sleep(1)
            else:
                print("Servo motor driver not initialized.")
                
        else:
            print("Invalid selection.")
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if controller:
            controller.stop_all()
            controller.stop_control_loop()
        print("Cleanup completed")

if __name__ == "__main__":
    main() 