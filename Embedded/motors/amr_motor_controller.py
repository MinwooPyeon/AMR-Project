import os
import time
import math
import json
import threading
import smbus2 as smbus
from datetime import datetime
from motors.PCA9685 import PCA9685

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
                 debug=True):
        self.debug = debug
        self.motor_i2c_address = motor_i2c_address
        self.servo_i2c_address = servo_i2c_address
        self.imu_i2c_address = imu_i2c_address
        self.i2c_bus = i2c_bus

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

        self.imu_bus_num = None
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
                    print(f"Servo initialized - Address: 0x{self.servo_i2c_address:02X}")
            except Exception as e:
                if self.debug:
                    print(f"Servo initialization failed: {e}")
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
            self.imu_bus_num = self._find_imu_bus()
            if self.imu_bus_num is None:
                raise Exception("No available I2C bus found")

            detected_address = self._find_imu_address()
            if detected_address:
                self.imu_i2c_address = detected_address

            with smbus.SMBus(self.imu_bus_num) as bus:
                bus.write_byte_data(self.imu_i2c_address, 0x6B, 0)

            print("IMU calibration in progress...")
            angles = []
            for _ in range(100):
                angle = self._read_imu_yaw_raw()
                angles.append(angle)
                time.sleep(0.01)

            self.angle_offset = sum(angles) / len(angles)
            self.current_angle = 0.0
            self.target_angle = 0.0
            self.imu_available = True

            if self.debug:
                print(f"IMU initialized - Bus: {self.imu_bus_num}, "
                      f"Address: 0x{self.imu_i2c_address:02X}, "
                      f"Offset: {self.angle_offset:.2f} deg")

        except Exception as e:
            print(f"IMU initialization failed: {e}")
            print("Falling back to motor-only control.")
            self.imu_available = False

    def _find_imu_bus(self):
        for bus_num in range(10):
            if not os.path.exists(f"/dev/i2c-{bus_num}"):
                continue
            try:
                with smbus.SMBus(bus_num):
                    pass
                if self.debug:
                    print(f"I2C bus found: {bus_num}")
                return bus_num
            except Exception as e:
                if self.debug:
                    print(f"Bus {bus_num} unavailable: {e}")
        return None

    def _find_imu_address(self):
        for addr in [0x68, 0x69]:
            try:
                with smbus.SMBus(self.imu_bus_num) as bus:
                    bus.write_byte_data(addr, 0x6B, 0)
                    time.sleep(0.1)
                    who_am_i = bus.read_byte_data(addr, 0x75)
                if who_am_i == 0x68:
                    if self.debug:
                        print(f"IMU found at 0x{addr:02X}")
                    return addr
            except Exception as e:
                if self.debug:
                    print(f"Address 0x{addr:02X} failed: {e}")
        return None

    def _read_imu_yaw_raw(self) -> float:
        if not self.imu_bus_num:
            return 0.0

        try:
            with smbus.SMBus(self.imu_bus_num) as bus:
                gyro_z = bus.read_word_data(self.imu_i2c_address, 0x47)

            gyro_z = self._convert_to_signed(gyro_z)
            yaw_rate = gyro_z * (250.0 / 32768.0)
            return yaw_rate * 0.01

        except Exception as e:
            if self.debug:
                print(f"IMU read error: {e}")
            return 0.0

    def _read_imu_yaw(self) -> float:
        if not self.imu_available or self.imu_bus_num is None:
            return 0.0

        delta = self._read_imu_yaw_raw()

        with self.thread_lock:
            self.current_angle += delta
            self.current_angle = self._normalize_angle(self.current_angle)
            result = self._normalize_angle(self.current_angle - self.angle_offset)

        return result

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
            return max(-100, min(100, output))

    def _print_initialization_info(self):
        print(f"Motor I2C: 0x{self.motor_i2c_address:02X}")
        print(f"Servo I2C: 0x{self.servo_i2c_address:02X}")
        print(f"IMU I2C: 0x{self.imu_i2c_address:02X}")
        print(f"IMU available: {self.imu_available}")

    def start_control_loop(self):
        if self.control_thread and self.control_thread.is_alive():
            return

        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
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
                            print(f"Target angle reached: {current_angle:.2f} deg")

                time.sleep(0.01)

            except Exception as e:
                if self.debug:
                    print(f"Control loop error: {e}")
                time.sleep(0.1)

    def turn_left_90(self) -> bool:
        if self.is_turning:
            return False

        if self.imu_available:
            with self.thread_lock:
                self.target_angle = self._normalize_angle(self.current_angle + 90)
                self.turn_direction = 1
                self.is_turning = True
                self.integral = 0.0
        else:
            self.differential_drive(-40, 40)
            time.sleep(1.5)
            self.stop_all()

        return True

    def turn_right_90(self) -> bool:
        if self.is_turning:
            return False

        if self.imu_available:
            with self.thread_lock:
                self.target_angle = self._normalize_angle(self.current_angle - 90)
                self.turn_direction = -1
                self.is_turning = True
                self.integral = 0.0
        else:
            self.differential_drive(40, -40)
            time.sleep(1.5)
            self.stop_all()

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

    def stop_all(self):
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)
        self.motor_a_speed = 0
        self.motor_b_speed = 0

    def set_servo_angle(self, channel, angle) -> bool:
        if not getattr(self, 'servo_pwm', None):
            return False

        try:
            pulse_width = int(50 + (angle / 180.0) * 50)
            self.servo_pwm.setPWM(channel, 0, pulse_width)
            return True
        except Exception as e:
            if self.debug:
                print(f"Servo control failed: {e}")
            return False

    def get_motor_status(self):
        with self.thread_lock:
            return {
                'motor_a': {'speed': self.motor_a_speed, 'direction': self.motor_a_direction},
                'motor_b': {'speed': self.motor_b_speed, 'direction': self.motor_b_direction},
                'current_angle': self.current_angle,
                'is_turning': self.is_turning
            }

    def set_pid_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_current_angle(self) -> float:
        with self.thread_lock:
            return self.current_angle

    def is_turning_now(self) -> bool:
        return self.is_turning

    def center_robot(self) -> bool:
        if self.is_turning:
            return False

        current_angle = self._read_imu_yaw()
        if abs(current_angle) < 1.0:
            return True

        self.target_angle = 0.0
        error = self._calculate_angle_error(self.target_angle, current_angle)
        self.turn_direction = 1 if error > 0 else -1
        with self.thread_lock:
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
        print("1. 90 degree left turn")
        print("2. 90 degree right turn")
        print("3. Center alignment")
        print("4. Forward/backward")
        print("5. Servo test")

        choice = input("\nSelect (1-5): ").strip()

        if choice == "1":
            controller.start_control_loop()
            controller.turn_left_90()
            while controller.is_turning:
                print(f"\rTurning... {controller.get_current_angle():.2f} deg", end="")
                time.sleep(0.1)
            print("\nLeft turn done!")

        elif choice == "2":
            controller.start_control_loop()
            controller.turn_right_90()
            while controller.is_turning:
                print(f"\rTurning... {controller.get_current_angle():.2f} deg", end="")
                time.sleep(0.1)
            print("\nRight turn done!")

        elif choice == "3":
            controller.start_control_loop()
            controller.center_robot()
            while controller.is_turning:
                print(f"\rCentering... {controller.get_current_angle():.2f} deg", end="")
                time.sleep(0.1)
            print("\nCentered!")

        elif choice == "4":
            print("Forward 3 seconds")
            controller.differential_drive(30, 30)
            time.sleep(3)
            controller.stop_all()
            time.sleep(1)
            print("Backward 3 seconds")
            controller.differential_drive(-30, -30)
            time.sleep(3)
            controller.stop_all()

        elif choice == "5":
            if getattr(controller, 'servo_pwm', None):
                for channel in range(4):
                    for angle in [0, 90, 180, 90]:
                        controller.set_servo_angle(channel, angle)
                        time.sleep(1)
            else:
                print("Servo not available.")

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
