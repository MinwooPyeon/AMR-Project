
import time
import math
import smbus
from PCA9685 import PCA9685

class MotorDriverHAT:
    
    FORWARD = 'forward'
    BACKWARD = 'backward'
    
    def __init__(self, i2c_address=0x40, i2c_bus=7, debug=True):
        self.debug = debug
        self.i2c_address = i2c_address
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
        
        try:
            self.pwm = PCA9685(i2c_address, debug=debug)
            self.pwm.setPWMFreq(50)
            
            self.stop_all()
            
            if self.debug:
                print(f"Motor Driver HAT initialization completed")
                print(f"   I2C Address: 0x{i2c_address:02X}")
                print(f"   I2C Bus: {i2c_bus}")
                print(f"   PWM Frequency: 50Hz")
                
        except Exception as e:
            print(f"Motor Driver HAT initialization failed: {e}")
            raise
    
    def set_motor_speed(self, motor, direction, speed):
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
            
        if motor == 0:
            self.pwm.setDutycycle(self.PWMA, speed)
            if direction == self.FORWARD:
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
            
            self.motor_a_speed = speed
            self.motor_a_direction = direction
            
            if self.debug:
                print(f"Motor A: {direction}, speed: {speed}%")
                
        elif motor == 1:
            self.pwm.setDutycycle(self.PWMB, speed)
            if direction == self.FORWARD:
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
            else:
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)
            
            self.motor_b_speed = speed
            self.motor_b_direction = direction
            
            if self.debug:
                print(f"Motor B: {direction}, speed: {speed}%")
    
    def stop_motor(self, motor):
        if motor == 0:
            self.pwm.setDutycycle(self.PWMA, 0)
            self.motor_a_speed = 0
            if self.debug:
                print("Motor A stopped")
        elif motor == 1:
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
            if self.debug:
                print("Motor B stopped")
    
    def stop_all(self):
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        if self.debug:
            print("All motors stopped")
    
    def get_motor_status(self):
        return {
            'motor_a': {
                'speed': self.motor_a_speed,
                'direction': self.motor_a_direction
            },
            'motor_b': {
                'speed': self.motor_b_speed,
                'direction': self.motor_b_direction
            }
        }
    
    def test_motors(self):
        print("\nMotor test started")
        
        print("\nMotor A test")
        self.set_motor_speed(0, self.FORWARD, 40)
        time.sleep(2)
        self.set_motor_speed(0, self.BACKWARD, 40)
        time.sleep(2)
        self.stop_motor(0)
        
        print("\nMotor B test")
        self.set_motor_speed(1, self.FORWARD, 40)
        time.sleep(2)
        self.set_motor_speed(1, self.BACKWARD, 40)
        time.sleep(2)
        self.stop_motor(1)
        
        print("Motor test completed")
    
    def differential_drive(self, left_speed, right_speed):
        if left_speed > 0:
            self.set_motor_speed(0, self.FORWARD, abs(left_speed))
        elif left_speed < 0:
            self.set_motor_speed(0, self.BACKWARD, abs(left_speed))
        else:
            self.stop_motor(0)
        
        if right_speed > 0:
            self.set_motor_speed(1, self.FORWARD, abs(right_speed))
        elif right_speed < 0:
            self.set_motor_speed(1, self.BACKWARD, abs(right_speed))
        else:
            self.stop_motor(1)
        
        if self.debug:
            print(f"Differential drive: L={left_speed}, R={right_speed}")

def main():
    print("=" * 60)
    print("Jetson Nano Motor Driver HAT test")
    print("=" * 60)
    
    try:
        motor = MotorDriverHAT(debug=True)
        
        motor.test_motors()
        
        print("\nDifferential drive test")
        print("Forward")
        motor.differential_drive(50, 50)
        time.sleep(3)
        
        print("Backward")
        motor.differential_drive(-50, -50)
        time.sleep(3)
        
        print("Left turn")
        motor.differential_drive(-30, 30)
        time.sleep(3)
        
        print("Right turn")
        motor.differential_drive(30, -30)
        time.sleep(3)
        
        motor.stop_all()
        print("\nTest completed!")
        
    except KeyboardInterrupt:
        print("\nStopped by user")
        motor.stop_all()
    except Exception as e:
        print(f"\nError: {e}")
        motor.stop_all()
    finally:
        print("Cleanup completed")

if __name__ == "__main__":
    main() 