import time
from PCA9685 import PCA9685

class MotorDriver:
    
    def __init__(self, pwm):
        self.pwm = pwm
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4
        
        from config.system_config import get_config
        config = get_config()
        
        self.pwm.setPWMFreq(config.MOTOR_PWM_FREQUENCY)
        self.pwm.setPWM(self.PWMA, 0, 0)
        self.pwm.setPWM(self.AIN1, 0, 0)
        self.pwm.setPWM(self.AIN2, 0, 0)
        self.pwm.setPWM(self.PWMB, 0, 0)
        self.pwm.setPWM(self.BIN1, 0, 0)
        self.pwm.setPWM(self.BIN2, 0, 0)
        
        print("Motor driver initialization completed")
    
    def MotorRun(self, motor, index, speed):
        if speed > config.MOTOR_MAX_SPEED:
            speed = config.MOTOR_MAX_SPEED
        elif speed < 0:
            speed = 0
            
        if motor == 0:
            if index == 1:
                self.pwm.setPWM(self.AIN1, 0, config.MOTOR_PWM_RESOLUTION)
                self.pwm.setPWM(self.AIN2, 0, 0)
                self.pwm.setPWM(self.PWMA, 0, int(speed * config.MOTOR_SPEED_MULTIPLIER))
            else:
                self.pwm.setPWM(self.AIN1, 0, 0)
                self.pwm.setPWM(self.AIN2, 0, config.MOTOR_PWM_RESOLUTION)
                self.pwm.setPWM(self.PWMA, 0, int(speed * config.MOTOR_SPEED_MULTIPLIER))
        else:
            if index == 1:
                self.pwm.setPWM(self.BIN1, 0, config.MOTOR_PWM_RESOLUTION)
                self.pwm.setPWM(self.BIN2, 0, 0)
                self.pwm.setPWM(self.PWMB, 0, int(speed * config.MOTOR_SPEED_MULTIPLIER))
            else:
                self.pwm.setPWM(self.BIN1, 0, 0)
                self.pwm.setPWM(self.BIN2, 0, config.MOTOR_PWM_RESOLUTION)
                self.pwm.setPWM(self.PWMB, 0, int(speed * config.MOTOR_SPEED_MULTIPLIER))
    
    def MotorStop(self, motor):
        if motor == 0:
            self.pwm.setPWM(self.PWMA, 0, 0)
        else:
            self.pwm.setPWM(self.PWMB, 0, 0)

def main():
    print("=== Motor Driver Test ===")
    
    pwm = PCA9685(config.MOTOR_I2C_ADDRESS, debug=False)
    
    motor_driver = MotorDriver(pwm)
    
    try:
        print("Starting motor test...")
        
        print("Forward test (3 seconds)")
        motor_driver.MotorRun(0, 1, config.MOTOR_DEFAULT_SPEED)
        motor_driver.MotorRun(1, 1, config.MOTOR_DEFAULT_SPEED)
        time.sleep(3)
        
        print("Stop")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        print("Backward test (3 seconds)")
        motor_driver.MotorRun(0, 0, config.MOTOR_DEFAULT_SPEED)
        motor_driver.MotorRun(1, 0, config.MOTOR_DEFAULT_SPEED)
        time.sleep(3)
        
        print("Stop")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        print("Left turn test (3 seconds)")
        motor_driver.MotorRun(0, 0, config.MOTOR_DEFAULT_SPEED)
        motor_driver.MotorRun(1, 1, config.MOTOR_DEFAULT_SPEED)
        time.sleep(3)
        
        print("Stop")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        print("Right turn test (3 seconds)")
        motor_driver.MotorRun(0, 1, config.MOTOR_DEFAULT_SPEED)
        motor_driver.MotorRun(1, 0, config.MOTOR_DEFAULT_SPEED)
        time.sleep(3)
        
        print("Stop")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        
        print("Motor test completed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
    except Exception as e:
        print(f"Error occurred: {e}")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)

if __name__ == "__main__":
    main() 