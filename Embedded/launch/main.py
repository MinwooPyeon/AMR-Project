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
        
        self.pwm.setPWMFreq(50)
        self.pwm.setPWM(self.PWMA, 0, 0)
        self.pwm.setPWM(self.AIN1, 0, 0)
        self.pwm.setPWM(self.AIN2, 0, 0)
        self.pwm.setPWM(self.PWMB, 0, 0)
        self.pwm.setPWM(self.BIN1, 0, 0)
        self.pwm.setPWM(self.BIN2, 0, 0)
        
        print("Motor driver initialization completed")
    
    def MotorRun(self, motor, index, speed):
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
            
        if motor == 0:
            if index == 1:
                self.pwm.setPWM(self.AIN1, 0, 4095)
                self.pwm.setPWM(self.AIN2, 0, 0)
                self.pwm.setPWM(self.PWMA, 0, int(speed * 40.95))
            else:
                self.pwm.setPWM(self.AIN1, 0, 0)
                self.pwm.setPWM(self.AIN2, 0, 4095)
                self.pwm.setPWM(self.PWMA, 0, int(speed * 40.95))
        else:
            if index == 1:
                self.pwm.setPWM(self.BIN1, 0, 4095)
                self.pwm.setPWM(self.BIN2, 0, 0)
                self.pwm.setPWM(self.PWMB, 0, int(speed * 40.95))
            else:
                self.pwm.setPWM(self.BIN1, 0, 0)
                self.pwm.setPWM(self.BIN2, 0, 4095)
                self.pwm.setPWM(self.PWMB, 0, int(speed * 40.95))
    
    def MotorStop(self, motor):
        if motor == 0:
            self.pwm.setPWM(self.PWMA, 0, 0)
        else:
            self.pwm.setPWM(self.PWMB, 0, 0)

def main():
    print("=== Motor Driver Test ===")
    
    pwm = PCA9685(0x40, debug=False)
    
    motor_driver = MotorDriver(pwm)
    
    try:
        print("Starting motor test...")
        
        print("Forward test (3 seconds)")
        motor_driver.MotorRun(0, 1, 50)
        motor_driver.MotorRun(1, 1, 50)
        time.sleep(3)
        
        print("Stop")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        print("Backward test (3 seconds)")
        motor_driver.MotorRun(0, 0, 50)
        motor_driver.MotorRun(1, 0, 50)
        time.sleep(3)
        
        print("Stop")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        print("Left turn test (3 seconds)")
        motor_driver.MotorRun(0, 0, 50)
        motor_driver.MotorRun(1, 1, 50)
        time.sleep(3)
        
        print("Stop")
        motor_driver.MotorStop(0)
        motor_driver.MotorStop(1)
        time.sleep(1)
        
        print("Right turn test (3 seconds)")
        motor_driver.MotorRun(0, 1, 50)
        motor_driver.MotorRun(1, 0, 50)
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