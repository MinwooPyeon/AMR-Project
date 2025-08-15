import time
import threading
from typing import Dict, Optional
from utils.logger import motor_logger

class ServoMotorController:
    def __init__(self, i2c_address: int = 0x40, frequency: int = 50):
        self.i2c_address = i2c_address
        self.frequency = frequency
        
        self.servo_channels = {
            "servo1": 0,
            "servo2": 1,  
            "servo3": 2,  
            "servo4": 3   
        }
        
        self.angle_min = 0
        self.angle_max = 180
        self.default_angle = 90
        
        self.pulse_min = 500   
        self.pulse_max = 2500  
        
        self.current_angles = {
            "servo1": self.default_angle,
            "servo2": self.default_angle,
            "servo3": self.default_angle,
            "servo4": self.default_angle
        }
        
        self.angle_lock = threading.Lock()
        
        self.pca9685 = None
        self._initialize_pca9685()
        
        motor_logger.success("Servo motor controller initialization completed")
    
    def _initialize_pca9685(self):
        try:
            import smbus2 as smbus
            
            self.bus = smbus.SMBus(1)
            self._pca9685_init()
            
            motor_logger.success(f"PCA9685 driver initialization successful - Address: 0x{self.i2c_address:02X}")
            
        except ImportError:
            motor_logger.warn("smbus2 module not found, running in simulation mode")
            self.bus = None
        except Exception as e:
            motor_logger.error(f"PCA9685 initialization failed: {e}")
            self.bus = None
    
    def _pca9685_init(self):
        if not self.bus:
            return
        
        try:
            self.bus.write_byte_data(self.i2c_address, 0x00, 0x20)
            time.sleep(0.01)
            
            self.bus.write_byte_data(self.i2c_address, 0x01, 0x04)
            time.sleep(0.01)
            
            prescale = int(25000000 / (4096 * self.frequency) - 1)
            self.bus.write_byte_data(self.i2c_address, 0xFE, prescale)
            time.sleep(0.01)
            
            self.bus.write_byte_data(self.i2c_address, 0x00, 0xA0)
            time.sleep(0.01)
            
            motor_logger.info(f"PCA9685 initialization completed - Frequency: {self.frequency}Hz")
            
        except Exception as e:
            motor_logger.error(f"PCA9685 initialization error: {e}")
    
    def _set_pwm(self, channel: int, on: int, off: int):
        if not self.bus:
            motor_logger.debug(f"Simulation mode - Channel {channel}: ON={on}, OFF={off}")
            return
        
        try:
            self.bus.write_byte_data(self.i2c_address, 0x06 + 4 * channel, on & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x07 + 4 * channel, (on >> 8) & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x08 + 4 * channel, off & 0xFF)
            self.bus.write_byte_data(self.i2c_address, 0x09 + 4 * channel, (off >> 8) & 0xFF)
            
        except Exception as e:
            motor_logger.error(f"PWM setting error - Channel {channel}: {e}")
    
    def _angle_to_pulse(self, angle: float) -> int:
        pulse_width = int(self.pulse_min + (angle / 180.0) * (self.pulse_max - self.pulse_min))
        counter_value = int(pulse_width * 4096 / 20000)
        return counter_value
    
    def set_servo_angle(self, servo_name: str, angle: float) -> bool:
        if servo_name not in self.servo_channels:
            motor_logger.error(f"Unknown servo: {servo_name}")
            return False
        
        if angle < self.angle_min or angle > self.angle_max:
            motor_logger.error(f"Angle range error: {angle} degrees (range: {self.angle_min}-{self.angle_max} degrees)")
            return False
        
        try:
            channel = self.servo_channels[servo_name]
            pulse_counter = self._angle_to_pulse(angle)
            
            self._set_pwm(channel, 0, pulse_counter)
            
            with self.angle_lock:
                self.current_angles[servo_name] = angle
            
            motor_logger.info(f"{servo_name} angle set: {angle} degrees (channel: {channel})")
            return True
            
        except Exception as e:
            motor_logger.error(f"Servo angle setting error: {e}")
            return False
    
    def set_all_servos(self, angle: float) -> bool:
        success = True
        for servo_name in self.servo_channels.keys():
            if not self.set_servo_angle(servo_name, angle):
                success = False
        
        if success:
            motor_logger.info(f"All servo angles set: {angle} degrees")
        
        return success
    
    def set_servo_angles(self, angles: Dict[str, float]) -> bool:
        success = True
        for servo_name, angle in angles.items():
            if not self.set_servo_angle(servo_name, angle):
                success = False
        
        if success:
            motor_logger.info(f"All servo angles set: {angles}")
        
        return success
    
    def get_servo_angle(self, servo_name: str) -> Optional[float]:
        if servo_name not in self.current_angles:
            motor_logger.error(f"Unknown servo: {servo_name}")
            return None
        
        with self.angle_lock:
            return self.current_angles[servo_name]
    
    def get_all_angles(self) -> Dict[str, float]:
        with self.angle_lock:
            return self.current_angles.copy()
    
    def reset_all_servos(self) -> bool:
        return self.set_all_servos(self.default_angle)
    
    def sweep_servo(self, servo_name: str, start_angle: float = 0, end_angle: float = 180, 
                   step: float = 5, delay: float = 0.1) -> bool:
        if servo_name not in self.servo_channels:
            motor_logger.error(f"Unknown servo: {servo_name}")
            return False
        
        try:
            motor_logger.info(f"{servo_name} sweep started: {start_angle} degrees → {end_angle} degrees")
            
            for angle in range(int(start_angle), int(end_angle) + 1, int(step)):
                self.set_servo_angle(servo_name, angle)
                time.sleep(delay)
            
            for angle in range(int(end_angle), int(start_angle) - 1, -int(step)):
                self.set_servo_angle(servo_name, angle)
                time.sleep(delay)
            
            motor_logger.info(f"{servo_name} sweep completed")
            return True
            
        except Exception as e:
            motor_logger.error(f"Servo sweep error: {e}")
            return False
    
    def get_status(self) -> Dict:
        return {
            "initialized": self.bus is not None,
            "i2c_address": f"0x{self.i2c_address:02X}",
            "frequency": self.frequency,
            "current_angles": self.get_all_angles(),
            "servo_channels": self.servo_channels,
            "angle_range": {"min": self.angle_min, "max": self.angle_max},
            "pulse_range": {"min": self.pulse_min, "max": self.pulse_max}
        }
    
    def cleanup(self):
        if self.bus:
            try:
                self.bus.close()
                motor_logger.info("I2C bus connection released")
            except Exception as e:
                motor_logger.error(f"I2C bus release error: {e}")

def test_servo_motor_controller():
    print("=== Servo motor controller test ===")
    print("Control 4 servo motors using PCA9685 driver")
    print("=" * 50)
    
    controller = ServoMotorController()
    
    status = controller.get_status()
    print(f"\nServo motor controller status:")
    print(f"  - Initialized: {'Yes' if status['initialized'] else 'No'}")
    print(f"  - I2C address: {status['i2c_address']}")
    print(f"  - PWM frequency: {status['frequency']}Hz")
    print(f"  - Angle range: {status['angle_range']['min']}° - {status['angle_range']['max']}°")
    
    print(f"\nServo motor channels:")
    for servo_name, channel in status['servo_channels'].items():
        print(f"  - {servo_name}: channel {channel}")
    
    try:
        print(f"\nTest started...")
        
        print("\n1. Set all servo motors to 90 degrees")
        controller.set_all_servos(90)
        time.sleep(2)
        
        print("\n2. Test individual servo motors")
        test_angles = [0, 45, 90, 135, 180]
        
        for servo_name in controller.servo_channels.keys():
            print(f"\n   {servo_name} test:")
            for angle in test_angles:
                controller.set_servo_angle(servo_name, angle)
                print(f"     → {angle} degrees")
                time.sleep(0.5)
        
        print("\n3. Reset all servo motors to 90 degrees")
        controller.reset_all_servos()
        time.sleep(2)
        
        print("\n4. servo1 sweep test")
        controller.sweep_servo("servo1", 0, 180, 10, 0.1)
        
        print("\n5. Check final state")
        final_angles = controller.get_all_angles()
        for servo_name, angle in final_angles.items():
            print(f"   {servo_name}: {angle} degrees")
        
        print("\nServo motor controller test completed")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    test_servo_motor_controller() 