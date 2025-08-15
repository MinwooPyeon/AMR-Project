import time
import math
import smbus
import json
import base64
import requests
import sys
import os
from datetime import datetime
from PCA9685 import PCA9685

project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
sys.path.insert(0, project_root)

from mqtt.sensor_data_transmitter import SensorDataTransmitter

class AIMotorController:
    
    FORWARD = 'forward'
    BACKWARD = 'backward'
    
    def __init__(self, i2c_address=0x40, i2c_bus=7, debug=True, api_url=None, 
                 backend_broker="192.168.100.141", backend_port=1883):
        self.debug = debug
        self.i2c_address = i2c_address
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
        
        self.motor_speeds = {
            'forward': 50,
            'backward': 50,
            'left': 50,
            'right': 50,
            'stop': 0,
            'custom': 50
        }
        
        try:
            self.pwm = PCA9685(i2c_address, debug=debug)
            self.pwm.setPWMFreq(20)
            
            self.stop_all()
            
            if self.debug:
                print(f"   I2C Address: 0x{i2c_address:02X}")
                print(f"   I2C Bus: {i2c_bus}")
                print(f"   PWM Frequency: 20Hz")
                if api_url:
                    print(f"   AI API URL: {api_url}")
                print(f"   Backend MQTT: {backend_broker}:{backend_port}")
                print(f"   Motor speeds: {self.motor_speeds}")
                
        except Exception as e:
            print(f"AI motor controller initialization failed: {e}")
            raise

    def set_serial_number(self, serial):
        self.serial_number = serial
        if self.debug:
            print(f"Serial number set: {serial}")

    def connect_backend(self):
        try:
            if self.debug:
                print(f"Backend connection attempt: {self.backend_broker}:{self.backend_port}")
            
            self.backend_transmitter = SensorDataTransmitter(
                robot_id=self.serial_number,
                mqtt_broker=self.backend_broker,
                mqtt_port=self.backend_port
            )
            
            if self.backend_transmitter.connect_mqtt():
                self.backend_connected = True
                if self.debug:
                    print("Backend connection successful")
                    print(f"Connected broker: {self.backend_broker}:{self.backend_port}")
                    print(f"Robot ID: {self.serial_number}")
                return True
            else:
                if self.debug:
                    print("Backend connection failed")
                return False
                
        except Exception as e:
            if self.debug:
                print(f"Backend connection error: {e}")
            return False

    def disconnect_backend(self):
        if self.backend_transmitter:
            self.backend_transmitter.disconnect_mqtt()
            self.backend_connected = False
            if self.debug:
                print("Backend connection disconnected")

    def send_to_backend(self, ai_data=None, motor_status=None):
        if not self.backend_connected or not self.backend_transmitter:
            if self.debug:
                print("Backend is not connected.")
            return False
        
        try:
            if self.debug:
                print("Backend data transmission started")
            
            state = "RUNNING"
            x = 0.0
            y = 0.0
            speed = 40.0
            angle = 0.0
            
            if ai_data:
                x = float(ai_data.get('x', 0))
                y = float(ai_data.get('y', 0))
                
                motor_case = self.determine_case_from_coordinates(x, y)
                
                if motor_case in ['forward', '전진']:
                    speed = self.get_motor_speed('forward')
                elif motor_case in ['backward', '후진']:
                    speed = self.get_motor_speed('backward')
                elif motor_case in ['left', '좌회전']:
                    speed = self.get_motor_speed('left')
                elif motor_case in ['right', '우회전']:
                    speed = self.get_motor_speed('right')
                elif motor_case in ['stop', '정지']:
                    speed = self.get_motor_speed('stop')
                elif motor_case in ['custom', '커스텀']:
                    speed = self.get_motor_speed('custom')
            
            if motor_status:
                motor_a_speed = motor_status.get('motor_a', {}).get('speed', 0)
                motor_b_speed = motor_status.get('motor_b', {}).get('speed', 0)
                if motor_a_speed > 0 or motor_b_speed > 0:
                    speed = max(motor_a_speed, motor_b_speed)
            
            if self.debug:
                print(f"Data to send: state={state}, x={x}, y={y}, speed={speed}, angle={angle}")
            
            self.backend_transmitter.update_embedded_data(
                serial=self.serial_number,
                state=state,
                x=x,
                y=y,
                speed=speed,
                angle=angle
            )
            
            if self.backend_transmitter.send_embedded_data():
                if self.debug:
                    print(f"Backend transmission successful: {state}, speed: {speed}%")
                    print(f"Topic: status")
                    print(f"Data: {self.backend_transmitter.get_embedded_data()}")
                return True
            else:
                if self.debug:
                    print("Backend transmission failed")
                    print(f"Connection status: {self.backend_transmitter.connected}")
                    print(f"Broker: {self.backend_broker}:{self.backend_port}")
                return False
                
        except Exception as e:
            if self.debug:
                print(f"Backend transmission error: {e}")
            return False

    def get_ai_data(self):
        if not self.api_url:
            if self.debug:
                print("API URL is not set.")
            return None
            
        try:
            response = requests.get(self.api_url, timeout=5)
            if response.status_code == 200:
                data = response.json()
                self.last_ai_data = data
                if self.debug:
                    print(f"AI data received: {data}")
                return data
            else:
                if self.debug:
                    print(f"API request failed: {response.status_code}")
                return None
        except Exception as e:
            if self.debug:
                print(f"AI data retrieval error: {e}")
            return None

    def process_ai_command(self, ai_data):
        if not ai_data:
            return
            
        try:
            serial = ai_data.get('serial', '')
            if serial and serial != self.serial_number:
                if self.debug:
                    print(f"Serial number mismatch: {serial} != {self.serial_number}")
                return
                
            x = float(ai_data.get('x', 0))
            y = float(ai_data.get('y', 0))
            case = ai_data.get('case', '')
            timestamp = ai_data.get('timeStamp', '')
            
            motor_case = self.determine_case_from_coordinates(x, y)
            
            if self.debug:
                print(f"AI command processing:")
                print(f"  Coordinates: ({x}, {y})")
                print(f"  Case: {case}")
                print(f"  Motor control: {motor_case}")
                print(f"  Timestamp: {timestamp}")
            
            self.execute_ai_case(motor_case, x, y)
            
            motor_status = self.get_motor_status()
            self.send_to_backend(ai_data, motor_status)
            
        except Exception as e:
            if self.debug:
                print(f"AI command processing error: {e}")

    def determine_case_from_coordinates(self, x, y):
        abs_x = abs(x)
        abs_y = abs(y)
        
        threshold = 0.3
        
        if abs_x <= threshold and abs_y <= threshold:
            return 'stop'
        
        if abs_y > abs_x and abs_y > threshold:
            if y > 0:
                return 'forward'
            else:
                return 'backward'
        
        if abs_x >= abs_y and abs_x > threshold:
            if x > 0:
                return 'right'
            else:
                return 'left'
        
        return 'custom'

    def execute_ai_case(self, case, x, y):
        case = case.lower()
        
        speed = self.get_motor_speed(case)
        
        if case == 'forward':
            self.differential_drive(speed, speed)
            if self.debug:
                print(f"Forward command executed: speed {speed}%")
                
        elif case == 'backward':
            self.differential_drive(-speed, -speed)
            if self.debug:
                print(f"Backward command executed: speed {speed}%")
                
        elif case == 'left':
            self.differential_drive(-speed, speed)
            if self.debug:
                print(f"Left turn command executed: speed {speed}%")
                
        elif case == 'right':
            self.differential_drive(speed, -speed)
            if self.debug:
                print(f"Right turn command executed: speed {speed}%")
                
        elif case == 'stop':
            self.stop_all()
            if self.debug:
                print("Stop command executed")
                
        elif case == 'custom':
            left_speed = int(y * 100)  
            right_speed = int(x * 100) 
            
            left_speed = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))
            
            self.differential_drive(left_speed, right_speed)
            if self.debug:
                print(f"Custom control: L={left_speed}, R={right_speed}")
                
        else:
            if self.debug:
                print(f"Unknown case: {case}")
            self.stop_all()

    def send_status_to_ai(self, status_data=None):
        if not self.api_url:
            return
            
        try:
            status = {
                "serial": self.serial_number,
                "motor_a_speed": self.motor_a_speed,
                "motor_b_speed": self.motor_b_speed,
                "motor_a_direction": self.motor_a_direction,
                "motor_b_direction": self.motor_b_direction,
                "timestamp": datetime.now().isoformat()
            }
            
            if status_data:
                status.update(status_data)
                
            response = requests.post(self.api_url, json=status, timeout=5)
            
            if self.debug:
                if response.status_code == 200:
                    print("Status transmission successful")
                else:
                    print(f"Status transmission failed: {response.status_code}")
                    
        except Exception as e:
            if self.debug:
                print(f"Status transmission error: {e}")

    def run_ai_control_loop(self, interval=1.0):
        if not self.api_url:
            print("API URL is not set.")
            return
            
        print(f"AI control loop started (interval: {interval} seconds)")
        
        try:
            while True:
                ai_data = self.get_ai_data()
                
                if ai_data:
                    self.process_ai_command(ai_data)
                    self.send_status_to_ai()
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\nAI control loop interrupted")
            self.stop_all()
        except Exception as e:
            print(f"AI control loop error: {e}")
            self.stop_all()

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
        
        print("\nMotor A and B simultaneous forward test")
        self.differential_drive(40, 40)
        time.sleep(2)
        
        print("\nMotor A and B simultaneous backward test")
        self.differential_drive(-40, -40)
        time.sleep(2)
        
        print("\nMotor A and B simultaneous stop")
        print("\nMotor test started")
        
        print("\n1. Forward start (10 seconds)")
        self.differential_drive(30, 30)
        time.sleep(10)
        
        print("\n2. Stop start (1 second)")
        self.stop_all()
        time.sleep(1)
        
        print("\n3. Backward start (10 seconds)")
        self.differential_drive(-30, -30)
        time.sleep(10)
        
        print("\n4. Final stop")
        self.stop_all()
        
        print("Motor test completed")
    
    def differential_drive(self, left_speed, right_speed):
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
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
            self.pwm.setLevel(self.BIN1, 0)
            self.pwm.setLevel(self.BIN2, 1)
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.FORWARD
        elif right_speed < 0:
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 1)
            self.pwm.setLevel(self.BIN2, 0)
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.BACKWARD
        else:
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
        
        if self.debug:
            print(f"Differential drive: L={left_speed}, R={right_speed}")

    def set_motor_speed_config(self, speeds):
        self.motor_speeds.update(speeds)
        if self.debug:
            print(f"Motor speed configuration updated: {self.motor_speeds}")

    def get_motor_speed(self, case):
        case = case.lower()
        return self.motor_speeds.get(case, self.motor_speeds['custom'])

def main():
    print("=" * 60)
    print("AI Motor Controller Test")
    print("=" * 60)
    
    ai_api_url = "http://localhost:5001/pose"
    
    try:
        motor = AIMotorController(
            debug=True, 
            api_url=ai_api_url,
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        motor.set_serial_number("AMR001")
        
        if motor.connect_backend():
            print("Backend connection successful")
        else:
            print("Backend connection failed")
        
        motor.run_ai_control_loop(interval=1.0)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        motor.stop_all()
        motor.disconnect_backend()
    except Exception as e:
        print(f"\nError occurred: {e}")
        motor.stop_all()
        motor.disconnect_backend()
    finally:
        print("Cleanup completed")

if __name__ == "__main__":
    main()
