import time
import sys
import os
import json
import threading
import termios
import tty
import select
from typing import Dict, Optional, Callable, Any

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from PCA9685 import PCA9685

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    print("paho-mqtt is not installed. MQTT functionality is disabled.")
    print("Install: pip install paho-mqtt")
    MQTT_AVAILABLE = False

class IntegratedMotorControl:
    def __init__(self, i2c_address=0x40, i2c_bus=1, debug=True):
        print("Integrated motor control system started")
        
        self.robot_id = "AMR001"
        self.debug = debug
        
        self.PWMA, self.AIN1, self.AIN2 = 0, 1, 2
        self.PWMB, self.BIN1, self.BIN2 = 5, 3, 4
        self.speed = 50
        self.action = 'stop'
        self.running = False
        self.old_terminal = None
        
        self.control_mode = 'manual'
        
        self.position = {"x": 0.0, "y": 0.0}
        self.robot_state = "IDLE"
        self.motor_angle = 0.0
        
        self.pwm_thread = None
        self.mqtt_thread = None
        self.ai_thread = None
        
        self.data_lock = threading.Lock()
        
        self.init_motor_driver(i2c_address, i2c_bus)
        
        if MQTT_AVAILABLE:
            self.init_mqtt_clients()
        else:
            print("MQTT functionality disabled - keyboard control only")
            print("AI Moving is controlled by keyboard")

    def init_motor_driver(self, addr, bus):
        for attempt in range(3):
            try:
                print(f"PCA9685 connection attempt {attempt+1}/3...")
                self.pwm = PCA9685(addr, debug=False, i2c_bus=bus)
                self.pwm.setPWMFreq(50)
                self.stop()
                print("PCA9685 connection successful!")
                return
            except Exception as e:
                print(f"Connection failed: {e}")
                if attempt < 2:
                    time.sleep(1)
        
        print("\nSolution:")
        print("sudo pkill -f jtop")
        print("sudo chmod 666 /dev/i2c-*")
        raise Exception("PCA9685 connection failed")

    def init_mqtt_clients(self):
        if not MQTT_AVAILABLE:
            return
        
        try:
            self.backend_client = mqtt.Client(client_id=f"embedded_{self.robot_id}")
            self.backend_client.on_connect = self.on_backend_connect
            self.backend_client.on_message = self.on_backend_message
            
            try:
                self.backend_client.connect("192.168.100.141", 1883, 60)
                self.backend_client.loop_start()
                print("Backend MQTT connection attempt...")
            except Exception as e:
                print(f"Backend MQTT connection failed: {e}")
                
        except Exception as e:
            print(f"MQTT initialization failed: {e}")

    def on_backend_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Backend MQTT connection successful")
            client.subscribe("alert")
            if self.debug:
                print("Backend alert topic subscription completed")
        else:
            print(f"Backend MQTT connection failed: {rc}")

    def on_backend_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            if self.debug:
                print(f"Backend message received [{topic}]: {data}")
            
            if topic == "alert":
                self.process_alert_command(data)
            
        except Exception as e:
            if self.debug:
                print(f"Backend message processing error: {e}")


    def process_alert_command(self, alert_data):
        if not alert_data:
            return
        
        try:
            serial = alert_data.get("serial", "")
            x = alert_data.get("x", 0.0)
            y = alert_data.get("y", 0.0)
            img = alert_data.get("img", "")
            case = alert_data.get("case", "")
            timestamp = alert_data.get("timeStamp", "")
            
            if serial != self.robot_id:
                if self.debug:
                    print(f"Ignoring other robot alert: {serial}")
                return
            
            if self.debug:
                print(f"Alert received: case={case}, position=({x}, {y})")
                print(f"  Time: {timestamp}")
                if img:
                    print(f"  Image: {len(img)} bytes")
            
            if case == "EMERGENCY":
                self.stop()
                if self.debug:
                    print("Emergency situation - Motor stop")
            elif case == "WARNING":
                if self.debug:
                    print("Warning alert received")
            elif case == "OBSTACLE":
                if self.debug:
                    print("Obstacle detection alert")
            
            with self.data_lock:
                self.position["x"] = float(x)
                self.position["y"] = float(y)
            
            self.send_status_to_backend()
            
        except Exception as e:
            if self.debug:
                print(f"Alert processing error: {e}")

    def send_status_to_backend(self):
        if not MQTT_AVAILABLE or not hasattr(self, 'backend_client'):
            return
        
        try:
            with self.data_lock:
                status_data = {
                    "serial": self.robot_id,
                    "state": self.robot_state,
                    "x": float(self.position["x"]),
                    "y": float(self.position["y"]),
                    "speed": float(self.speed),
                    "angle": float(self.motor_angle)
                }
            
            message = json.dumps(status_data)
            self.backend_client.publish("status", message)
            
            if self.debug:
                print(f"Status topic sent: {status_data}")
                
        except Exception as e:
            if self.debug:
                print(f"Status topic send error: {e}")

    def setup_keyboard(self):
        try:
            self.old_terminal = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            if self.debug:
                print("Keyboard setup completed")
        except:
            print("Keyboard setup failed")

    def restore_keyboard(self):
        if self.old_terminal:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal)

    def get_key(self):
        if select.select([sys.stdin], [], [], 0.01)[0]:
            return sys.stdin.read(1)
        return None

    def forward(self):
        self.action = 'forward'
        self.robot_state = "MOVING"
        self.pwm.setDutycycle(self.PWMA, self.speed)
        self.pwm.setLevel(self.AIN1, 0)
        self.pwm.setLevel(self.AIN2, 1)
        self.pwm.setDutycycle(self.PWMB, self.speed)
        self.pwm.setLevel(self.BIN1, 1)
        self.pwm.setLevel(self.BIN2, 0)

    def backward(self):
        self.action = 'backward'
        self.robot_state = "MOVING"
        self.pwm.setDutycycle(self.PWMA, self.speed)
        self.pwm.setLevel(self.AIN1, 1)
        self.pwm.setLevel(self.AIN2, 0)
        self.pwm.setDutycycle(self.PWMB, self.speed)
        self.pwm.setLevel(self.BIN1, 0)
        self.pwm.setLevel(self.BIN2, 1)

    def left(self):
        self.action = 'left'
        self.robot_state = "TURNING"
        self.pwm.setDutycycle(self.PWMA, self.speed)
        self.pwm.setLevel(self.AIN1, 1)
        self.pwm.setLevel(self.AIN2, 0)
        self.pwm.setDutycycle(self.PWMB, self.speed)
        self.pwm.setLevel(self.BIN1, 1)
        self.pwm.setLevel(self.BIN2, 0)

    def right(self):
        self.action = 'right'
        self.robot_state = "TURNING"
        self.pwm.setDutycycle(self.PWMA, self.speed)
        self.pwm.setLevel(self.AIN1, 0)
        self.pwm.setLevel(self.AIN2, 1)
        self.pwm.setDutycycle(self.PWMB, self.speed)
        self.pwm.setLevel(self.BIN1, 0)
        self.pwm.setLevel(self.BIN2, 1)

    def stop(self):
        self.action = 'stop'
        self.robot_state = "IDLE"
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)

    def keep_moving(self):
        last_pwm_refresh = time.time()
        last_status_send = time.time()

        
        while self.running:
            current_time = time.time()
            
            if (current_time - last_pwm_refresh) >= 0.5:
                try:
                    if self.action == 'forward':
                        self.forward()
                    elif self.action == 'backward':
                        self.backward()
                    elif self.action == 'left':
                        self.left()
                    elif self.action == 'right':
                        self.right()
                    
                    last_pwm_refresh = current_time
                    
                except Exception as e:
                    if self.debug:
                        print(f"\nPWM refresh error: {e}")
            
            if (current_time - last_status_send) >= 2.0:
                self.send_status_to_backend()
                last_status_send = current_time
            
            time.sleep(0.1)

    def print_status(self):
        with self.data_lock:
            mode_str = "AI Control" if self.control_mode == 'ai' else "Manual Control"
            pos_str = f"Position: ({self.position['x']:.1f}, {self.position['y']:.1f})"
            
        print(f"\r{mode_str} | Speed: {self.speed}% | {pos_str} | State: {self.robot_state}", end="")
        sys.stdout.flush()

    def main_loop(self):
        self.setup_keyboard()
        self.running = True
        
        self.pwm_thread = threading.Thread(target=self.keep_moving, daemon=True)
        self.pwm_thread.start()
        
        print("\n" + "="*70)
        print("Integrated Motor Control System")
        print("="*70)
        print("Manual Control:")
        print("  W: Forward    S: Backward    A: Left    D: Right")
        print("  SPACE: Stop    +/-: Speed Control")
        print("Control Mode:")
        print("  M: Manual/AI Control Toggle")
        print("System:")
        print("  I: Status Info    Q: Quit")
        print("="*70)
        print(f"MQTT: {'Enabled' if MQTT_AVAILABLE else 'Disabled'}")
        print(f"Robot ID: {self.robot_id}")
        print("Send: status | Receive: alert")
        print("AI Moving: Keyboard Control")
        print("="*70)
        
        try:
            while self.running:
                key = self.get_key()
                
                if key:
                    if key in ['q', 'Q']:
                        print("\nExit...")
                        break
                    
                    elif key in ['m', 'M']:
                        self.control_mode = 'ai' if self.control_mode == 'manual' else 'manual'
                        mode_name = "AI Control" if self.control_mode == 'ai' else "Manual Control"
                        print(f"\n{mode_name} mode")
                        time.sleep(0.5)
                    
                    elif self.control_mode == 'manual':
                        if key == 'w':
                            self.forward()
                            print(f"\rForward (Speed: {self.speed}%)", end="")
                        elif key == 's':
                            self.backward()
                            print(f"\rBackward (Speed: {self.speed}%)", end="")
                        elif key == 'a':
                            self.left()
                            print(f"\rLeft turn (Speed: {self.speed}%)", end="")
                        elif key == 'd':
                            self.right()
                            print(f"\rRight turn (Speed: {self.speed}%)", end="")
                        elif key == ' ':
                            self.stop()
                            print(f"\rStop", end="")
                        elif key == '+':
                            self.speed = min(100, self.speed + 10)
                            print(f"\rSpeed increase: {self.speed}%", end="")
                        elif key == '-':
                            self.speed = max(10, self.speed - 10)
                            print(f"\rSpeed decrease: {self.speed}%", end="")
                        
                        sys.stdout.flush()
                    
                    elif key in ['i', 'I']:
                        print(f"\n" + "="*50)
                        print(f"Robot ID: {self.robot_id}")
                        print(f"Control Mode: {self.control_mode}")
                        print(f"Current Action: {self.action}")
                        print(f"Speed: {self.speed}%")
                        with self.data_lock:
                            print(f"Position: ({self.position['x']:.2f}, {self.position['y']:.2f})")
                            print(f"State: {self.robot_state}")
                        print(f"MQTT: {'Connected' if MQTT_AVAILABLE else 'Disabled'}")
                        print("MQTT Topics:")
                        print("  Send: status (Robot Status)")
                        print("  Receive: alert (Alert)")
                        print("AI Moving: Keyboard Control (API not available)")
                        print("="*50)
                
                if self.control_mode == 'ai':
                    self.print_status()
                
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\nCtrl+C to exit")
        finally:
            self.running = False
            self.stop()
            self.restore_keyboard()
            
            if MQTT_AVAILABLE:
                try:
                    if hasattr(self, 'ai_client'):
                        self.ai_client.loop_stop()
                        self.ai_client.disconnect()
                    if hasattr(self, 'backend_client'):
                        self.backend_client.loop_stop()
                        self.backend_client.disconnect()
                except:
                    pass
            
            print("\nIntegrated motor control system terminated")

def main():
    print("Integrated motor control system")
    print("Keyboard control + AI communication + Backend communication")
    
    try:
        print("\nPreparation check:")
        print("1. sudo pkill -f jtop  (jtop stop)")
        print("2. sudo chmod 666 /dev/i2c-*  (I2C permission)")
        
        if MQTT_AVAILABLE:
            print("3. MQTT broker status:")
            print("   - Backend: 192.168.100.141:1883")
        else:
            print("3. MQTT module not found - Keyboard control only")
        
        print("4. AI Moving: Keyboard control only (API communication not available)")
        
        print("\nPress Enter to proceed...")
        input()
        
        controller = IntegratedMotorControl(
            i2c_address=0x40, 
            i2c_bus=1, 
            debug=True
        )
        controller.main_loop()
        
    except Exception as e:
        print(f"\nError: {e}")
        print("\nSolution:")
        print("1. sudo pkill -f jtop")
        print("2. sudo chmod 666 /dev/i2c-*")
        print("3. Check hardware connection")
        print("4. Check MQTT broker status")

if __name__ == "__main__":
    main()
