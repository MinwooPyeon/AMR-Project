import sys
import os
import time
import signal
import threading

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from mqtt.sensor_data_transmitter import SensorDataTransmitter
from utils.logger import mqtt_logger

class PeriodicStatusSender:
    def __init__(self, robot_id: str = None, broker: str = None, port: int = None):
        from config.system_config import get_config
        config = get_config()
        
        self.robot_id = robot_id or config.SYSTEM_NAME
        self.broker = broker or config.MQTT_BROKER
        self.port = port or config.MQTT_PORT
        self.transmitter = SensorDataTransmitter(self.robot_id, self.broker, self.port)
        self.running = False
        
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        self.stop()
    
    def start(self):
        if not self.transmitter.connect_mqtt():
            return False
        
        self.transmitter.update_embedded_data(
            serial=self.robot_id,
            state="RUNNING",
            x=0.0,
            y=0.0,
            speed=0.0,
            angle=0.0
        )
        
        if self.transmitter.start_periodic_sending(interval=1.0):
            self.running = True
            
            self._start_status_simulation()
            
            return True
        else:
            return False
    
    def stop(self):
        if self.running:
            self.running = False
            self.transmitter.stop_periodic_sending()
            self.transmitter.disconnect_mqtt()
    
    def _start_status_simulation(self):
        def simulate_movement():
            x, y = 0.0, 0.0
            speed = 1.0
            angle = 0.0
            
            while self.running:
                x += speed * 0.1
                y += speed * config.MOTOR_SPEED_MULTIPLIER / 1000  
                angle += 5.0
                
                if angle >= config.ROTATION_360_DEGREES:
                    angle = 0.0
                
                self.transmitter.update_embedded_data(
                    x=x,
                    y=y,
                    speed=speed,
                    angle=angle
                )
                
                time.sleep(1.0)
        
        simulation_thread = threading.Thread(target=simulate_movement, daemon=True)
        simulation_thread.start()
    
    def run_forever(self):
        if self.start():
            try:
                while self.running:
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
            finally:
                self.stop()

def main():
    sender = PeriodicStatusSender()
    sender.run_forever()

if __name__ == "__main__":
    main() 