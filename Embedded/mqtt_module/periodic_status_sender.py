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
    def __init__(self, robot_id: str = "AMR001", broker: str = "192.168.100.141", port: int = 1883):
        self.robot_id = robot_id
        self.transmitter = SensorDataTransmitter(robot_id, broker, port)
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
                y += speed * 0.05
                angle += 5.0
                
                if angle >= 360.0:
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