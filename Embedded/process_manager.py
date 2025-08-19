import os
import sys
import time
import signal
import subprocess
import threading
from pathlib import Path
from typing import Dict, List

class ProcessManager:
    def __init__(self):
        self.base_dir = Path(__file__).parent
        self.processes: Dict[str, subprocess.Popen] = {}
        
        sys.path.append(str(self.base_dir))
        from config.system_config import get_config
        self.config = get_config()
        
        self.modules = {
            'mqtt-broker': {
                'command': ['mosquitto', '-c', str(self.base_dir / 'config' / 'mosquitto.conf')],
                'cwd': None,
                'env': {},
                'required': False,  
                'description': 'MQTT 브로커'
            },
            'sensors': {
                'command': [sys.executable, '-m', 'sensors'],
                'cwd': self.base_dir / 'sensors',
                'env': {
                    'MQTT_BROKER': self.config.LOCAL_MQTT_BROKER, 
                    'MQTT_PORT': str(self.config.LOCAL_MQTT_PORT)
                },
                'required': True,
                'description': '센서 모듈'
            },
            'mqtt-module': {
                'command': [sys.executable, 'mqtt_manager.py'],
                'cwd': self.base_dir / 'mqtt_module',
                'env': {
                    'MQTT_BROKER': self.config.LOCAL_MQTT_BROKER, 
                    'MQTT_PORT': str(self.config.LOCAL_MQTT_PORT)
                },
                'required': True,
                'description': 'MQTT 통신 모듈'
            }
        }
        
        self._add_optional_modules()
        
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def _add_optional_modules(self):
        optional_modules = {
            'motors': {
                'command': [sys.executable, 'motor_controller.py'],
                'cwd': self.base_dir / 'motors',
                'env': {
                    'MQTT_BROKER': self.config.LOCAL_MQTT_BROKER, 
                    'MQTT_PORT': str(self.config.LOCAL_MQTT_PORT)
                },
                'required': False,
                'description': '모터 제어 모듈'
            },
            'ai-module': {
                'command': [sys.executable, 'ai_processor.py'],
                'cwd': self.base_dir / 'ai_module',
                'env': {
                    'MQTT_BROKER': self.config.LOCAL_MQTT_BROKER, 
                    'MQTT_PORT': str(self.config.LOCAL_MQTT_PORT)
                },
                'required': False,
                'description': 'AI 처리 모듈'
            },
            'communication': {
                'command': [sys.executable, 'communication_manager.py'],
                'cwd': self.base_dir / 'communication_module',
                'env': {
                    'MQTT_BROKER': self.config.LOCAL_MQTT_BROKER, 
                    'MQTT_PORT': str(self.config.LOCAL_MQTT_PORT)
                },
                'required': False,
                'description': '통신 모듈'
            },
            'display': {
                'command': [sys.executable, 'display_manager.py'],
                'cwd': self.base_dir / 'display_module',
                'env': {
                    'MQTT_BROKER': self.config.LOCAL_MQTT_BROKER, 
                    'MQTT_PORT': str(self.config.LOCAL_MQTT_PORT)
                },
                'required': False,
                'description': '디스플레이 모듈'
            },
            'backup': {
                'command': [sys.executable, 'backup_manager.py'],
                'cwd': self.base_dir / 'backup_module',
                'env': {
                    'MQTT_BROKER': self.config.LOCAL_MQTT_BROKER, 
                    'MQTT_PORT': str(self.config.LOCAL_MQTT_PORT)
                },
                'required': False,
                'description': '백업 모듈'
            }
        }
        
        for module_name, config in optional_modules.items():
            main_script = config['cwd'] / config['command'][-1]
            if main_script.exists():
                self.modules[module_name] = config
                print(f"✓ {module_name} module found: {main_script}")
            else:
                print(f"- {module_name} module not found: {main_script}")
    
    def signal_handler(self, signum, frame):
        print(f"\nReceived signal {signum}, shutting down...")
        self.stop_all()
        sys.exit(0)
    
    def check_dependencies(self):
        print("Checking dependencies")
        
        try:
            subprocess.run(['mosquitto', '--help'], capture_output=True, check=True)
            print("mosquitto broker is available")
        except (subprocess.CalledProcessError, FileNotFoundError):
            print("mosquitto broker is not installed")
            print("install command: sudo apt-get install mosquitto")
            return False
        
        return True
    
    def start_module(self, module_name: str) -> bool:
        if module_name not in self.modules:
            print(f"Unknown module: {module_name}")
            return False
        
        if module_name in self.processes and self.processes[module_name].poll() is None:
            print(f"Module {module_name} is already running")
            return True
        
        module_config = self.modules[module_name]
        
        env = os.environ.copy()
        env.update(module_config['env'])
        env['PYTHONPATH'] = f"{self.base_dir}:{env.get('PYTHONPATH', '')}"
        
        try:
            print(f"Starting {module_name} ({module_config['description']})...")
            process = subprocess.Popen(
                module_config['command'],
                cwd=module_config['cwd'],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.processes[module_name] = process
            
            time.sleep(1)
            if process.poll() is None:
                print(f"✓ {module_name} started successfully (PID: {process.pid})")
                return True
            else:
                stdout, stderr = process.communicate()
                print(f"✗ {module_name} failed to start")
                if stdout.strip():
                    print(f"STDOUT: {stdout}")
                if stderr.strip():
                    print(f"STDERR: {stderr}")
                return False
                
        except Exception as e:
            print(f"Error starting {module_name}: {e}")
            return False
    
    def stop_module(self, module_name: str) -> bool:
        if module_name not in self.processes:
            print(f"Module {module_name} is not running")
            return True
        
        process = self.processes[module_name]
        
        try:
            print(f"Stopping {module_name}...")
            process.terminate()
            
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            
            del self.processes[module_name]
            print(f"✓ {module_name} stopped")
            return True
            
        except Exception as e:
            print(f"Error stopping {module_name}: {e}")
            return False
    
    def restart_module(self, module_name: str) -> bool:
        if self.stop_module(module_name):
            time.sleep(2)
            return self.start_module(module_name)
        return False
    
    def start_all(self) -> bool:
        print("Starting all AMR modules...")
        
        if not self.start_module('mqtt-broker'):
            print("Failed to start MQTT broker")
        
        time.sleep(3)  
        
        success = True
        for module_name, config in self.modules.items():
            if module_name != 'mqtt-broker':
                if not self.start_module(module_name):
                    if config.get('required', False):
                        success = False
                    else:
                        print(f"Failed to start {module_name}")
                time.sleep(1)  
        
        return success
    
    def stop_all(self):
        print("Stopping all AMR modules...")
        
        for module_name in list(self.processes.keys()):
            self.stop_module(module_name)
    
    def status(self):
        print("AMR System Status:")
        print("-" * 50)
        
        for module_name, config in self.modules.items():
            if module_name in self.processes:
                process = self.processes[module_name]
                if process.poll() is None:
                    print(f"✓ {module_name}: Running (PID: {process.pid}) - {config['description']}")
                else:
                    print(f"✗ {module_name}: Stopped - {config['description']}")
            else:
                print(f"- {module_name}: Not started - {config['description']}")
    
    def list_modules(self):
        print("Available modules:")
        print("-" * 30)
        for module_name, config in self.modules.items():
            status = "✓" if module_name in self.processes and self.processes[module_name].poll() is None else "-"
            print(f"{status} {module_name}: {config['description']}")
    
    def monitor(self):
        print("Monitoring AMR modules... (Press Ctrl+C to stop)")
        
        try:
            while True:
                for module_name in list(self.processes.keys()):
                    process = self.processes[module_name]
                    if process.poll() is not None:
                        print(f"Module {module_name} died, restarting...")
                        self.restart_module(module_name)
                
                time.sleep(5)
                
        except KeyboardInterrupt:
            print("\nStopping monitoring...")
            self.stop_all()

def main():
    manager = ProcessManager()
    
    if len(sys.argv) < 2:
        print("AMR Process Manager")
        print("Usage:")
        print("  python process_manager.py check")
        print("  python process_manager.py list")
        print("  python process_manager.py start <module>")
        print("  python process_manager.py stop <module>")
        print("  python process_manager.py restart <module>")
        print("  python process_manager.py start-all")
        print("  python process_manager.py stop-all")
        print("  python process_manager.py status")
        print("  python process_manager.py monitor")
        return
    
    command = sys.argv[1]
    
    if command == 'check':
        manager.check_dependencies()
    elif command == 'list':
        manager.list_modules()
    elif command == 'start' and len(sys.argv) > 2:
        module_name = sys.argv[2]
        manager.start_module(module_name)
    elif command == 'stop' and len(sys.argv) > 2:
        module_name = sys.argv[2]
        manager.stop_module(module_name)
    elif command == 'restart' and len(sys.argv) > 2:
        module_name = sys.argv[2]
        manager.restart_module(module_name)
    elif command == 'start-all':
        manager.start_all()
    elif command == 'stop-all':
        manager.stop_all()
    elif command == 'status':
        manager.status()
    elif command == 'monitor':
        manager.monitor()
    else:
        print("Invalid command")

if __name__ == '__main__':
    main()
