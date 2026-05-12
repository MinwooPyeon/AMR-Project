import os
import sys
import time
import json
import logging
import threading
from typing import Dict, List, Optional, Callable, Any
from pathlib import Path
from datetime import datetime
import smbus2 as smbus

sys.path.append(str(Path(__file__).parent.parent))
from config.system_config import get_config


class BatteryMonitor:

    def __init__(self):
        self.config = get_config()
        self.logger = self._setup_logger()

        self.battery_config = {
            'i2c_bus': 1,
            'battery_address': 0x36,
            'capacity_mah': 3000,
            'nominal_voltage': 3.7,
            'max_voltage': 4.2,
            'min_voltage': 3.0,
            'low_battery_threshold': 20.0,
            'critical_battery_threshold': 10.0,
            'charging_current_threshold': 100,
            'temperature_threshold': 60.0,
            'monitoring_interval': 1.0,
            'data_logging_interval': 60.0
        }

        self.battery_status = {
            'voltage': 0.0,
            'current': 0.0,
            'percentage': 0.0,
            'temperature': 0.0,
            'is_charging': False,
            'is_full': False,
            'is_low': False,
            'is_critical': False,
            'remaining_capacity': 0.0,
            'full_capacity': 0.0,
            'time_to_empty': 0.0,
            'time_to_full': 0.0,
            'cycle_count': 0,
            'health': 100.0
        }

        self.power_consumption = {
            'current_power': 0.0,
            'average_power': 0.0,
            'peak_power': 0.0,
            'total_energy': 0.0,
            'power_history': [],
        }

        self.alerts = {
            'low_battery': False,
            'critical_battery': False,
            'overheating': False,
            'overcurrent': False,
        }

        self.status_callbacks: List[Callable] = []
        self.alert_callbacks: List[Callable] = []

        self.lock = threading.Lock()
        self.i2c_bus: Optional[smbus.SMBus] = None

        self._stop_event = threading.Event()
        self.monitoring_thread: Optional[threading.Thread] = None

        self._initialize_battery_monitor()

    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger('battery_monitor')
        logger.setLevel(logging.INFO)

        if not logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter('[%(asctime)s] %(name)s - %(levelname)s: %(message)s'))
            logger.addHandler(handler)

        return logger

    def _initialize_battery_monitor(self):
        try:
            self._setup_i2c()
            self._initialize_battery_gauge()
            self._read_battery_status()
            self._start_monitoring()
            self.logger.info("Battery monitor initialized")
        except Exception as e:
            self.logger.error(f"Battery monitor initialization failed: {e}")
            raise

    def _setup_i2c(self):
        try:
            self.i2c_bus = smbus.SMBus(self.battery_config['i2c_bus'])
            self.logger.info(f"I2C bus {self.battery_config['i2c_bus']} initialized")
        except Exception as e:
            self.logger.warning(f"I2C setup failed, running in simulation mode: {e}")
            self.i2c_bus = None

    def _initialize_battery_gauge(self):
        if self.i2c_bus is None:
            self.logger.info("Simulation mode active")
            return
        self.logger.info("Battery gauge initialized")

    def _start_monitoring(self):
        self._stop_event.clear()
        self.monitoring_thread = threading.Thread(
            target=self._monitoring_loop,
            daemon=True,
            name="battery-monitor"
        )
        self.monitoring_thread.start()
        self.logger.info("Battery monitoring started")

    def _monitoring_loop(self):
        last_log_time = time.time()

        while not self._stop_event.is_set():
            try:
                self._read_battery_status()
                self._calculate_power_consumption()
                self._check_alerts()
                self._notify_status_callbacks()

                if time.time() - last_log_time >= self.battery_config['data_logging_interval']:
                    self._log_battery_data()
                    last_log_time = time.time()

            except Exception as e:
                self.logger.error(f"Battery monitoring error: {e}")

            self._stop_event.wait(timeout=self.battery_config['monitoring_interval'])

    def _read_battery_status(self):
        try:
            if self.i2c_bus is None:
                self._simulate_battery_status()
            else:
                self._read_battery_hardware()
            self._update_battery_status()
        except Exception as e:
            self.logger.error(f"Failed to read battery status: {e}")

    def _simulate_battery_status(self):
        import random
        self.battery_status['voltage'] = 3.7 + random.uniform(-0.1, 0.1)
        if self.battery_status['is_charging']:
            self.battery_status['current'] = random.uniform(50, 200)
        else:
            self.battery_status['current'] = random.uniform(-500, -100)
        self.battery_status['temperature'] = random.uniform(20, 35)
        self.battery_status['remaining_capacity'] = random.uniform(2000, 3000)
        self.battery_status['full_capacity'] = self.battery_config['capacity_mah']
        self.battery_status['percentage'] = (
            self.battery_status['remaining_capacity'] / self.battery_status['full_capacity'] * 100
        )

    def _read_battery_hardware(self):
        try:
            vcell_msb = self.i2c_bus.read_byte_data(self.battery_config['battery_address'], 0x02)
            vcell_lsb = self.i2c_bus.read_byte_data(self.battery_config['battery_address'], 0x03)
            vcell_raw = (vcell_msb << 4) | (vcell_lsb >> 4)
            self.battery_status['voltage'] = vcell_raw * 0.000078125

            soc_msb = self.i2c_bus.read_byte_data(self.battery_config['battery_address'], 0x04)
            soc_lsb = self.i2c_bus.read_byte_data(self.battery_config['battery_address'], 0x05)
            soc_raw = (soc_msb << 8) | soc_lsb
            self.battery_status['percentage'] = soc_raw / 256.0

        except Exception as e:
            self.logger.error(f"Hardware read failed, switching to simulation: {e}")
            self._simulate_battery_status()

    def _update_battery_status(self):
        with self.lock:
            s = self.battery_status
            s['is_charging'] = s['current'] > self.battery_config['charging_current_threshold']
            s['is_full'] = s['percentage'] >= 95.0 and s['is_charging']
            s['is_low'] = s['percentage'] <= self.battery_config['low_battery_threshold']
            s['is_critical'] = s['percentage'] <= self.battery_config['critical_battery_threshold']
            s['health'] = min(100.0, s['full_capacity'] / self.battery_config['capacity_mah'] * 100)

            if s['current'] != 0:
                if s['is_charging']:
                    remaining = s['full_capacity'] - s['remaining_capacity']
                    s['time_to_full'] = remaining / abs(s['current']) * 60
                    s['time_to_empty'] = 0
                else:
                    s['time_to_empty'] = s['remaining_capacity'] / abs(s['current']) * 60
                    s['time_to_full'] = 0

    def _calculate_power_consumption(self):
        with self.lock:
            current_power = (
                self.battery_status['voltage'] * abs(self.battery_status['current']) / 1000.0
            )
            self.power_consumption['current_power'] = current_power

            timestamp = time.time()
            history = self.power_consumption['power_history']
            history.append({'timestamp': timestamp, 'power': current_power})

            if len(history) > 1000:
                self.power_consumption['power_history'] = history[-1000:]

            recent = [d['power'] for d in history[-100:]]
            if recent:
                self.power_consumption['average_power'] = sum(recent) / len(recent)
                self.power_consumption['peak_power'] = max(
                    self.power_consumption['peak_power'], current_power
                )

            if len(history) >= 2:
                last = history[-2]
                dt = timestamp - last['timestamp']
                self.power_consumption['total_energy'] += (current_power + last['power']) / 2 * dt / 3600

    def _check_alerts(self):
        with self.lock:
            s = self.battery_status
            a = self.alerts

            if s['is_low'] and not a['low_battery']:
                a['low_battery'] = True
                self._raise_alert('low_battery', f"Low battery: {s['percentage']:.1f}%")
            elif not s['is_low']:
                a['low_battery'] = False

            if s['is_critical'] and not a['critical_battery']:
                a['critical_battery'] = True
                self._raise_alert('critical_battery', f"Critical battery: {s['percentage']:.1f}%")
            elif not s['is_critical']:
                a['critical_battery'] = False

            if s['temperature'] > self.battery_config['temperature_threshold'] and not a['overheating']:
                a['overheating'] = True
                self._raise_alert('overheating', f"Overheating: {s['temperature']:.1f}°C")
            elif s['temperature'] <= self.battery_config['temperature_threshold']:
                a['overheating'] = False

            if abs(s['current']) > 2000 and not a['overcurrent']:
                a['overcurrent'] = True
                self._raise_alert('overcurrent', f"Overcurrent: {s['current']:.1f}mA")
            elif abs(s['current']) <= 2000:
                a['overcurrent'] = False

    def _raise_alert(self, alert_type: str, message: str):
        alert = {
            'type': alert_type,
            'message': message,
            'timestamp': time.time(),
            'battery_status': self.battery_status.copy()
        }
        self.logger.warning(f"Battery alert: {alert_type} - {message}")
        for callback in self.alert_callbacks:
            try:
                callback(alert)
            except Exception as e:
                self.logger.error(f"Alert callback error: {e}")

    def _notify_status_callbacks(self):
        status_data = {
            'battery_status': self.battery_status.copy(),
            'power_consumption': {
                k: v for k, v in self.power_consumption.items() if k != 'power_history'
            },
            'alerts': self.alerts.copy(),
            'timestamp': time.time()
        }
        for callback in self.status_callbacks:
            try:
                callback(status_data)
            except Exception as e:
                self.logger.error(f"Status callback error: {e}")

    def _log_battery_data(self):
        log_data = {
            'timestamp': datetime.now().isoformat(),
            'battery_status': self.battery_status.copy(),
            'power_consumption': {
                'current_power': self.power_consumption['current_power'],
                'average_power': self.power_consumption['average_power'],
                'total_energy': self.power_consumption['total_energy']
            },
            'alerts': self.alerts.copy()
        }

        log_file = Path('logs/battery_data.json')
        log_file.parent.mkdir(exist_ok=True)

        try:
            logs = []
            if log_file.exists():
                with open(log_file, 'r') as f:
                    logs = json.load(f)

            logs.append(log_data)
            if len(logs) > 1000:
                logs = logs[-1000:]

            with open(log_file, 'w') as f:
                json.dump(logs, f, indent=2)

        except Exception as e:
            self.logger.error(f"Failed to log battery data: {e}")

    def get_battery_status(self) -> Dict[str, Any]:
        with self.lock:
            return self.battery_status.copy()

    def get_power_consumption(self) -> Dict[str, Any]:
        with self.lock:
            return {k: v for k, v in self.power_consumption.items() if k != 'power_history'}

    def get_alerts(self) -> Dict[str, bool]:
        with self.lock:
            return self.alerts.copy()

    def get_battery_health(self) -> Dict[str, Any]:
        with self.lock:
            return {
                'health_percentage': self.battery_status['health'],
                'cycle_count': self.battery_status['cycle_count'],
                'full_capacity': self.battery_status['full_capacity'],
                'design_capacity': self.battery_config['capacity_mah'],
            }

    def add_status_callback(self, callback: Callable):
        self.status_callbacks.append(callback)

    def add_alert_callback(self, callback: Callable):
        self.alert_callbacks.append(callback)

    def stop_monitoring(self):
        self._stop_event.set()
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=5)
        if self.i2c_bus:
            try:
                self.i2c_bus.close()
            except Exception:
                pass
        self.logger.info("Battery monitoring stopped")

    def update_config(self, new_config: Dict[str, Any]):
        with self.lock:
            self.battery_config.update(new_config)


_battery_monitor: Optional[BatteryMonitor] = None
_monitor_lock = threading.Lock()


def get_battery_monitor() -> BatteryMonitor:
    global _battery_monitor
    if _battery_monitor is None:
        with _monitor_lock:
            if _battery_monitor is None:
                _battery_monitor = BatteryMonitor()
    return _battery_monitor


if __name__ == "__main__":
    monitor = get_battery_monitor()

    def status_callback(data):
        s = data['battery_status']
        print(f"Battery: {s['percentage']:.1f}% | {s['voltage']:.2f}V | {s['current']:.1f}mA")

    def alert_callback(alert):
        print(f"ALERT: {alert['type']} - {alert['message']}")

    monitor.add_status_callback(status_callback)
    monitor.add_alert_callback(alert_callback)

    print("Monitoring for 10 seconds...")
    time.sleep(10)

    print(f"Status: {monitor.get_battery_status()}")
    monitor.stop_monitoring()
    print("Done!")
