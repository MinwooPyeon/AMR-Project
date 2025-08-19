import os
import sys
import time
import hashlib
import hmac
import secrets
import logging
import json
import threading
from typing import Dict, List, Optional, Callable
from pathlib import Path
from datetime import datetime, timedelta
import ssl
import socket
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
import base64

sys.path.append(str(Path(__file__).parent.parent))
from config.system_config import get_config

class SecurityManager:
    
    def __init__(self):
        self.config = get_config()
        self.logger = self._setup_logger()
        
        self.security_config = {
            'max_login_attempts': 3,
            'lockout_duration': 300,  
            'session_timeout': 3600,  
            'password_min_length': 8,
            'require_special_chars': True,
            'encryption_enabled': True,
            'ssl_required': True,
            'rate_limit_requests': 100,  
            'rate_limit_window': 60,  
        }
        
        self.failed_login_attempts = {}
        self.active_sessions = {}
        self.rate_limit_counters = {}
        self.encryption_key = None
        self.security_alerts = []
        
        self.security_callbacks = []
        
        self.lock = threading.Lock()
        
        self._initialize_security()
    
    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger('security_manager')
        logger.setLevel(logging.INFO)
        
        log_dir = Path('logs')
        log_dir.mkdir(exist_ok=True)
        
        file_handler = logging.FileHandler(log_dir / 'security.log')
        file_handler.setLevel(logging.INFO)
        
        formatter = logging.Formatter(
            '[%(asctime)s] %(levelname)s: %(message)s'
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        return logger
    
    def _initialize_security(self):
        try:
            self._setup_encryption()
            
            self._verify_ssl_certificates()
            
            self._apply_security_policies()
            
            self._start_security_monitoring()
            
            self.logger.info("Security manager initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Security initialization failed: {e}")
            raise
    
    def _setup_encryption(self):
        key_file = Path('security/encryption.key')
        key_file.parent.mkdir(exist_ok=True)
        
        if key_file.exists():
            with open(key_file, 'rb') as f:
                self.encryption_key = f.read()
        else:
            self.encryption_key = Fernet.generate_key()
            with open(key_file, 'wb') as f:
                f.write(self.encryption_key)
        
        self.cipher = Fernet(self.encryption_key)
    
    def _verify_ssl_certificates(self):
        if not self.security_config['ssl_required']:
            return
        
        cert_paths = [
            'security/certs/server.crt',
            'security/certs/ca.crt'
        ]
        
        for cert_path in cert_paths:
            if not Path(cert_path).exists():
                self.logger.warning(f"SSL certificate not found: {cert_path}")
    
    def _apply_security_policies(self):
        self._set_secure_file_permissions()
        
        self._configure_network_security()
        
        self._configure_system_security()
    
    def _set_secure_file_permissions(self):
        sensitive_files = [
            'security/encryption.key',
            'config/system_config.py',
            'logs/security.log'
        ]
        
        for file_path in sensitive_files:
            if Path(file_path).exists():
                os.chmod(file_path, 0o600)
    
    def _configure_network_security(self):
        self._secure_mqtt_broker()
        
        self._secure_websocket()
    
    def _secure_mqtt_broker(self):
        mqtt_config = {
            'allow_anonymous': False,
            'password_file': 'security/mqtt_passwords.txt',
            'acl_file': 'security/mqtt_acls.txt',
            'certificate_file': 'security/certs/server.crt',
            'key_file': 'security/certs/server.key',
            'require_certificate': True
        }
        
        self._update_mqtt_config(mqtt_config)
    
    def _update_mqtt_config(self, config: Dict):
        config_path = Path('config/mosquitto.conf')
        if config_path.exists():
            with open(config_path, 'a') as f:
                f.write('\n# Security Settings\n')
                f.write(f'allow_anonymous {str(config["allow_anonymous"]).lower()}\n')
                f.write(f'password_file {config["password_file"]}\n')
                f.write(f'acl_file {config["acl_file"]}\n')
                if config.get('certificate_file'):
                    f.write(f'certfile {config["certificate_file"]}\n')
                if config.get('key_file'):
                    f.write(f'keyfile {config["key_file"]}\n')
    
    def _secure_websocket(self):
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE
        
        cert_file = Path('security/certs/server.crt')
        key_file = Path('security/certs/server.key')
        
        if cert_file.exists() and key_file.exists():
            ssl_context.load_cert_chain(cert_file, key_file)
    
    def _configure_system_security(self):
        self._isolate_processes()
        
        self._set_resource_limits()
    
    def _isolate_processes(self):
        module_users = {
            'sensors': 'amr_sensors',
            'motors': 'amr_motors',
            'mqtt-module': 'amr_mqtt',
            'ai-module': 'amr_ai'
        }
        
        self._create_user_setup_script(module_users)
    
    def _create_user_setup_script(self, users: Dict[str, str]):
        script_content = ""
        for module, user in users.items():
            script_content += f"""
sudo useradd -r -s /bin/false {user}
sudo mkdir -p /home/{user}
sudo chown {user}:{user} /home/{user}
"""
        
        script_path = Path('security/setup_users.sh')
        script_path.parent.mkdir(exist_ok=True)
        
        with open(script_path, 'w') as f:
            f.write(script_content)
        
        os.chmod(script_path, 0o755)
    
    def _set_resource_limits(self):
        limits_config = {
            'max_memory_per_process': '512M',
            'max_cpu_per_process': '50%',
            'max_file_descriptors': 1024,
            'max_processes': 100
        }
        
        self._create_limits_config(limits_config)
    
    def _create_limits_config(self, limits: Dict):
        config_content = """
amr_sensors soft memlock 512000
amr_sensors hard memlock 512000
amr_motors soft memlock 512000
amr_motors hard memlock 512000
amr_mqtt soft memlock 512000
amr_mqtt hard memlock 512000
amr_ai soft memlock 1024000
amr_ai hard memlock 1024000

# CPU limits
amr_sensors soft cpu 50
amr_sensors hard cpu 50
amr_motors soft cpu 50
amr_motors hard cpu 50
amr_mqtt soft cpu 30
amr_mqtt hard cpu 30
amr_ai soft cpu 80
amr_ai hard cpu 80

# File descriptor limits
amr_sensors soft nofile 1024
amr_sensors hard nofile 1024
amr_motors soft nofile 1024
amr_motors hard nofile 1024
amr_mqtt soft nofile 2048
amr_mqtt hard nofile 2048
amr_ai soft nofile 2048
amr_ai hard nofile 2048
"""
        
        config_path = Path('security/amr_limits.conf')
        config_path.parent.mkdir(exist_ok=True)
        
        with open(config_path, 'w') as f:
            f.write(config_content)
    
    def _start_security_monitoring(self):
        monitoring_thread = threading.Thread(
            target=self._security_monitoring_loop,
            daemon=True
        )
        monitoring_thread.start()
    
    def _security_monitoring_loop(self):
        while True:
            try:
                self._check_system_security()
                
                self._check_network_security()
                
                self._check_file_integrity()
                
                time.sleep(300)
                
            except Exception as e:
                self.logger.error(f"Security monitoring error: {e}")
                time.sleep(60)
    
    def _check_system_security(self):
        self._check_process_security()
        
        self._check_file_permissions()
        
        self._check_login_attempts()
    
    def _check_process_security(self):
        expected_processes = [
            'mosquitto',
            'python3',
            'amr_system'
        ]
        
        pass
    
    def _check_file_permissions(self):
        sensitive_files = [
            'security/encryption.key',
            'config/system_config.py'
        ]
        
        for file_path in sensitive_files:
            if Path(file_path).exists():
                stat = os.stat(file_path)
                if stat.st_mode & 0o777 != 0o600:
                    self._raise_security_alert(
                        'file_permission',
                        f'Insecure file permissions: {file_path}'
                    )
    
    def _check_login_attempts(self):
        current_time = time.time()
        
        with self.lock:
            expired_attempts = [
                ip for ip, data in self.failed_login_attempts.items()
                if current_time - data['last_attempt'] > self.security_config['lockout_duration']
            ]
            
            for ip in expired_attempts:
                del self.failed_login_attempts[ip]
    
    def _check_network_security(self):
        self._detect_port_scanning()
        
        self._check_abnormal_connections()
    
    def _detect_port_scanning(self):
        pass
    
    def _check_abnormal_connections(self):
        expected_ports = [
            self.config.MQTT_PORT,
            self.config.WEBSOCKET_PORT,
            self.config.HTTP_PORT
        ]
        
        pass
    
    def _check_file_integrity(self):
        important_files = [
            'config/system_config.py',
            'process_manager.py',
            'run_amr.sh'
        ]
        
        for file_path in important_files:
            if Path(file_path).exists():
                current_hash = self._calculate_file_hash(file_path)
                stored_hash = self._get_stored_hash(file_path)
                
                if stored_hash and current_hash != stored_hash:
                    self._raise_security_alert(
                        'file_integrity',
                        f'File integrity check failed: {file_path}'
                    )
                else:
                    self._store_file_hash(file_path, current_hash)
    
    def _calculate_file_hash(self, file_path: str) -> str:
        hash_sha256 = hashlib.sha256()
        
        with open(file_path, 'rb') as f:
            for chunk in iter(lambda: f.read(4096), b""):
                hash_sha256.update(chunk)
        
        return hash_sha256.hexdigest()
    
    def _get_stored_hash(self, file_path: str) -> Optional[str]:
        hash_file = Path('security/file_hashes.json')
        
        if hash_file.exists():
            try:
                with open(hash_file, 'r') as f:
                    hashes = json.load(f)
                    return hashes.get(file_path)
            except:
                pass
        
        return None
    
    def _store_file_hash(self, file_path: str, file_hash: str):
        hash_file = Path('security/file_hashes.json')
        hash_file.parent.mkdir(exist_ok=True)
        
        try:
            if hash_file.exists():
                with open(hash_file, 'r') as f:
                    hashes = json.load(f)
            else:
                hashes = {}
            
            hashes[file_path] = file_hash
            
            with open(hash_file, 'w') as f:
                json.dump(hashes, f, indent=2)
                
        except Exception as e:
            self.logger.error(f"Failed to store file hash: {e}")
    
    def authenticate_user(self, username: str, password: str, client_ip: str) -> bool:
        if self._is_ip_locked(client_ip):
            self.logger.warning(f"Login attempt from locked IP: {client_ip}")
            return False
        
        if self._verify_password(username, password):
            self._reset_failed_attempts(client_ip)
            self._create_session(username, client_ip)
            self.logger.info(f"Successful login: {username} from {client_ip}")
            return True
        else:
            self._record_failed_attempt(client_ip)
            self.logger.warning(f"Failed login attempt: {username} from {client_ip}")
            return False
    
    def _is_ip_locked(self, client_ip: str) -> bool:
        with self.lock:
            if client_ip in self.failed_login_attempts:
                data = self.failed_login_attempts[client_ip]
                if (data['count'] >= self.security_config['max_login_attempts'] and
                    time.time() - data['last_attempt'] < self.security_config['lockout_duration']):
                    return True
        return False
    
    def _verify_password(self, username: str, password: str) -> bool:
        if len(password) < self.security_config['password_min_length']:
            return False
        
        if self.security_config['require_special_chars']:
            if not any(c in '!@#$%^&*()_+-=[]{}|;:,.<>?' for c in password):
                return False
        
        valid_users = {
            'admin': 'admin123!',
            'operator': 'op123!',
            'viewer': 'view123!',
            'minwoo': 'minwoo',
            'viewer': 'view123!'
        }
        
        return username in valid_users and valid_users[username] == password
    
    def _record_failed_attempt(self, client_ip: str):
        with self.lock:
            if client_ip in self.failed_login_attempts:
                self.failed_login_attempts[client_ip]['count'] += 1
                self.failed_login_attempts[client_ip]['last_attempt'] = time.time()
            else:
                self.failed_login_attempts[client_ip] = {
                    'count': 1,
                    'last_attempt': time.time()
                }
    
    def _reset_failed_attempts(self, client_ip: str):
        with self.lock:
            if client_ip in self.failed_login_attempts:
                del self.failed_login_attempts[client_ip]
    
    def _create_session(self, username: str, client_ip: str):
        session_id = secrets.token_urlsafe(32)
        session_data = {
            'username': username,
            'client_ip': client_ip,
            'created_at': time.time(),
            'last_activity': time.time()
        }
        
        with self.lock:
            self.active_sessions[session_id] = session_data
        
        return session_id
    
    def validate_session(self, session_id: str) -> bool:
        with self.lock:
            if session_id in self.active_sessions:
                session = self.active_sessions[session_id]
                
                if time.time() - session['last_activity'] > self.security_config['session_timeout']:
                    del self.active_sessions[session_id]
                    return False
                
                session['last_activity'] = time.time()
                return True
        
        return False
    
    def encrypt_data(self, data: str) -> str:
        if not self.security_config['encryption_enabled']:
            return data
        
        try:
            encrypted_data = self.cipher.encrypt(data.encode())
            return base64.b64encode(encrypted_data).decode()
        except Exception as e:
            self.logger.error(f"Encryption failed: {e}")
            return data
    
    def decrypt_data(self, encrypted_data: str) -> str:
        if not self.security_config['encryption_enabled']:
            return encrypted_data
        
        try:
            decoded_data = base64.b64decode(encrypted_data.encode())
            decrypted_data = self.cipher.decrypt(decoded_data)
            return decrypted_data.decode()
        except Exception as e:
            self.logger.error(f"Decryption failed: {e}")
            return encrypted_data
    
    def check_rate_limit(self, client_ip: str) -> bool:
        current_time = time.time()
        
        with self.lock:
            if client_ip in self.rate_limit_counters:
                counter = self.rate_limit_counters[client_ip]
                
                if current_time - counter['window_start'] > self.security_config['rate_limit_window']:
                    counter['count'] = 1
                    counter['window_start'] = current_time
                else:
                    counter['count'] += 1
                    
                    if counter['count'] > self.security_config['rate_limit_requests']:
                        return False
            else:
                self.rate_limit_counters[client_ip] = {
                    'count': 1,
                    'window_start': current_time
                }
        
        return True
    
    def _raise_security_alert(self, alert_type: str, message: str):
        alert = {
            'type': alert_type,
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'severity': 'high'
        }
        
        self.security_alerts.append(alert)
        self.logger.warning(f"Security alert: {alert_type} - {message}")
        
        for callback in self.security_callbacks:
            try:
                callback(alert)
            except Exception as e:
                self.logger.error(f"Security callback error: {e}")
    
    def add_security_callback(self, callback: Callable):
        self.security_callbacks.append(callback)
    
    def get_security_status(self) -> Dict:
        return {
            'failed_login_attempts': len(self.failed_login_attempts),
            'active_sessions': len(self.active_sessions),
            'security_alerts': len(self.security_alerts),
            'encryption_enabled': self.security_config['encryption_enabled'],
            'ssl_required': self.security_config['ssl_required']
        }
    
    def get_security_alerts(self) -> List[Dict]:
        return self.security_alerts.copy()

security_manager = SecurityManager()

def get_security_manager() -> SecurityManager:
    return security_manager

if __name__ == "__main__":
    manager = get_security_manager()
    
    print("=== Security Manager Test ===")
    print(f"Security Status: {manager.get_security_status()}")
    
    print("\nAuthentication Test:")
    print(f"Valid login (admin): {manager.authenticate_user('admin', 'admin123!', '192.168.1.100')}")
    print(f"Valid login (minwoo): {manager.authenticate_user('minwoo', 'minwoo', '192.168.1.100')}")
    print(f"Valid login: {manager.authenticate_user('admin', 'admin123!', '192.168.1.100')}")
    print(f"Invalid login: {manager.authenticate_user('admin', 'wrong', '192.168.1.100')}")
    
    print("\nEncryption Test:")
    test_data = "sensitive data"
    encrypted = manager.encrypt_data(test_data)
    decrypted = manager.decrypt_data(encrypted)
    print(f"Original: {test_data}")
    print(f"Encrypted: {encrypted}")
    print(f"Decrypted: {decrypted}")
    print(f"Match: {test_data == decrypted}")
    
    print("\nRate Limit Test:")
    for i in range(5):
        allowed = manager.check_rate_limit('192.168.1.100')
        print(f"Request {i+1}: {'Allowed' if allowed else 'Blocked'}")
    
    print("\nSecurity Manager test completed!")
