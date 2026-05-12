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
from datetime import datetime
import ssl
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
            'ssl_required': False,
            'rate_limit_requests': 100,
            'rate_limit_window': 60,
        }

        self.failed_login_attempts: Dict = {}
        self.active_sessions: Dict = {}
        self.rate_limit_counters: Dict = {}
        self.security_alerts: List = []
        self.security_callbacks: List[Callable] = []

        self.lock = threading.Lock()
        self._stop_event = threading.Event()

        self._initialize_security()

    def _setup_logger(self) -> logging.Logger:
        logger = logging.getLogger('security_manager')
        logger.setLevel(logging.INFO)

        if not logger.handlers:
            log_dir = Path('logs')
            log_dir.mkdir(exist_ok=True)

            file_handler = logging.FileHandler(log_dir / 'security.log')
            file_handler.setLevel(logging.INFO)
            formatter = logging.Formatter('[%(asctime)s] %(levelname)s: %(message)s')
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)

        return logger

    def _initialize_security(self):
        try:
            self._setup_encryption()
            self._apply_security_policies()
            self._start_security_monitoring()
            self.logger.info("Security manager initialized")
        except Exception as e:
            self.logger.error(f"Security initialization failed: {e}")
            raise

    def _setup_encryption(self):
        try:
            from cryptography.fernet import Fernet
        except ImportError:
            self.logger.warning("cryptography package not installed; encryption disabled")
            self.security_config['encryption_enabled'] = False
            self.cipher = None
            return

        key_file = Path('security/encryption.key')
        key_file.parent.mkdir(exist_ok=True)

        if key_file.exists():
            with open(key_file, 'rb') as f:
                self.encryption_key = f.read()
        else:
            self.encryption_key = Fernet.generate_key()
            with open(key_file, 'wb') as f:
                f.write(self.encryption_key)
            os.chmod(key_file, 0o600)

        self.cipher = Fernet(self.encryption_key)

    def _apply_security_policies(self):
        self._set_secure_file_permissions()

    def _set_secure_file_permissions(self):
        sensitive_files = [
            'security/encryption.key',
            'logs/security.log'
        ]
        for file_path in sensitive_files:
            p = Path(file_path)
            if p.exists():
                os.chmod(p, 0o600)

    def _start_security_monitoring(self):
        monitoring_thread = threading.Thread(
            target=self._security_monitoring_loop,
            daemon=True,
            name="security-monitor"
        )
        monitoring_thread.start()

    def _security_monitoring_loop(self):
        while not self._stop_event.is_set():
            try:
                self._check_file_permissions()
                self._cleanup_expired_login_attempts()
            except Exception as e:
                self.logger.error(f"Security monitoring error: {e}")

            self._stop_event.wait(timeout=300)

    def _check_file_permissions(self):
        sensitive_files = ['security/encryption.key']
        for file_path in sensitive_files:
            p = Path(file_path)
            if p.exists():
                stat = os.stat(p)
                if stat.st_mode & 0o777 != 0o600:
                    self._raise_security_alert('file_permission', f'Insecure permissions: {file_path}')
                    os.chmod(p, 0o600)

    def _cleanup_expired_login_attempts(self):
        current_time = time.time()
        with self.lock:
            expired = [
                ip for ip, data in self.failed_login_attempts.items()
                if current_time - data['last_attempt'] > self.security_config['lockout_duration']
            ]
            for ip in expired:
                del self.failed_login_attempts[ip]

    def shutdown(self):
        self._stop_event.set()

    def authenticate_user(self, username: str, password: str, client_ip: str) -> bool:
        if self._is_ip_locked(client_ip):
            self.logger.warning(f"Login blocked for locked IP: {client_ip}")
            return False

        if self._verify_password(username, password):
            self._reset_failed_attempts(client_ip)
            session_id = self._create_session(username, client_ip)
            self.logger.info(f"Successful login: {username} from {client_ip}")
            return True
        else:
            self._record_failed_attempt(client_ip)
            self.logger.warning(f"Failed login: {username} from {client_ip}")
            return False

    def _verify_password(self, username: str, password: str) -> bool:
        if len(password) < self.security_config['password_min_length']:
            return False

        if self.security_config['require_special_chars']:
            if not any(c in '!@#$%^&*()_+-=[]{}|;:,.<>?' for c in password):
                return False

        # Credentials must be set via environment variables: AMR_USER_<USERNAME>=<hashed_password>
        env_key = f"AMR_USER_{username.upper()}"
        stored = os.getenv(env_key)
        if not stored:
            return False

        password_hash = hashlib.sha256(password.encode()).hexdigest()
        return hmac.compare_digest(stored, password_hash)

    def _is_ip_locked(self, client_ip: str) -> bool:
        with self.lock:
            data = self.failed_login_attempts.get(client_ip)
            if data:
                if (data['count'] >= self.security_config['max_login_attempts'] and
                        time.time() - data['last_attempt'] < self.security_config['lockout_duration']):
                    return True
        return False

    def _record_failed_attempt(self, client_ip: str):
        with self.lock:
            if client_ip in self.failed_login_attempts:
                self.failed_login_attempts[client_ip]['count'] += 1
                self.failed_login_attempts[client_ip]['last_attempt'] = time.time()
            else:
                self.failed_login_attempts[client_ip] = {'count': 1, 'last_attempt': time.time()}

    def _reset_failed_attempts(self, client_ip: str):
        with self.lock:
            self.failed_login_attempts.pop(client_ip, None)

    def _create_session(self, username: str, client_ip: str) -> str:
        session_id = secrets.token_urlsafe(32)
        with self.lock:
            self.active_sessions[session_id] = {
                'username': username,
                'client_ip': client_ip,
                'created_at': time.time(),
                'last_activity': time.time()
            }
        return session_id

    def validate_session(self, session_id: str) -> bool:
        with self.lock:
            session = self.active_sessions.get(session_id)
            if not session:
                return False
            if time.time() - session['last_activity'] > self.security_config['session_timeout']:
                del self.active_sessions[session_id]
                return False
            session['last_activity'] = time.time()
            return True

    def encrypt_data(self, data: str) -> str:
        if not self.security_config['encryption_enabled'] or not self.cipher:
            return data
        try:
            return base64.b64encode(self.cipher.encrypt(data.encode())).decode()
        except Exception as e:
            self.logger.error(f"Encryption failed: {e}")
            return data

    def decrypt_data(self, encrypted_data: str) -> str:
        if not self.security_config['encryption_enabled'] or not self.cipher:
            return encrypted_data
        try:
            return self.cipher.decrypt(base64.b64decode(encrypted_data.encode())).decode()
        except Exception as e:
            self.logger.error(f"Decryption failed: {e}")
            return encrypted_data

    def check_rate_limit(self, client_ip: str) -> bool:
        current_time = time.time()
        with self.lock:
            counter = self.rate_limit_counters.get(client_ip)
            if counter:
                if current_time - counter['window_start'] > self.security_config['rate_limit_window']:
                    self.rate_limit_counters[client_ip] = {'count': 1, 'window_start': current_time}
                else:
                    counter['count'] += 1
                    if counter['count'] > self.security_config['rate_limit_requests']:
                        return False
            else:
                self.rate_limit_counters[client_ip] = {'count': 1, 'window_start': current_time}
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
        }

    def get_security_alerts(self) -> List[Dict]:
        return self.security_alerts.copy()


_security_manager: Optional[SecurityManager] = None
_manager_lock = threading.Lock()


def get_security_manager() -> SecurityManager:
    global _security_manager
    if _security_manager is None:
        with _manager_lock:
            if _security_manager is None:
                _security_manager = SecurityManager()
    return _security_manager


if __name__ == "__main__":
    manager = get_security_manager()
    print(f"Security Status: {manager.get_security_status()}")
