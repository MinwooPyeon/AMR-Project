class AMRBaseException(Exception):
    
    def __init__(self, message: str, error_code: str = None, details: dict = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code
        self.details = details or {}
    
    def __str__(self):
        if self.error_code:
            return f"[{self.error_code}] {self.message}"
        return self.message


class AMRConnectionError(AMRBaseException):
    
    def __init__(self, service: str, message: str, error_code: str = None, details: dict = None):
        super().__init__(f"{service} connection error: {message}", error_code, details)
        self.service = service


class AMRConfigError(AMRBaseException):
    
    def __init__(self, config_name: str, message: str, error_code: str = None, details: dict = None):
        super().__init__(f"Configuration error in {config_name}: {message}", error_code, details)
        self.config_name = config_name


class AMRSecurityError(AMRBaseException):
    
    def __init__(self, security_type: str, message: str, error_code: str = None, details: dict = None):
        super().__init__(f"Security error ({security_type}): {message}", error_code, details)
        self.security_type = security_type


class AMRHardwareError(AMRBaseException):
    
    def __init__(self, device: str, message: str, error_code: str = None, details: dict = None):
        super().__init__(f"Hardware error in {device}: {message}", error_code, details)
        self.device = device


class AMRCommunicationError(AMRBaseException):
    
    def __init__(self, protocol: str, message: str, error_code: str = None, details: dict = None):
        super().__init__(f"Communication error ({protocol}): {message}", error_code, details)
        self.protocol = protocol


class AMRValidationError(AMRBaseException):
    
    def __init__(self, field: str, message: str, error_code: str = None, details: dict = None):
        super().__init__(f"Validation error in {field}: {message}", error_code, details)
        self.field = field


class AMRTimeoutError(AMRBaseException):
    
    def __init__(self, operation: str, timeout: float, error_code: str = None, details: dict = None):
        super().__init__(f"Timeout error in {operation} after {timeout}s", error_code, details)
        self.operation = operation
        self.timeout = timeout


class AMRResourceError(AMRBaseException):
    
    def __init__(self, resource: str, message: str, error_code: str = None, details: dict = None):
        super().__init__(f"Resource error in {resource}: {message}", error_code, details)
        self.resource = resource


class MQTTConnectionError(AMRConnectionError):
    
    def __init__(self, message: str, broker: str = None, port: int = None, details: dict = None):
        super().__init__("MQTT", message, "MQTT_CONNECTION_ERROR", details)
        self.broker = broker
        self.port = port


class MotorControlError(AMRHardwareError):
    
    def __init__(self, message: str, motor_id: str = None, details: dict = None):
        super().__init__("Motor", message, "MOTOR_CONTROL_ERROR", details)
        self.motor_id = motor_id


class SensorError(AMRHardwareError):
    
    def __init__(self, message: str, sensor_type: str = None, details: dict = None):
        super().__init__("Sensor", message, "SENSOR_ERROR", details)
        self.sensor_type = sensor_type


class AuthenticationError(AMRSecurityError):
    
    def __init__(self, message: str, username: str = None, details: dict = None):
        super().__init__("Authentication", message, "AUTH_ERROR", details)
        self.username = username


class AuthorizationError(AMRSecurityError):
    
    def __init__(self, message: str, user: str = None, action: str = None, details: dict = None):
        super().__init__("Authorization", message, "AUTHZ_ERROR", details)
        self.user = user
        self.action = action


class ConfigurationNotFoundError(AMRConfigError):
    
    def __init__(self, config_path: str, details: dict = None):
        super().__init__("File", f"Configuration file not found: {config_path}", "CONFIG_NOT_FOUND", details)
        self.config_path = config_path


class InvalidConfigurationError(AMRConfigError):
    
    def __init__(self, config_name: str, field: str, value: str, details: dict = None):
        super().__init__(config_name, f"Invalid value '{value}' for field '{field}'", "INVALID_CONFIG", details)
        self.field = field
        self.value = value


class DataValidationError(AMRValidationError):
    
    def __init__(self, field: str, expected_type: str, actual_value: str, details: dict = None):
        super().__init__(field, f"Expected {expected_type}, got {actual_value}", "DATA_VALIDATION_ERROR", details)
        self.expected_type = expected_type
        self.actual_value = actual_value


class NetworkTimeoutError(AMRTimeoutError):

    
    def __init__(self, operation: str, timeout: float, host: str = None, details: dict = None):
        super().__init__(operation, timeout, "NETWORK_TIMEOUT", details)
        self.host = host


class ResourceUnavailableError(AMRResourceError):

    
    def __init__(self, resource: str, reason: str = None, details: dict = None):
        super().__init__(resource, f"Resource unavailable: {reason or 'Unknown reason'}", "RESOURCE_UNAVAILABLE", details)
        self.reason = reason
