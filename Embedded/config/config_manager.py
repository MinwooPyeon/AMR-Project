import yaml
import os
import logging
from typing import Dict, Any, Optional
from pathlib import Path

class ConfigManager:
    def __init__(self, config_dir: str = "config"):
        self.config_dir = Path(config_dir)
        self.logger = logging.getLogger("ConfigManager")
        self.configs = {}
        self.load_all_configs()
    
    def load_all_configs(self):
        config_files = [
            "camera_config.yaml",
            "nav2_params.yaml", 
            "slam_params.yaml",
            "motor_config.yaml",
            "imu_config.yaml",
            "mqtt_config.yaml",
            "robot_config.yaml",
            "ai_config.yaml"
        ]
        
        for config_file in config_files:
            self.load_config(config_file)
    
    def load_config(self, filename: str) -> Optional[Dict[str, Any]]:
        file_path = self.config_dir / filename
        
        if not file_path.exists():
            self.logger.warning(f"Config file not found: {filename}")
            return None
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                self.configs[filename] = config
                self.logger.info(f"Loaded config: {filename}")
                return config
        except Exception as e:
            self.logger.error(f"Failed to load config {filename}: {e}")
            return None
    
    def save_config(self, filename: str, config: Dict[str, Any]) -> bool:
        file_path = self.config_dir / filename
        
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
            self.configs[filename] = config
            self.logger.info(f"Saved config: {filename}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to save config {filename}: {e}")
            return False
    
    def get_config(self, filename: str) -> Optional[Dict[str, Any]]:
        return self.configs.get(filename)
    
    def get_parameter(self, filename: str, parameter_path: str) -> Any:
        config = self.get_config(filename)
        if not config:
            return None
        
        keys = parameter_path.split('.')
        value = config
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return None
        
        return value
    
    def set_parameter(self, filename: str, parameter_path: str, value: Any) -> bool:
        config = self.get_config(filename)
        if not config:
            return False
        
        keys = parameter_path.split('.')
        current = config
        
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        
        current[keys[-1]] = value
        return self.save_config(filename, config)
    
    def get_robot_config(self) -> Dict[str, Any]:
        return self.get_config("robot_config.yaml") or {}
    
    def get_motor_config(self) -> Dict[str, Any]:
        return self.get_config("motor_config.yaml") or {}
    
    def get_imu_config(self) -> Dict[str, Any]:
        return self.get_config("imu_config.yaml") or {}
    
    def get_camera_config(self) -> Dict[str, Any]:
        return self.get_config("camera_config.yaml") or {}
    
    def get_mqtt_config(self) -> Dict[str, Any]:
        return self.get_config("mqtt_config.yaml") or {}
    
    def get_ai_config(self) -> Dict[str, Any]:
        return self.get_config("ai_config.yaml") or {}
    
    def get_nav2_config(self) -> Dict[str, Any]:
        return self.get_config("nav2_params.yaml") or {}
    
    def get_slam_config(self) -> Dict[str, Any]:
        return self.get_config("slam_params.yaml") or {}
    
    def validate_config(self, filename: str) -> bool:
        config = self.get_config(filename)
        if not config:
            return False
        
        required_fields = {
            "robot_config.yaml": ["robot"],
            "motor_config.yaml": ["motor"],
            "imu_config.yaml": ["imu"],
            "camera_config.yaml": ["camera"],
            "mqtt_config.yaml": ["mqtt"],
            "ai_config.yaml": ["ai"]
        }
        
        if filename in required_fields:
            for field in required_fields[filename]:
                if field not in config:
                    self.logger.error(f"Required field missing in {filename}: {field}")
                    return False
        
        return True
    
    def list_configs(self) -> list:
        return list(self.configs.keys())
    
    def reload_configs(self):
        self.configs.clear()
        self.load_all_configs()

def main():
    print("=== Config Manager Test ===")
    
    manager = ConfigManager()
    
    print("Loaded configs:")
    for config_name in manager.list_configs():
        print(f"- {config_name}")
    
    robot_name = manager.get_parameter("robot_config.yaml", "robot.ros__parameters.robot_name")
    print(f"Robot name: {robot_name}")
    
    print("Config manager test completed")

if __name__ == "__main__":
    main()
