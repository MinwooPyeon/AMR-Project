import os
from typing import List, Dict, Any

class BackupConfig:
    DEFAULT_BACKUP_DIR = "backup"
    DEFAULT_KEEP_COUNT = 5
    
    BACKUP_PATHS = [
        "ai",
        "config", 
        "motor_control",
        "mqtt",
        "ros2",
        "sensor_sync",
        "src",
        "include",
        "tests",
        "utils"
    ]
    
    EXCLUDE_PATTERNS = [
        "__pycache__",
        "*.pyc",
        "*.pyo",
        "*.pyd",
        ".git",
        ".vscode",
        "node_modules",
        "*.log",
        "*.tmp",
        "*.temp"
    ]
    
    BACKUP_SCHEDULE = {
        "daily": {
            "enabled": True,
            "time": "02:00",
            "paths": ["ai", "config", "motor_control"],
            "keep_count": 7
        },
        "weekly": {
            "enabled": True,
            "day": "sunday",
            "time": "03:00", 
            "paths": BACKUP_PATHS,
            "keep_count": 4
        },
        "monthly": {
            "enabled": True,
            "day": 1,
            "time": "04:00",
            "paths": BACKUP_PATHS,
            "keep_count": 12
        }
    }
    
    @classmethod
    def get_backup_paths(cls) -> List[str]:
        return cls.BACKUP_PATHS.copy()
    
    @classmethod
    def get_exclude_patterns(cls) -> List[str]:
        return cls.EXCLUDE_PATTERNS.copy()
    
    @classmethod
    def get_schedule_config(cls) -> Dict[str, Any]:
        return cls.BACKUP_SCHEDULE.copy()
    
    @classmethod
    def validate_paths(cls, paths: List[str]) -> List[str]:
        valid_paths = []
        for path in paths:
            if os.path.exists(path):
                valid_paths.append(path)
        return valid_paths
