import os
import time
import schedule
import logging
from datetime import datetime
from typing import List

from .backup_manager import BackupManager
from .backup_config import BackupConfig

class AutoBackup:
    def __init__(self):
        self.manager = BackupManager(BackupConfig.DEFAULT_BACKUP_DIR)
        self.logger = logging.getLogger("AutoBackup")
        self.setup_logging()
    
    def setup_logging(self):
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    
    def create_daily_backup(self):
        config = BackupConfig.get_schedule_config()["daily"]
        paths = BackupConfig.validate_paths(config["paths"])
        
        if paths:
            backup_name = f"daily_backup_{datetime.now().strftime('%Y%m%d')}"
            self.manager.create_backup(paths, backup_name)
            self.manager.cleanup_old_backups(config["keep_count"])
    
    def create_weekly_backup(self):
        config = BackupConfig.get_schedule_config()["weekly"]
        paths = BackupConfig.validate_paths(config["paths"])
        
        if paths:
            backup_name = f"weekly_backup_{datetime.now().strftime('%Y%m%d')}"
            self.manager.create_backup(paths, backup_name)
            self.manager.cleanup_old_backups(config["keep_count"])
    
    def create_monthly_backup(self):
        config = BackupConfig.get_schedule_config()["monthly"]
        paths = BackupConfig.validate_paths(config["paths"])
        
        if paths:
            backup_name = f"monthly_backup_{datetime.now().strftime('%Y%m')}"
            self.manager.create_backup(paths, backup_name)
            self.manager.cleanup_old_backups(config["keep_count"])
    
    def setup_schedule(self):
        schedule_config = BackupConfig.get_schedule_config()
        
        if schedule_config["daily"]["enabled"]:
            schedule.every().day.at(schedule_config["daily"]["time"]).do(self.create_daily_backup)
        
        if schedule_config["weekly"]["enabled"]:
            schedule.every().sunday.at(schedule_config["weekly"]["time"]).do(self.create_weekly_backup)
        
        if schedule_config["monthly"]["enabled"]:
            schedule.every().month.at(schedule_config["monthly"]["time"]).do(self.create_monthly_backup)
    
    def run(self):
        self.logger.info("Auto backup service started")
        self.setup_schedule()
        
        try:
            while True:
                schedule.run_pending()
                time.sleep(60)
        except KeyboardInterrupt:
            self.logger.info("Auto backup service stopped")
    
    def create_manual_backup(self, backup_type: str = "manual"):
        paths = BackupConfig.get_backup_paths()
        valid_paths = BackupConfig.validate_paths(paths)
        
        if valid_paths:
            backup_name = f"{backup_type}_backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            result = self.manager.create_backup(valid_paths, backup_name)
            
            if result:
                self.logger.info(f"Manual backup created: {result}")
                return result
            else:
                self.logger.error("Manual backup creation failed")
                return None
        else:
            self.logger.error("No valid paths found for backup")
            return None

def main():
    print("=== Auto Backup Service ===")
    
    auto_backup = AutoBackup()
    
    print("Creating initial backup...")
    auto_backup.create_manual_backup("initial")
    
    print("Starting auto backup service...")
    print("Press Ctrl+C to stop")
    
    auto_backup.run()

if __name__ == "__main__":
    main()
