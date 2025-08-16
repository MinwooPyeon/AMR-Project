import os
import shutil
import json
import zipfile
from datetime import datetime
from typing import List, Dict, Optional
import logging

class BackupManager:
    def __init__(self, backup_dir: str = "backup"):
        self.backup_dir = backup_dir
        self.logger = logging.getLogger("BackupManager")
        
        if not os.path.exists(self.backup_dir):
            os.makedirs(self.backup_dir)
    
    def create_backup(self, source_paths: List[str], backup_name: str = None) -> str:
        if not backup_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_name = f"backup_{timestamp}"
        
        backup_path = os.path.join(self.backup_dir, backup_name)
        
        try:
            if len(source_paths) == 1 and os.path.isfile(source_paths[0]):
                self._backup_single_file(source_paths[0], backup_path)
            else:
                self._backup_multiple_items(source_paths, backup_path)
            
            self.logger.info(f"Backup created: {backup_path}")
            return backup_path
            
        except Exception as e:
            self.logger.error(f"Backup creation failed: {e}")
            return None
    
    def _backup_single_file(self, source_path: str, backup_path: str):
        shutil.copy2(source_path, backup_path)
    
    def _backup_multiple_items(self, source_paths: List[str], backup_path: str):
        os.makedirs(backup_path, exist_ok=True)
        
        for source_path in source_paths:
            if os.path.exists(source_path):
                if os.path.isfile(source_path):
                    shutil.copy2(source_path, backup_path)
                elif os.path.isdir(source_path):
                    dir_name = os.path.basename(source_path)
                    dest_path = os.path.join(backup_path, dir_name)
                    shutil.copytree(source_path, dest_path, dirs_exist_ok=True)
    
    def create_zip_backup(self, source_paths: List[str], backup_name: str = None) -> str:
        if not backup_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_name = f"backup_{timestamp}.zip"
        
        backup_path = os.path.join(self.backup_dir, backup_name)
        
        try:
            with zipfile.ZipFile(backup_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                for source_path in source_paths:
                    if os.path.exists(source_path):
                        if os.path.isfile(source_path):
                            zipf.write(source_path, os.path.basename(source_path))
                        elif os.path.isdir(source_path):
                            for root, dirs, files in os.walk(source_path):
                                for file in files:
                                    file_path = os.path.join(root, file)
                                    arc_name = os.path.relpath(file_path, source_path)
                                    zipf.write(file_path, arc_name)
            
            self.logger.info(f"ZIP backup created: {backup_path}")
            return backup_path
            
        except Exception as e:
            self.logger.error(f"ZIP backup creation failed: {e}")
            return None
    
    def list_backups(self) -> List[Dict]:
        backups = []
        
        try:
            for item in os.listdir(self.backup_dir):
                item_path = os.path.join(self.backup_dir, item)
                if os.path.exists(item_path):
                    stat = os.stat(item_path)
                    backups.append({
                        "name": item,
                        "path": item_path,
                        "size": stat.st_size,
                        "modified": datetime.fromtimestamp(stat.st_mtime).isoformat(),
                        "is_dir": os.path.isdir(item_path)
                    })
            
            return sorted(backups, key=lambda x: x["modified"], reverse=True)
            
        except Exception as e:
            self.logger.error(f"Failed to list backups: {e}")
            return []
    
    def restore_backup(self, backup_name: str, restore_path: str) -> bool:
        backup_path = os.path.join(self.backup_dir, backup_name)
        
        if not os.path.exists(backup_path):
            self.logger.error(f"Backup not found: {backup_name}")
            return False
        
        try:
            if os.path.isfile(backup_path) and backup_path.endswith('.zip'):
                self._restore_zip_backup(backup_path, restore_path)
            elif os.path.isdir(backup_path):
                shutil.copytree(backup_path, restore_path, dirs_exist_ok=True)
            else:
                shutil.copy2(backup_path, restore_path)
            
            self.logger.info(f"Backup restored: {backup_name} -> {restore_path}")
            return True
            
        except Exception as e:
            self.logger.error(f"Backup restoration failed: {e}")
            return False
    
    def _restore_zip_backup(self, zip_path: str, restore_path: str):
        os.makedirs(restore_path, exist_ok=True)
        
        with zipfile.ZipFile(zip_path, 'r') as zipf:
            zipf.extractall(restore_path)
    
    def delete_backup(self, backup_name: str) -> bool:
        backup_path = os.path.join(self.backup_dir, backup_name)
        
        if not os.path.exists(backup_path):
            self.logger.error(f"Backup not found: {backup_name}")
            return False
        
        try:
            if os.path.isdir(backup_path):
                shutil.rmtree(backup_path)
            else:
                os.remove(backup_path)
            
            self.logger.info(f"Backup deleted: {backup_name}")
            return True
            
        except Exception as e:
            self.logger.error(f"Backup deletion failed: {e}")
            return False
    
    def cleanup_old_backups(self, keep_count: int = 5) -> int:
        backups = self.list_backups()
        
        if len(backups) <= keep_count:
            return 0
        
        deleted_count = 0
        for backup in backups[keep_count:]:
            if self.delete_backup(backup["name"]):
                deleted_count += 1
        
        return deleted_count

def main():
    print("=== Backup Manager Test ===")
    
    manager = BackupManager()
    
    source_paths = [
        "../ai",
        "../config",
        "../motor_control"
    ]
    
    print("Creating backup...")
    backup_path = manager.create_backup(source_paths, "test_backup")
    
    if backup_path:
        print(f"Backup created: {backup_path}")
        
        print("\nListing backups:")
        backups = manager.list_backups()
        for backup in backups:
            print(f"- {backup['name']} ({backup['size']} bytes, {backup['modified']})")
    
    print("\nBackup manager test completed")

if __name__ == "__main__":
    main()
