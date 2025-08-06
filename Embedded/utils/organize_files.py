#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import shutil
import sys

def create_directories():
    directories = [
        "python/ai_subscriber",
        "python/amr_sync",
        "python/backend",
        "python/motor_control",
        "python/test",
        "scripts",
        "docs"
    ]
    for directory in directories:
        os.makedirs(directory, exist_ok=True)


def move_files():
    file_moves = [
        ("ai_position_subscriber.py", "python/ai_subscriber/"),
        ("amr_real_data_sync.py", "python/amr_sync/"),
        ("backend_mqtt_subscriber.py", "python/backend/"),
        ("real_motor_controller.py", "python/motor_control/"),
        ("test_amr_with_ai_subscriber.py", "python/test/"),
        ("test_backup_system.py", "python/test/"),
        ("test_bidirectional_communication.py", "python/test/"),
        ("run_sensor_sync.sh", "scripts/"),
    ]
    for source, destination in file_moves:
        if os.path.exists(source):
            shutil.move(source, destination)
        else:
            pass


def create_readme():
    pass

def create_main_readme():
    pass

def main():
    create_directories()
    move_files()
    create_readme()
    create_main_readme()

if __name__ == "__main__":
    main() 