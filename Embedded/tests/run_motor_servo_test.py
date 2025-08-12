#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터-서보 동시 구동 테스트 실행 스크립트
"""

import sys
import os
import subprocess

def install_requirements():
    """필요한 패키지 설치"""
    print("📦 필요한 패키지 설치 중...")
    try:
        subprocess.check_call([
            sys.executable, "-m", "pip", "install", "-r", 
            "requirements_motor_servo.txt"
        ])
        print("✅ 패키지 설치 완료")
    except subprocess.CalledProcessError as e:
        print(f"❌ 패키지 설치 실패: {e}")
        return False
    return True

def run_test():
    """테스트 실행"""
    print("🚀 모터-서보 동시 구동 테스트 실행")
    try:
        subprocess.check_call([
            sys.executable, "motor_servo_simultaneous_test.py"
        ])
        print("✅ 테스트 완료")
    except subprocess.CalledProcessError as e:
        print(f"❌ 테스트 실행 실패: {e}")
    except KeyboardInterrupt:
        print("\n⚠️ 테스트 중단됨")

def main():
    """메인 함수"""
    print("=" * 60)
    print("🔧 모터-서보 동시 구동 테스트")
    print("=" * 60)
    
    # 현재 디렉토리를 tests 폴더로 변경
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    
    # 패키지 설치
    if not install_requirements():
        print("패키지 설치에 실패했습니다. 수동으로 설치해주세요.")
        return
    
    # 테스트 실행
    run_test()

if __name__ == "__main__":
    main() 