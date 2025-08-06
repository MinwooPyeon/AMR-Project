#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AMR 메인 실행 파일
Linux 환경에서 실행하기 위한 스크립트
"""

import sys
import os

# 현재 디렉토리를 Python 경로에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# 환경 변수 설정
os.environ['PYTHONPATH'] = current_dir + ':' + os.environ.get('PYTHONPATH', '')

try:
    from main.amr_real_data_sync import AMRRealDataSync
    print("✅ AMR 시스템 모듈 import 성공")
    
    # AMR 시스템 실행
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True, enable_backup=True)
    print("✅ AMR 시스템 초기화 완료")
    
    # 모터 상태 확인
    print("\n🔧 모터 상태 확인:")
    motor_speeds = amr_sync.get_motor_speeds()
    print(f"  - 왼쪽 모터 속도: {motor_speeds.get('left_speed', 0)}%")
    print(f"  - 오른쪽 모터 속도: {motor_speeds.get('right_speed', 0)}%")
    print(f"  - 모터 동작 중: {'예' if motor_speeds.get('is_running', False) else '아니오'}")
    
    print("\n✅ AMR 시스템 실행 완료")
    
except ImportError as e:
    print(f"❌ 모듈 import 오류: {e}")
    print("💡 해결 방법:")
    print("   1. 현재 디렉토리 확인: pwd")
    print("   2. Python 경로 설정: export PYTHONPATH=\".\"")
    print("   3. 다시 실행: python3 run_amr.py")
    
except Exception as e:
    print(f"❌ 실행 오류: {e}")
    print("💡 디버깅 정보:")
    print(f"   - 현재 디렉토리: {os.getcwd()}")
    print(f"   - Python 경로: {sys.path}")
    print(f"   - 파일 경로: {os.path.abspath(__file__)}")

if __name__ == "__main__":
    print("🚀 AMR 시스템 시작...") 