#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GY-BN008x IMU-모터 통합 시스템 간단 테스트
이 스크립트는 C++ 프로그램을 실행하여 IMU-모터 통합 시스템을 테스트합니다.
"""

import subprocess
import time
import signal
import sys
import os

class IMUMotorTester:
    def __init__(self):
        self.process = None
        self.running = False
        
    def signal_handler(self, signum, frame):
        """시그널 핸들러 - Ctrl+C 처리"""
        print("\n종료 신호 수신...")
        self.stop()
        sys.exit(0)
        
    def start(self):
        """테스트 프로그램 시작"""
        try:
            # C++ 테스트 프로그램 실행
            self.process = subprocess.Popen(
                ["./imu_motor_test"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            self.running = True
            print("IMU-모터 통합 테스트 프로그램 시작됨")
            
        except FileNotFoundError:
            print("오류: imu_motor_test 실행 파일을 찾을 수 없습니다.")
            print("먼저 빌드를 완료해주세요:")
            print("  mkdir build && cd build")
            print("  cmake ..")
            print("  make")
            return False
        except Exception as e:
            print(f"오류: {e}")
            return False
            
        return True
        
    def stop(self):
        """테스트 프로그램 정지"""
        if self.process and self.running:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.running = False
            print("테스트 프로그램 정지됨")
            
    def monitor_output(self):
        """프로그램 출력 모니터링"""
        if not self.process:
            return
            
        print("=== 프로그램 출력 모니터링 ===")
        
        try:
            while self.running:
                # 실시간 출력 읽기
                output = self.process.stdout.readline()
                if output:
                    print(output.strip())
                    
                # 프로세스 상태 확인
                if self.process.poll() is not None:
                    print("프로그램이 종료되었습니다.")
                    break
                    
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n사용자에 의해 중단됨")
        except Exception as e:
            print(f"모니터링 오류: {e}")
        finally:
            self.stop()
            
    def run_simple_test(self):
        """간단한 테스트 실행"""
        print("=== GY-BN008x IMU-모터 통합 시스템 간단 테스트 ===")
        print("이 테스트는 C++ 프로그램을 실행하여 다음을 확인합니다:")
        print("1. IMU 센서 초기화")
        print("2. 모터 드라이버 초기화")
        print("3. 좌회전 90도 테스트")
        print("4. 우회전 90도 테스트")
        print("5. 연속 회전 테스트")
        print("================================================")
        
        # 시그널 핸들러 설정
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 테스트 시작
        if self.start():
            self.monitor_output()
        else:
            print("테스트 시작 실패")
            
def main():
    """메인 함수"""
    tester = IMUMotorTester()
    
    # 현재 디렉토리 확인
    if not os.path.exists("imu_motor_test"):
        print("imu_motor_test 실행 파일이 현재 디렉토리에 없습니다.")
        print("build 디렉토리에서 실행하거나 빌드를 먼저 완료해주세요.")
        return
        
    # 테스트 실행
    tester.run_simple_test()
    
if __name__ == "__main__":
    main()
