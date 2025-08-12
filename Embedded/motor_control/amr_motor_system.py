#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AMR 모터 시스템 메인 실행 스크립트
PCA9685를 이용한 모터 및 서보모터 제어 + 백엔드 통신
"""

import time
import signal
import sys
import threading
from motor_control.pca9685_motor_controller import PCA9685MotorController

class AMRMotorSystem:
    """AMR 모터 시스템 메인 클래스"""
    
    def __init__(self):
        self.controller = None
        self.running = False
        self.status_thread = None
        
        # 시그널 핸들러 설정
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, signum, frame):
        """시그널 핸들러"""
        print(f"\n⚠️ 시그널 {signum} 수신. 시스템을 종료합니다...")
        self.cleanup()
        sys.exit(0)
    
    def initialize(self):
        """시스템 초기화"""
        print("=== AMR 모터 시스템 초기화 ===")
        
        try:
            # PCA9685 모터 컨트롤러 초기화
            self.controller = PCA9685MotorController(i2c_bus=7, debug=False)
            
            if not self.controller.pca9685:
                print("❌ PCA9685 초기화 실패")
                return False
            
            print("✅ PCA9685 모터 컨트롤러 초기화 완료")
            
            # 상태 모니터링 스레드 시작
            self.running = True
            self.status_thread = threading.Thread(target=self.status_monitor)
            self.status_thread.daemon = True
            self.status_thread.start()
            
            print("✅ 상태 모니터링 스레드 시작")
            print("✅ AMR 모터 시스템 초기화 완료")
            
            return True
            
        except Exception as e:
            print(f"❌ 시스템 초기화 실패: {e}")
            return False
    
    def status_monitor(self):
        """상태 모니터링 스레드"""
        while self.running:
            try:
                # 상태 전송
                if self.controller:
                    self.controller.send_status()
                
                # 5초마다 상태 전송
                time.sleep(5)
                
            except Exception as e:
                print(f"❌ 상태 모니터링 오류: {e}")
                time.sleep(1)
    
    def show_menu(self):
        """메뉴 표시"""
        print("\n" + "="*50)
        print("AMR 모터 시스템 제어 메뉴")
        print("="*50)
        print("1. 모터 제어")
        print("2. 리프트 제어")
        print("3. 상태 확인")
        print("4. 테스트 모드")
        print("0. 종료")
        print("="*50)
    
    def motor_control_menu(self):
        """모터 제어 메뉴"""
        while True:
            print("\n--- 모터 제어 ---")
            print("1. 전진")
            print("2. 후진")
            print("3. 좌회전")
            print("4. 우회전")
            print("5. 정지")
            print("6. 좌측 모터 전진")
            print("7. 우측 모터 전진")
            print("0. 이전 메뉴")
            
            choice = input("선택: ").strip()
            
            if choice == '1':
                speed = int(input("속도 (0-100): ") or "50")
                self.controller.move_forward(speed)
            elif choice == '2':
                speed = int(input("속도 (0-100): ") or "50")
                self.controller.move_backward(speed)
            elif choice == '3':
                speed = int(input("속도 (0-100): ") or "50")
                self.controller.turn_left(speed)
            elif choice == '4':
                speed = int(input("속도 (0-100): ") or "50")
                self.controller.turn_right(speed)
            elif choice == '5':
                self.controller.stop_all_motors()
            elif choice == '6':
                speed = int(input("속도 (0-100): ") or "50")
                self.controller.set_left_motor_forward(speed)
            elif choice == '7':
                speed = int(input("속도 (0-100): ") or "50")
                self.controller.set_right_motor_forward(speed)
            elif choice == '0':
                break
    

    
    def lift_control_menu(self):
        """리프트 제어 메뉴"""
        while True:
            print("\n--- 리프트 제어 ---")
            print("1. 리프트 상승")
            print("2. 리프트 하강")
            print("3. 리프트 중간 위치")
            print("0. 이전 메뉴")
            
            choice = input("선택: ").strip()
            
            if choice == '1':
                self.controller.lift_up()
            elif choice == '2':
                self.controller.lift_down()
            elif choice == '3':
                self.controller.lift_middle()
            elif choice == '0':
                break
    
    def show_status(self):
        """상태 확인"""
        if self.controller:
            status = self.controller.get_status()
            print("\n=== AMR 모터 시스템 상태 ===")
            print(f"PCA9685 사용 가능: {'✅' if status['pca9685_available'] else '❌'}")
            
            print("\n모터 상태:")
            for motor, motor_status in status['motor_status'].items():
                direction = motor_status['direction']
                speed = motor_status['speed']
                print(f"  {motor}: {direction} {speed}%")
            

        else:
            print("❌ 컨트롤러가 초기화되지 않았습니다")
    
    def test_mode(self):
        """테스트 모드"""
        print("\n=== 테스트 모드 시작 ===")
        
        try:
            # 모터 테스트
            print("1. 모터 테스트...")
            self.controller.move_forward(30)
            time.sleep(2)
            self.controller.stop_all_motors()
            time.sleep(1)
            
            self.controller.move_backward(30)
            time.sleep(2)
            self.controller.stop_all_motors()
            time.sleep(1)
            

            
            # 리프트 테스트
            print("2. 리프트 테스트...")
            self.controller.lift_up()
            time.sleep(2)
            self.controller.lift_middle()
            time.sleep(2)
            self.controller.lift_down()
            time.sleep(2)
            
            print("✅ 테스트 모드 완료")
            
        except Exception as e:
            print(f"❌ 테스트 모드 오류: {e}")
    
    def run(self):
        """메인 실행 루프"""
        if not self.initialize():
            print("❌ 시스템 초기화 실패")
            return
        
        print("\n🚀 AMR 모터 시스템이 시작되었습니다!")
        print("💡 백엔드에서 MQTT를 통해 명령을 보낼 수 있습니다.")
        print("💡 Ctrl+C로 종료할 수 있습니다.")
        
        while True:
            try:
                self.show_menu()
                choice = input("선택: ").strip()
                
                if choice == '1':
                    self.motor_control_menu()
                elif choice == '2':
                    self.lift_control_menu()
                elif choice == '3':
                    self.show_status()
                elif choice == '4':
                    self.test_mode()
                elif choice == '0':
                    break
                else:
                    print("❌ 잘못된 선택입니다.")
                    
            except KeyboardInterrupt:
                print("\n⚠️ 사용자에 의해 중단됨")
                break
            except Exception as e:
                print(f"❌ 오류 발생: {e}")
    
    def cleanup(self):
        """정리"""
        print("\n🧹 AMR 모터 시스템 정리 중...")
        
        self.running = False
        
        if self.controller:
            self.controller.cleanup()
        
        print("✅ AMR 모터 시스템 정리 완료")

def main():
    """메인 함수"""
    amr_system = AMRMotorSystem()
    
    try:
        amr_system.run()
    except Exception as e:
        print(f"❌ 시스템 실행 오류: {e}")
    finally:
        amr_system.cleanup()

if __name__ == "__main__":
    main() 