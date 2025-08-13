#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
키보드 입력으로 수동운전하는 컨트롤러
IMU AI 모터 컨트롤러를 상속받아 키보드 입력 처리 기능 추가
"""

import sys
import os
import time
import threading
from datetime import datetime

# 프로젝트 루트 경로 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

# 기존 IMU AI 모터 컨트롤러 import
from imu_ai_motor_controller import IMUAIMotorController

class ManualKeyboardController(IMUAIMotorController):
    """
    키보드 입력으로 수동운전하는 컨트롤러
    """
    
    def __init__(self, i2c_address=0x40, i2c_bus=1, debug=True, 
                 backend_broker="192.168.100.141", backend_port=1883):
        """
        Args:
            i2c_address (int): I2C 주소 (기본값: 0x40)
            i2c_bus (int): I2C 버스 번호 (기본값: 1)
            debug (bool): 디버그 모드 (기본값: True)
            backend_broker (str): Backend MQTT 브로커 주소 (기본값: "192.168.100.141")
            backend_port (int): Backend MQTT 포트 (기본값: 1883)
        """
        # 부모 클래스 초기화 (AI API URL은 None으로 설정)
        super().__init__(
            i2c_address=i2c_address,
            i2c_bus=i2c_bus,
            debug=debug,
            api_url=None,  # 수동 제어이므로 AI API 사용 안 함
            backend_broker=backend_broker,
            backend_port=backend_port
        )
        
        # 키보드 제어 관련 변수
        self.keyboard_running = False
        self.keyboard_thread = None
        self.current_keys = set()  # 현재 눌린 키들
        self.key_lock = threading.Lock()
        
        # 키보드 매핑 설정
        self.key_mapping = {
            'w': 'forward',      # 전진
            's': 'backward',     # 후진
            'a': 'left',         # 좌회전
            'd': 'right',        # 우회전
            'q': 'turn_left_90', # 90도 좌회전
            'e': 'turn_right_90', # 90도 우회전
            'space': 'stop',     # 정지
            'r': 'center',       # 정중앙 맞추기
            '1': 'speed_low',    # 저속
            '2': 'speed_medium', # 중속
            '3': 'speed_high',   # 고속
            '4': 'speed_max',    # 최고속
            'z': 'servo_up',     # 서보 UP
            'x': 'servo_down',   # 서보 DOWN
            'c': 'servo_center', # 서보 중앙
        }
        
        # 속도 설정
        self.speed_levels = {
            'speed_low': 20,     # 20%
            'speed_medium': 50,  # 50%
            'speed_high': 80,    # 80%
            'speed_max': 100,    # 100%
        }
        self.current_speed_level = 'speed_medium'  # 기본 중속
        
        if self.debug:
            print("키보드 수동 제어 컨트롤러 초기화 완료")
            self.print_controls()

    def print_controls(self):
        """키보드 조작법 출력"""
        print("\n" + "="*60)
        print("키보드 수동 제어 조작법")
        print("="*60)
        print("이동 제어:")
        print("  W - 전진")
        print("  S - 후진")
        print("  A - 좌회전")
        print("  D - 우회전")
        print("  SPACE - 정지")
        print()
        print("회전 제어:")
        print("  Q - 90도 좌회전")
        print("  E - 90도 우회전")
        print("  R - 정중앙 맞추기")
        print()
        print("속도 제어:")
        print("  1 - 저속 (20%)")
        print("  2 - 중속 (50%)")
        print("  3 - 고속 (80%)")
        print("  4 - 최고속 (100%)")
        print()
        print("서보 제어:")
        print("  Z - 서보 UP")
        print("  X - 서보 DOWN")
        print("  C - 서보 중앙")
        print()
        print("기타:")
        print("  ESC 또는 Ctrl+C - 종료")
        print("="*60)

    def start_keyboard_control(self):
        """키보드 제어 시작"""
        if self.keyboard_running:
            if self.debug:
                print("키보드 제어가 이미 실행 중입니다.")
            return
        
        self.keyboard_running = True
        self.keyboard_thread = threading.Thread(target=self._keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        if self.debug:
            print("키보드 제어 시작")
            print("키를 눌러 로봇을 조작하세요.")

    def stop_keyboard_control(self):
        """키보드 제어 정지"""
        self.keyboard_running = False
        if self.keyboard_thread:
            self.keyboard_thread.join(timeout=1.0)
        
        # 모든 모터 정지
        self.stop_all()
        
        if self.debug:
            print("키보드 제어 정지")

    def _keyboard_loop(self):
        """키보드 입력 처리 루프"""
        try:
            # Windows 환경에서 키보드 입력 처리
            if os.name == 'nt':  # Windows
                import msvcrt
                while self.keyboard_running:
                    if msvcrt.kbhit():
                        key = msvcrt.getch()
                        
                        # 특수 키 처리
                        if key == b'\x00':  # 확장 키
                            key = msvcrt.getch()
                            if key == b'H':  # 화살표 위
                                self._process_key('w')
                            elif key == b'P':  # 화살표 아래
                                self._process_key('s')
                            elif key == b'K':  # 화살표 왼쪽
                                self._process_key('a')
                            elif key == b'M':  # 화살표 오른쪽
                                self._process_key('d')
                        elif key == b' ':  # 스페이스바
                            self._process_key('space')
                        elif key == b'\x1b':  # ESC
                            self.keyboard_running = False
                            if self.debug:
                                print("ESC 키 감지 - 종료")
                        else:
                            # 일반 키 처리
                            try:
                                key_char = key.decode('utf-8').lower()
                                self._process_key(key_char)
                            except UnicodeDecodeError:
                                pass
                    time.sleep(0.01)
            else:  # Linux/Unix
                import tty
                import termios
                
                # 터미널 설정 저장
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                
                try:
                    # 터미널을 raw 모드로 설정
                    tty.setraw(sys.stdin.fileno())
                    
                    while self.keyboard_running:
                        if sys.stdin.readable():
                            key = sys.stdin.read(1).lower()
                            self._process_key(key)
                        time.sleep(0.01)
                finally:
                    # 터미널 설정 복원
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    
        except Exception as e:
            if self.debug:
                print(f"키보드 입력 처리 오류: {e}")

    def _process_key(self, key):
        """개별 키 처리"""
        with self.key_lock:
            if key in self.key_mapping:
                action = self.key_mapping[key]
                self._execute_action(action)
            elif key == '\x1b':  # ESC 키
                self.keyboard_running = False
                if self.debug:
                    print("ESC 키 감지 - 종료")

    def _execute_action(self, action):
        """액션 실행"""
        try:
            if action == 'forward':
                speed = self.speed_levels[self.current_speed_level]
                self.differential_drive(speed, speed)
                if self.debug:
                    print(f"전진: 속도 {speed}%")
                    
            elif action == 'backward':
                speed = self.speed_levels[self.current_speed_level]
                self.differential_drive(-speed, -speed)
                if self.debug:
                    print(f"후진: 속도 {speed}%")
                    
            elif action == 'left':
                speed = self.speed_levels[self.current_speed_level]
                self.differential_drive(-speed, speed)
                if self.debug:
                    print(f"좌회전: 속도 {speed}%")
                    
            elif action == 'right':
                speed = self.speed_levels[self.current_speed_level]
                self.differential_drive(speed, -speed)
                if self.debug:
                    print(f"우회전: 속도 {speed}%")
                    
            elif action == 'stop':
                self.stop_all()
                if self.debug:
                    print("정지")
                    
            elif action == 'turn_left_90':
                if not self.is_turning:
                    self.turn_left_90()
                    if self.debug:
                        print("90도 좌회전 시작")
                        
            elif action == 'turn_right_90':
                if not self.is_turning:
                    self.turn_right_90()
                    if self.debug:
                        print("90도 우회전 시작")
                        
            elif action == 'center':
                if not self.is_turning:
                    self.center_robot()
                    if self.debug:
                        print("정중앙 맞추기 시작")
                        
            elif action in ['speed_low', 'speed_medium', 'speed_high', 'speed_max']:
                self.current_speed_level = action
                speed = self.speed_levels[action]
                if self.debug:
                    print(f"속도 변경: {action} ({speed}%)")
                    
            elif action == 'servo_up':
                if self.servo_up():
                    if self.debug:
                        print("서보 UP")
                        
            elif action == 'servo_down':
                if self.servo_down():
                    if self.debug:
                        print("서보 DOWN")
                        
            elif action == 'servo_center':
                if hasattr(self, 'servo_pwm') and self.servo_pwm is not None:
                    self.set_servo_us(self.steering_servo_channel, self.servo_center_us)
                    if self.debug:
                        print("서보 중앙")
                        
        except Exception as e:
            if self.debug:
                print(f"액션 실행 오류: {e}")

    def get_current_status(self):
        """현재 상태 정보 반환"""
        status = {
            "motor_a_speed": self.motor_a_speed,
            "motor_b_speed": self.motor_b_speed,
            "motor_a_direction": self.motor_a_direction,
            "motor_b_direction": self.motor_b_direction,
            "current_angle": self.current_angle,
            "is_turning": self.is_turning,
            "current_speed_level": self.current_speed_level,
            "speed_percentage": self.speed_levels[self.current_speed_level],
            "servo_is_up": self.servo_is_up,
            "keyboard_running": self.keyboard_running,
            "timestamp": datetime.now().isoformat()
        }
        return status

    def run_manual_control(self):
        """수동 제어 메인 루프"""
        print("\n" + "="*60)
        print("키보드 수동 제어 모드 시작")
        print("="*60)
        
        try:
            # IMU 제어 루프 시작
            self.start_control_loop()
            
            # 키보드 제어 시작
            self.start_keyboard_control()
            
            # 상태 출력 스레드 시작
            status_thread = threading.Thread(target=self._status_monitor)
            status_thread.daemon = True
            status_thread.start()
            
            # 메인 루프
            while self.keyboard_running:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n사용자에 의해 중단")
        except Exception as e:
            print(f"\n오류 발생: {e}")
        finally:
            self.stop_keyboard_control()
            self.stop_control_loop()
            self.disconnect_backend()
            print("수동 제어 종료")

    def _status_monitor(self):
        """상태 모니터링 스레드"""
        last_status_time = time.time()
        
        while self.keyboard_running:
            try:
                current_time = time.time()
                
                # 5초마다 상태 출력
                if current_time - last_status_time >= 5.0:
                    status = self.get_current_status()
                    print(f"\n[상태] 모터A: {status['motor_a_speed']}%({status['motor_a_direction']}), "
                          f"모터B: {status['motor_b_speed']}%({status['motor_b_direction']}), "
                          f"각도: {status['current_angle']:.1f}°, "
                          f"속도: {status['speed_percentage']}%")
                    last_status_time = current_time
                
                time.sleep(1.0)
                
            except Exception as e:
                if self.debug:
                    print(f"상태 모니터링 오류: {e}")
                time.sleep(1.0)

def main():
    """메인 함수"""
    print("키보드 수동 제어 컨트롤러")
    
    controller = None
    
    try:
        # 컨트롤러 초기화
        controller = ManualKeyboardController(
            debug=True,
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        # 시리얼 번호 설정
        controller.set_serial_number("AMR001")
        
        # Backend 연결
        if controller.connect_backend():
            print("Backend 연결 성공")
        else:
            print("Backend 연결 실패 (계속 진행)")
        
        # 수동 제어 시작
        controller.run_manual_control()
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단")
    except Exception as e:
        print(f"\n오류 발생: {e}")
    finally:
        if controller:
            controller.stop_all()
            controller.stop_control_loop()
            controller.disconnect_backend()
        print("정리 완료")

if __name__ == "__main__":
    main()
