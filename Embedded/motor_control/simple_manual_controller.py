#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
간단한 키보드 수동 주행 컨트롤러
IMU와 서보모터 없이 기본적인 모터 제어만 구현하되, Backend MQTT와 AI 통신은 유지
"""

import sys
import os
import time
import threading
from datetime import datetime

# 프로젝트 루트 경로 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

# PCA9685 모듈 import
from PCA9685 import PCA9685

# MQTT 통신을 위한 import
from mqtt.sensor_data_transmitter import SensorDataTransmitter

class SimpleManualController:
    """
    간단한 키보드 수동 주행 컨트롤러 (Backend MQTT 및 AI 통신 포함)
    """
    
    # 모터 방향 정의
    FORWARD = 'forward'
    BACKWARD = 'backward'
    
    def __init__(self, i2c_address=0x40, i2c_bus=1, debug=True, 
                 api_url=None, backend_broker="192.168.100.141", backend_port=1883):
        """
        Args:
            i2c_address (int): I2C 주소 (기본값: 0x40)
            i2c_bus (int): I2C 버스 번호 (기본값: 1)
            debug (bool): 디버그 모드 (기본값: True)
            api_url (str): AI API URL (기본값: None)
            backend_broker (str): Backend MQTT 브로커 주소 (기본값: "192.168.100.141")
            backend_port (int): Backend MQTT 포트 (기본값: 1883)
        """
        self.debug = debug
        self.i2c_address = i2c_address
        self.i2c_bus = i2c_bus
        self.api_url = api_url
        self.backend_broker = backend_broker
        self.backend_port = backend_port
        
        # 모터 핀 정의
        self.PWMA = 0  # 모터 A PWM
        self.AIN1 = 1  # 모터 A 방향 1
        self.AIN2 = 2  # 모터 A 방향 2
        self.PWMB = 5  # 모터 B PWM
        self.BIN1 = 3  # 모터 B 방향 1
        self.BIN2 = 4  # 모터 B 방향 2
        
        # 모터 상태
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        self.motor_a_direction = self.FORWARD
        self.motor_b_direction = self.FORWARD
        
        # AI 통신 상태
        self.last_ai_data = None
        self.serial_number = "AMR001"  # 기본 시리얼 번호
        
        # Backend 통신
        self.backend_transmitter = None
        self.backend_connected = False
        
        # 키보드 제어 관련 변수
        self.keyboard_running = False
        self.keyboard_thread = None
        self.key_lock = threading.Lock()
        
        # 키보드 매핑 설정
        self.key_mapping = {
            'w': 'forward',      # 전진
            's': 'backward',     # 후진
            'a': 'left',         # 좌회전
            'd': 'right',        # 우회전
            'space': 'stop',     # 정지
            '1': 'speed_low',    # 저속
            '2': 'speed_medium', # 중속
            '3': 'speed_high',   # 고속
            '4': 'speed_max',    # 최고속
        }
        
        # 속도 설정
        self.speed_levels = {
            'speed_low': 20,     # 20%
            'speed_medium': 50,  # 50%
            'speed_high': 80,    # 80%
            'speed_max': 100,    # 100%
        }
        self.current_speed_level = 'speed_medium'  # 기본 중속
        
        try:
            # PCA9685 모터 드라이버 초기화
            self.pwm = PCA9685(i2c_address, debug=debug)
            self.pwm.setPWMFreq(20)  # 20Hz PWM 주파수
            
            # 모든 모터 정지
            self.stop_all()
            
            if self.debug:
                print(f"간단한 수동 제어 컨트롤러 초기화 완료")
                print(f"   모터 드라이버 주소: 0x{i2c_address:02X}")
                print(f"   I2C 버스: {self.i2c_bus}")
                print(f"   PWM 주파수: 20Hz")
                if api_url:
                    print(f"   AI API URL: {api_url}")
                print(f"   Backend MQTT: {backend_broker}:{backend_port}")
                self.print_controls()
                
        except Exception as e:
            print(f"간단한 수동 제어 컨트롤러 초기화 실패: {e}")
            raise

    def print_controls(self):
        """키보드 조작법 출력"""
        print("\n" + "="*50)
        print("키보드 수동 제어 조작법")
        print("="*50)
        print("이동 제어:")
        print("  W - 전진")
        print("  S - 후진")
        print("  A - 좌회전")
        print("  D - 우회전")
        print("  SPACE - 정지")
        print()
        print("속도 제어:")
        print("  1 - 저속 (20%)")
        print("  2 - 중속 (50%)")
        print("  3 - 고속 (80%)")
        print("  4 - 최고속 (100%)")
        print()
        print("기타:")
        print("  ESC 또는 Ctrl+C - 종료")
        print("="*50)

    def set_serial_number(self, serial):
        """시리얼 번호 설정"""
        self.serial_number = serial
        if self.debug:
            print(f"시리얼 번호 설정: {serial}")

    def connect_backend(self):
        """Backend MQTT 연결"""
        try:
            if self.debug:
                print(f"Backend 연결 시도: {self.backend_broker}:{self.backend_port}")
            
            self.backend_transmitter = SensorDataTransmitter(
                robot_id=self.serial_number,
                mqtt_broker=self.backend_broker,
                mqtt_port=self.backend_port
            )
            
            if self.backend_transmitter.connect_mqtt():
                self.backend_connected = True
                if self.debug:
                    print("Backend 연결 성공")
                    print(f"연결된 브로커: {self.backend_broker}:{self.backend_port}")
                    print(f"로봇 ID: {self.serial_number}")
                return True
            else:
                if self.debug:
                    print("Backend 연결 실패")
                return False
                
        except Exception as e:
            if self.debug:
                print(f"Backend 연결 오류: {e}")
            return False

    def disconnect_backend(self):
        """Backend MQTT 연결 해제"""
        if self.backend_transmitter:
            self.backend_transmitter.disconnect_mqtt()
            self.backend_connected = False
            if self.debug:
                print("Backend 연결 해제")

    def send_to_backend(self, motor_status=None):
        """Backend로 상태 데이터 전송"""
        if not self.backend_connected or not self.backend_transmitter:
            if self.debug:
                print("Backend가 연결되지 않았습니다.")
            return False
        
        try:
            if self.debug:
                print("Backend 데이터 전송 시작")
            
            state = "RUNNING"
            x = 0.0
            y = 0.0
            speed = self.speed_levels[self.current_speed_level]
            angle = 0.0  # IMU 없으므로 0도
            
            if motor_status:
                motor_a_speed = motor_status.get('motor_a', {}).get('speed', 0)
                motor_b_speed = motor_status.get('motor_b', {}).get('speed', 0)
                if motor_a_speed > 0 or motor_b_speed > 0:
                    speed = max(motor_a_speed, motor_b_speed)
            
            if self.debug:
                print(f"전송할 데이터: state={state}, x={x}, y={y}, speed={speed}, angle={angle}")
            
            self.backend_transmitter.update_embedded_data(
                serial=self.serial_number,
                state=state,
                x=x,
                y=y,
                speed=speed,
                angle=angle
            )
            
            if self.backend_transmitter.send_embedded_data():
                if self.debug:
                    print(f"Backend 전송 성공: {state}, 속도: {speed}%, 각도: {angle:.2f}도")
                return True
            else:
                if self.debug:
                    print("Backend 전송 실패")
                return False
                
        except Exception as e:
            if self.debug:
                print(f"Backend 전송 오류: {e}")
            return False

    def get_ai_data(self):
        """AI 서버에서 명령 데이터 가져오기"""
        if not self.api_url:
            if self.debug:
                print("API URL이 설정되지 않았습니다.")
            return None
            
        try:
            import requests
            response = requests.get(self.api_url, timeout=5)
            if response.status_code == 200:
                data = response.json()
                self.last_ai_data = data
                if self.debug:
                    print(f"AI 명령 수신: {data}")
                return data
            else:
                if self.debug:
                    print(f"AI 명령 API 요청 실패: {response.status_code}")
                return None
        except Exception as e:
            if self.debug:
                print(f"AI 명령 가져오기 오류: {e}")
            return None

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
                    
            elif action in ['speed_low', 'speed_medium', 'speed_high', 'speed_max']:
                self.current_speed_level = action
                speed = self.speed_levels[action]
                if self.debug:
                    print(f"속도 변경: {action} ({speed}%)")
                    
            # Backend로 상태 전송
            motor_status = self.get_motor_status()
            self.send_to_backend(motor_status)
                    
        except Exception as e:
            if self.debug:
                print(f"액션 실행 오류: {e}")

    def differential_drive(self, left_speed, right_speed):
        """
        차동 구동 (로봇 이동) - 모터 A와 B 동시 제어
        
        Args:
            left_speed (int): 왼쪽 모터 속도 (-100 ~ 100)
            right_speed (int): 오른쪽 모터 속도 (-100 ~ 100)
        """
        # 속도 제한
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # 모터 A (왼쪽 모터) 설정
        if left_speed > 0:
            # 모터 A 전진
            self.pwm.setDutycycle(self.PWMA, abs(left_speed))
            self.pwm.setLevel(self.AIN1, 0)
            self.pwm.setLevel(self.AIN2, 1)
            self.motor_a_speed = abs(left_speed)
            self.motor_a_direction = self.FORWARD
        elif left_speed < 0:
            # 모터 A 후진
            self.pwm.setDutycycle(self.PWMA, abs(left_speed))
            self.pwm.setLevel(self.AIN1, 1)
            self.pwm.setLevel(self.AIN2, 0)
            self.motor_a_speed = abs(left_speed)
            self.motor_a_direction = self.BACKWARD
        else:
            # 모터 A 정지
            self.pwm.setDutycycle(self.PWMA, 0)
            self.motor_a_speed = 0
        
        # 모터 B (오른쪽 모터) 설정 - 방향 반전
        if right_speed > 0:
            # 모터 B 전진 (방향 반전)
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 1)  # 반전: 0→1
            self.pwm.setLevel(self.BIN2, 0)  # 반전: 1→0
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.FORWARD
        elif right_speed < 0:
            # 모터 B 후진 (방향 반전)
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 0)  # 반전: 1→0
            self.pwm.setLevel(self.BIN2, 1)  # 반전: 0→1
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.BACKWARD
        else:
            # 모터 B 정지
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
        
        if self.debug:
            print(f"차동 구동: L={left_speed}, R={right_speed}")

    def stop_all(self):
        """모든 모터 정지"""
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        if self.debug:
            print("모든 모터 정지")

    def get_motor_status(self):
        """모터 상태 정보 반환"""
        return {
            'motor_a': {
                'speed': self.motor_a_speed,
                'direction': self.motor_a_direction
            },
            'motor_b': {
                'speed': self.motor_b_speed,
                'direction': self.motor_b_direction
            }
        }

    def get_current_status(self):
        """현재 상태 정보 반환"""
        status = {
            "motor_a_speed": self.motor_a_speed,
            "motor_b_speed": self.motor_b_speed,
            "motor_a_direction": self.motor_a_direction,
            "motor_b_direction": self.motor_b_direction,
            "current_speed_level": self.current_speed_level,
            "speed_percentage": self.speed_levels[self.current_speed_level],
            "keyboard_running": self.keyboard_running,
            "backend_connected": self.backend_connected,
            "timestamp": datetime.now().isoformat()
        }
        return status

    def run_manual_control(self):
        """수동 제어 메인 루프"""
        print("\n" + "="*50)
        print("간단한 키보드 수동 제어 모드 시작")
        print("="*50)
        
        try:
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
                          f"속도: {status['speed_percentage']}%, "
                          f"Backend: {'연결됨' if status['backend_connected'] else '연결안됨'}")
                    last_status_time = current_time
                
                time.sleep(1.0)
                
            except Exception as e:
                if self.debug:
                    print(f"상태 모니터링 오류: {e}")
                time.sleep(1.0)

def main():
    """메인 함수"""
    print("간단한 키보드 수동 제어 컨트롤러 (Backend MQTT 포함)")
    
    controller = None
    
    try:
        # 컨트롤러 초기화
        controller = SimpleManualController(
            debug=True,
            api_url="http://localhost:5001/command",  # AI API URL (선택사항)
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
            controller.disconnect_backend()
        print("정리 완료")

if __name__ == "__main__":
    main()
