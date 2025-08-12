"""
AI 통신을 위한 모터 제어 클래스
Motor Driver HAT을 사용한 DC 모터 제어 + AI 통신 기능 + Backend 통신
"""

import time
import math
import smbus
import json
import base64
import requests
import sys
import os
from datetime import datetime
from PCA9685 import PCA9685
<<<<<<< HEAD

# 프로젝트 루트를 Python 경로에 추가
=======
>>>>>>> c63d83b (merge: Resolve conflicts)
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
sys.path.insert(0, project_root)

from mqtt.sensor_data_transmitter import SensorDataTransmitter

class AIMotorController:
    
    # 모터 방향 정의
    FORWARD = 'forward'
    BACKWARD = 'backward'
    
    def __init__(self, i2c_address=0x40, i2c_bus=7, debug=True, api_url=None, 
                 backend_broker="192.168.100.141", backend_port=1883):
        """
        Args:
            i2c_address (int): I2C 주소 (기본값: 0x40)
            i2c_bus (int): I2C 버스 번호 (기본값: 7)
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
        
        # 모터 속도 설정
        self.motor_speeds = {
<<<<<<< HEAD
            'forward': 50,    # 전진 속도
            'backward': 50,   # 후진 속도
            'left': 50,       # 좌회전 속도
            'right': 50,      # 우회전 속도
            'stop': 0,        # 정지 속도
            'custom': 50      # 커스텀 기본 속도
=======
            'forward': 50,    # 전진 속도 (50%)
            'backward': 50,   # 후진 속도 (50%)
            'left': 50,       # 좌회전 속도 (50%)
            'right': 50,      # 우회전 속도 (50%)
            'stop': 0,        # 정지 속도
            'custom': 50      # 커스텀 기본 속도 (50%)
>>>>>>> c63d83b (merge: Resolve conflicts)
        }
        
        try:
            # PCA9685 초기화
            self.pwm = PCA9685(i2c_address, debug=debug)
<<<<<<< HEAD
            self.pwm.setPWMFreq(50) 
=======
            self.pwm.setPWMFreq(20)  # 20Hz PWM 주파수 (더 강한 토크)
>>>>>>> c63d83b (merge: Resolve conflicts)
            
            # 모든 모터 정지
            self.stop_all()
            
            if self.debug:
                print(f"   I2C 주소: 0x{i2c_address:02X}")
                print(f"   I2C 버스: {i2c_bus}")
<<<<<<< HEAD
                print(f"   PWM 주파수: 50Hz")
=======
                print(f"   PWM 주파수: 20Hz")
>>>>>>> c63d83b (merge: Resolve conflicts)
                if api_url:
                    print(f"   AI API URL: {api_url}")
                print(f"   Backend MQTT: {backend_broker}:{backend_port}")
                print(f"   하드코딩된 모터 속도: {self.motor_speeds}")
                
        except Exception as e:
            print(f"AI 모터 컨트롤러 초기화 실패: {e}")
            raise

    def set_serial_number(self, serial):
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

    def send_to_backend(self, ai_data=None, motor_status=None):
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
<<<<<<< HEAD
            speed = 40.0
=======
            speed = 40.0  # 기본 속도 40%
>>>>>>> c63d83b (merge: Resolve conflicts)
            angle = 0.0
            
            if ai_data:
                x = float(ai_data.get('x', 0))
                y = float(ai_data.get('y', 0))
                
                motor_case = self.determine_case_from_coordinates(x, y)
                
                if motor_case in ['forward', '전진']:
                    speed = self.get_motor_speed('forward')
                elif motor_case in ['backward', '후진']:
                    speed = self.get_motor_speed('backward')
                elif motor_case in ['left', '좌회전']:
                    speed = self.get_motor_speed('left')
                elif motor_case in ['right', '우회전']:
                    speed = self.get_motor_speed('right')
                elif motor_case in ['stop', '정지']:
                    speed = self.get_motor_speed('stop')
                elif motor_case in ['custom', '커스텀']:
                    speed = self.get_motor_speed('custom')
            
            if motor_status:
                motor_a_speed = motor_status.get('motor_a', {}).get('speed', 0)
                motor_b_speed = motor_status.get('motor_b', {}).get('speed', 0)
<<<<<<< HEAD
                speed = max(motor_a_speed, motor_b_speed)
=======
                # 모터 상태에서 속도가 0보다 크면 그 값을 사용, 아니면 계산된 speed 유지
                if motor_a_speed > 0 or motor_b_speed > 0:
                    speed = max(motor_a_speed, motor_b_speed)
>>>>>>> c63d83b (merge: Resolve conflicts)
            
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
                    print(f"Backend 전송 성공: {state}, 속도: {speed}%")
                    print(f"토픽: status")
                    print(f"데이터: {self.backend_transmitter.get_embedded_data()}")
                return True
            else:
                if self.debug:
                    print("Backend 전송 실패")
                    print(f"연결 상태: {self.backend_transmitter.connected}")
                    print(f"브로커: {self.backend_broker}:{self.backend_port}")
                return False
                
        except Exception as e:
            if self.debug:
                print(f"Backend 전송 오류: {e}")
            return False

    def get_ai_data(self):
        if not self.api_url:
            if self.debug:
                print("API URL이 설정되지 않았습니다.")
            return None
            
        try:
            response = requests.get(self.api_url, timeout=5)
            if response.status_code == 200:
                data = response.json()
                self.last_ai_data = data
                if self.debug:
                    print(f"AI 데이터 수신: {data}")
                return data
            else:
                if self.debug:
                    print(f"API 요청 실패: {response.status_code}")
                return None
        except Exception as e:
            if self.debug:
                print(f"AI 데이터 가져오기 오류: {e}")
            return None

    def process_ai_command(self, ai_data):
        if not ai_data:
            return
            
        try:
            serial = ai_data.get('serial', '')
            if serial and serial != self.serial_number:
                if self.debug:
                    print(f"시리얼 번호 불일치: {serial} != {self.serial_number}")
                return
                
            x = float(ai_data.get('x', 0))
            y = float(ai_data.get('y', 0))
            case = ai_data.get('case', '')
            timestamp = ai_data.get('timeStamp', '')
            
            motor_case = self.determine_case_from_coordinates(x, y)
            
            if self.debug:
                print(f"AI 명령 처리:")
                print(f"  좌표: ({x}, {y})")
                print(f"  상황: {case}")
                print(f"  모터 제어: {motor_case}")
                print(f"  타임스탬프: {timestamp}")
            
            self.execute_ai_case(motor_case, x, y)
            
            motor_status = self.get_motor_status()
            self.send_to_backend(ai_data, motor_status)
            
        except Exception as e:
            if self.debug:
                print(f"AI 명령 처리 오류: {e}")

    def determine_case_from_coordinates(self, x, y):
        abs_x = abs(x)
        abs_y = abs(y)
        
        threshold = 0.3
        
        if abs_x <= threshold and abs_y <= threshold:
            return 'stop'
        
        if abs_y > abs_x and abs_y > threshold:
            if y > 0:
                return 'forward'
            else:
                return 'backward'
        
        if abs_x >= abs_y and abs_x > threshold:
            if x > 0:
                return 'right'
            else:
                return 'left'
        
        return 'custom'

    def execute_ai_case(self, case, x, y):
        case = case.lower()
        
        speed = self.get_motor_speed(case)
        
        if case == 'forward' or case == '전진':
            # 전진: 양쪽 모터 동일 속도
            self.differential_drive(speed, speed)
            if self.debug:
                print(f"전진 명령 실행: 속도 {speed}%")
                
        elif case == 'backward' or case == '후진':
            # 후진: 양쪽 모터 동일 속도
            self.differential_drive(-speed, -speed)
            if self.debug:
                print(f"후진 명령 실행: 속도 {speed}%")
                
<<<<<<< HEAD
        elif case == 'left' or case == '좌회전':
            # 좌회전: 양쪽 모터 모두 speed%
            self.differential_drive(speed, -speed)
            if self.debug:
                print(f"좌회전 명령 실행: 양쪽 모터 모두 {speed}%")
                
        elif case == 'right' or case == '우회전':
            # 우회전: 양쪽 모터 모두 speed%
            self.differential_drive(-speed, speed)
            if self.debug:
                print(f"우회전 명령 실행: 양쪽 모터 모두 {speed}%")
                
=======
>>>>>>> c63d83b (merge: Resolve conflicts)
        elif case == 'stop' or case == '정지':
            self.stop_all()
            if self.debug:
                print("정지 명령 실행")
                
<<<<<<< HEAD
=======
        elif case == 'left' or case == '좌회전':
            # 좌회전: 왼쪽 모터 후진, 오른쪽 모터 전진
            self.differential_drive(-speed, speed)
            if self.debug:
                print(f"좌회전 명령 실행: 속도 {speed}%")
                
        elif case == 'right' or case == '우회전':
            # 우회전: 왼쪽 모터 전진, 오른쪽 모터 후진
            self.differential_drive(speed, -speed)
            if self.debug:
                print(f"우회전 명령 실행: 속도 {speed}%")
                
>>>>>>> c63d83b (merge: Resolve conflicts)
        elif case == 'custom' or case == '커스텀':
            left_speed = int(y * 100)  
            right_speed = int(x * 100) 
            
            left_speed = max(-100, min(100, left_speed))
            right_speed = max(-100, min(100, right_speed))
            
            self.differential_drive(left_speed, right_speed)
            if self.debug:
                print(f"커스텀 제어: L={left_speed}, R={right_speed}")
                
        else:
            if self.debug:
                print(f"알 수 없는 케이스: {case}")
            self.stop_all()

    def send_status_to_ai(self, status_data=None):
        if not self.api_url:
            return
            
        try:
            status = {
                "serial": self.serial_number,
                "motor_a_speed": self.motor_a_speed,
                "motor_b_speed": self.motor_b_speed,
                "motor_a_direction": self.motor_a_direction,
                "motor_b_direction": self.motor_b_direction,
                "timestamp": datetime.now().isoformat()
            }
            
            if status_data:
                status.update(status_data)
                
            response = requests.post(self.api_url, json=status, timeout=5)
            
            if self.debug:
                if response.status_code == 200:
                    print("상태 전송 성공")
                else:
                    print(f"상태 전송 실패: {response.status_code}")
                    
        except Exception as e:
            if self.debug:
                print(f"상태 전송 오류: {e}")

    def run_ai_control_loop(self, interval=1.0):
        if not self.api_url:
            print("API URL이 설정되지 않았습니다.")
            return
            
        print(f"AI 제어 루프 시작 (간격: {interval}초)")
        
        try:
            while True:
                ai_data = self.get_ai_data()
                
                if ai_data:
                    self.process_ai_command(ai_data)
                    self.send_status_to_ai()
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\nAI 제어 루프 중단")
            self.stop_all()
        except Exception as e:
            print(f"AI 제어 루프 오류: {e}")
            self.stop_all()

    def set_motor_speed(self, motor, direction, speed):
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
            
        if motor == 0:  # 모터 A
            self.pwm.setDutycycle(self.PWMA, speed)
            if direction == self.FORWARD:
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            else:  # backward
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
            
            self.motor_a_speed = speed
            self.motor_a_direction = direction
            
            if self.debug:
                print(f"모터 A: {direction}, 속도: {speed}%")
                
        elif motor == 1:  # 모터 B
            self.pwm.setDutycycle(self.PWMB, speed)
            if direction == self.FORWARD:
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
            else:  # backward
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)
            
            self.motor_b_speed = speed
            self.motor_b_direction = direction
            
            if self.debug:
                print(f"모터 B: {direction}, 속도: {speed}%")
    
    def stop_motor(self, motor):
        if motor == 0:
            self.pwm.setDutycycle(self.PWMA, 0)
            self.motor_a_speed = 0
            if self.debug:
                print("모터 A 정지")
        elif motor == 1:
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
            if self.debug:
                print("모터 B 정지")
    
    def stop_all(self):
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)
        self.motor_a_speed = 0
        self.motor_b_speed = 0
        if self.debug:
            print("모든 모터 정지")
    
    def get_motor_status(self):
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
    
    def test_motors(self):
<<<<<<< HEAD
        """모터 테스트 - 모터 A와 B 동시 제어"""
        print("\n모터 테스트 시작")
        
        print("\n모터 A와 B 동시 전진 테스트")
        self.differential_drive(40, 40)
        time.sleep(2)
        
        print("\n모터 A와 B 동시 후진 테스트")
        self.differential_drive(-40, -40)
        time.sleep(2)
        
        print("\n모터 A와 B 동시 정지")
=======
        """모터 테스트 - 10초 전진, 1초 정지, 10초 후진"""
        print("\n모터 테스트 시작")
        
        # 1. 전진 10초
        print("\n1. 전진 시작 (10초)")
        self.differential_drive(30, 30)
        time.sleep(10)
        
        # 2. 정지 1초
        print("\n2. 정지 시작 (1초)")
        self.stop_all()
        time.sleep(1)
        
        # 3. 후진 10초
        print("\n3. 후진 시작 (10초)")
        self.differential_drive(-30, -30)
        time.sleep(10)
        
        # 4. 최종 정지
        print("\n4. 최종 정지")
>>>>>>> c63d83b (merge: Resolve conflicts)
        self.stop_all()
        
        print("모터 테스트 완료")
    
    def differential_drive(self, left_speed, right_speed):
        """
        차동 구동 (로봇 이동) - 모터 A와 B 동시 제어
        
        Args:
            left_speed (int): 왼쪽 모터 속도 (-100 ~ 100)
            right_speed (int): 오른쪽 모터 속도 (-100 ~ 100)
        """
<<<<<<< HEAD
=======
        # 속도 제한 (100%까지 허용)
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
>>>>>>> c63d83b (merge: Resolve conflicts)
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
        
        # 모터 B (오른쪽 모터) 설정
        if right_speed > 0:
            # 모터 B 전진
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 0)
            self.pwm.setLevel(self.BIN2, 1)
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.FORWARD
        elif right_speed < 0:
            # 모터 B 후진
            self.pwm.setDutycycle(self.PWMB, abs(right_speed))
            self.pwm.setLevel(self.BIN1, 1)
            self.pwm.setLevel(self.BIN2, 0)
            self.motor_b_speed = abs(right_speed)
            self.motor_b_direction = self.BACKWARD
        else:
            # 모터 B 정지
            self.pwm.setDutycycle(self.PWMB, 0)
            self.motor_b_speed = 0
        
        if self.debug:
            print(f"차동 구동: L={left_speed}, R={right_speed}")

    def set_motor_speed_config(self, speeds):
        self.motor_speeds.update(speeds)
        if self.debug:
            print(f"모터 속도 설정 업데이트: {self.motor_speeds}")

    def get_motor_speed(self, case):
        case = case.lower()
        return self.motor_speeds.get(case, self.motor_speeds['custom'])

def main():
    print("=" * 60)
    print("AI 모터 컨트롤러 테스트")
    print("=" * 60)
    
    ai_api_url = "http://localhost:5001/pose"
    
    try:
        motor = AIMotorController(
            debug=True, 
            api_url=ai_api_url,
            backend_broker="192.168.100.141",
            backend_port=1883
        )
        
        motor.set_serial_number("AMR001")
        
        if motor.connect_backend():
            print("Backend 연결 성공")
        else:
            print("Backend 연결 실패")
        
        motor.run_ai_control_loop(interval=1.0)
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단")
        motor.stop_all()
        motor.disconnect_backend()
    except Exception as e:
        print(f"\n오류 발생: {e}")
        motor.stop_all()
        motor.disconnect_backend()
    finally:
        print("정리 완료")

if __name__ == "__main__":
    main()
