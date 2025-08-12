#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
AI 모터 컨트롤러 테스트 스크립트 (백엔드 통신 포함)
"""

import json
import time
from ai_motor_controller import AIMotorController

def test_backend_connection():
    """백엔드 연결 테스트"""
    print("=" * 60)
    print("백엔드 연결 테스트")
    print("=" * 60)
    
    # AI 모터 컨트롤러 초기화 (백엔드 연결 포함)
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 시리얼 번호 설정
    motor.set_serial_number("AMR001")
    
    try:
        # 백엔드 연결 테스트
        print("백엔드 연결 시도 중...")
        if motor.connect_backend():
            print("백엔드 연결 성공")
            
            # 연결 상태 확인
            if motor.backend_transmitter:
                print(f"Transmitter 연결 상태: {motor.backend_transmitter.connected}")
                print(f"브로커 정보: {motor.backend_transmitter.mqtt_broker}:{motor.backend_transmitter.mqtt_port}")
                print(f"로봇 ID: {motor.backend_transmitter.robot_id}")
            
            # 테스트 데이터 전송
            test_data = {
                "serial": "AMR001",
                "x": "0.5",
                "y": "0.3",
                "case": "forward",
                "timeStamp": "2024-01-01T12:00:00Z"
            }
            
            print(f"테스트 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
            
            # 백엔드로 데이터 전송
            motor_status = motor.get_motor_status()
            print(f"모터 상태: {json.dumps(motor_status, indent=2, ensure_ascii=False)}")
            
            if motor.send_to_backend(test_data, motor_status):
                print("백엔드 데이터 전송 성공")
                
                # 전송 통계 확인
                if motor.backend_transmitter:
                    stats = motor.backend_transmitter.get_transmission_stats()
                    print(f"전송 통계: {json.dumps(stats, indent=2, ensure_ascii=False)}")
            else:
                print("백엔드 데이터 전송 실패")
            
            # 연결 해제
            motor.disconnect_backend()
            print("백엔드 연결 해제 완료")
            
        else:
            print("백엔드 연결 실패")
            print("연결 실패 가능한 원인:")
            print("1. MQTT 브로커가 실행되지 않음")
            print("2. 네트워크 연결 문제")
            print("3. 브로커 주소/포트 오류")
            print("4. 방화벽 설정")
            
    except Exception as e:
        print(f"백엔드 연결 테스트 오류: {e}")
    finally:
        motor.stop_all()

def test_ai_data_processing():
    """AI 데이터 처리 테스트 (백엔드 통신 포함)"""
    print("=" * 60)
    print("AI 데이터 처리 테스트 (백엔드 통신 포함)")
    print("=" * 60)
    
    # AI 모터 컨트롤러 초기화 (백엔드 연결 포함)
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 시리얼 번호 설정
    motor.set_serial_number("AMR001")
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    # 테스트용 AI 데이터 (case는 상황 설명)
    test_cases = [
        {
            "serial": "AMR001",
            "x": "0.5",
            "y": "0.8",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:00:00Z"
        },
        {
            "serial": "AMR001",
            "x": "-0.2",
            "y": "0.8",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:01:00Z"
        },
        {
            "serial": "AMR001",
            "x": "0.0",
            "y": "0.0",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:02:00Z"
        },
        {
            "serial": "AMR001",
            "x": "0.7",
            "y": "0.4",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:03:00Z"
        }
    ]
    
    try:
        for i, test_data in enumerate(test_cases, 1):
            print(f"\n테스트 케이스 {i}:")
            print(f"입력 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
            
            # AI 명령 처리 (백엔드로 자동 전송됨)
            motor.process_ai_command(test_data)
            
            # 잠시 대기
            time.sleep(2)
            
        print("\n모든 테스트 완료!")
        
    except Exception as e:
        print(f"테스트 중 오류 발생: {e}")
    finally:
        motor.disconnect_backend()
        motor.stop_all()

def test_hardcoded_speed_values():
    """하드코딩된 Speed 값 테스트 (백엔드 통신 포함)"""
    print("\n" + "=" * 60)
    print("하드코딩된 Speed 값 테스트 (백엔드 통신 포함)")
    print("=" * 60)
    
    # AI 모터 컨트롤러 초기화 (백엔드 연결 포함)
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 시리얼 번호 설정
    motor.set_serial_number("AMR001")
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    # 하드코딩된 속도 값 확인
    print(f"기본 모터 속도 설정: {motor.motor_speeds}")
    
    # 다양한 케이스 테스트
    test_cases = [
        {"case": "forward", "expected_speed": 40},
        {"case": "backward", "expected_speed": 40},
        {"case": "stop", "expected_speed": 0},
        {"case": "custom", "expected_speed": 40}
    ]
    
    try:
        for i, test_case in enumerate(test_cases, 1):
            case = test_case["case"]
            expected_speed = test_case["expected_speed"]
            actual_speed = motor.get_motor_speed(case)
            
            print(f"\n테스트 케이스 {i}: {case}")
            print(f"예상 속도: {expected_speed}%")
            print(f"실제 속도: {actual_speed}%")
            
            if actual_speed == expected_speed:
                print("속도 값 일치")
            else:
                print("속도 값 불일치")
            
            # 테스트 데이터로 백엔드 전송 테스트
            test_data = {
                "serial": "AMR001",
                "x": "0.5",
                "y": "0.3",
                "case": case,
                "timeStamp": "2024-01-01T12:00:00Z"
            }
            
            motor.process_ai_command(test_data)
            time.sleep(1)
            
        print("\n모든 하드코딩된 Speed 테스트 완료!")
        
    except Exception as e:
        print(f"하드코딩된 Speed 테스트 중 오류 발생: {e}")
    finally:
        motor.disconnect_backend()
        motor.stop_all()

def test_speed_config_update():
    """속도 설정 변경 테스트 (백엔드 통신 포함)"""
    print("\n" + "=" * 60)
    print("속도 설정 변경 테스트 (백엔드 통신 포함)")
    print("=" * 60)
    
    # AI 모터 컨트롤러 초기화 (백엔드 연결 포함)
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 시리얼 번호 설정
    motor.set_serial_number("AMR001")
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    try:
        # 초기 설정 확인
        print(f"초기 모터 속도 설정: {motor.motor_speeds}")
        
        # 새로운 속도 설정
        new_speeds = {
                    'forward': 40,    # 전진 속도
        'backward': 40,   # 후진 속도
        'custom': 40      # 커스텀 속도
        }
        
        # 속도 설정 업데이트
        motor.set_motor_speed_config(new_speeds)
        
        # 업데이트된 설정 확인
        print(f"업데이트된 모터 속도 설정: {motor.motor_speeds}")
        
        # 테스트 데이터로 확인
        test_data = {
            "serial": "AMR001",
            "x": "0.5",
            "y": "0.3",
            "case": "forward",
            "timeStamp": "2024-01-01T12:00:00Z"
        }
        
        print(f"\n업데이트된 설정으로 테스트:")
        print(f"입력 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
        
        # AI 명령 처리 (백엔드로 자동 전송됨)
        motor.process_ai_command(test_data)
        
        print("\n속도 설정 변경 테스트 완료!")
        
    except Exception as e:
        print(f"속도 설정 변경 테스트 중 오류 발생: {e}")
    finally:
        motor.disconnect_backend()
        motor.stop_all()

def test_coordinate_based_case_detection():
    """좌표 기반 케이스 판단 테스트 (백엔드 통신 포함)"""
    print("\n" + "=" * 60)
    print("좌표 기반 케이스 판단 테스트 (백엔드 통신 포함)")
    print("=" * 60)
    
    # AI 모터 컨트롤러 초기화 (백엔드 연결 포함)
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 시리얼 번호 설정
    motor.set_serial_number("AMR001")
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    # case 필드가 상황 설명인 테스트 데이터들
    test_cases = [
        {
            "serial": "AMR001",
            "x": "0.5",
            "y": "0.8",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:00:00Z"
        },
        {
            "serial": "AMR001",
            "x": "0.5",
            "y": "-0.8",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:01:00Z"
        },
        {
            "serial": "AMR001",
            "x": "-0.8",
            "y": "0.3",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:02:00Z"
        },
        {
            "serial": "AMR001",
            "x": "0.8",
            "y": "0.3",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:03:00Z"
        },
        {
            "serial": "AMR001",
            "x": "0.1",
            "y": "0.1",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:04:00Z"
        },
        {
            "serial": "AMR001",
            "x": "0.4",
            "y": "0.4",
            "img": "base64_encoded_image_data",
            "case": "",
            "timeStamp": "2024-01-01T12:05:00Z"
        }
    ]
    
    try:
        for i, test_data in enumerate(test_cases, 1):
            print(f"\n테스트 케이스 {i}:")
            print(f"입력 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
            
            x = float(test_data.get('x', 0))
            y = float(test_data.get('y', 0))
            detected_case = motor.determine_case_from_coordinates(x, y)
            print(f"좌표 ({x}, {y}) -> 판단된 모터 제어: {detected_case}")
            
            motor.process_ai_command(test_data)
            
            time.sleep(2)
            
        print("\n모든 좌표 기반 케이스 판단 테스트 완료!")
        
    except Exception as e:
        print(f"좌표 기반 케이스 판단 테스트 중 오류 발생: {e}")
    finally:
        motor.disconnect_backend()
        motor.stop_all()

def test_threshold_adjustment():
    """임계값 조정 테스트 (백엔드 통신 포함)"""
    print("\n" + "=" * 60)
    print("임계값 조정 테스트 (백엔드 통신 포함)")
    print("=" * 60)
    
    # AI 모터 컨트롤러 초기화 (백엔드 연결 포함)
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 시리얼 번호 설정
    motor.set_serial_number("AMR001")
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    # 다양한 임계값 테스트
    threshold_tests = [
        {"x": 0.1, "y": 0.1, "description": "매우 작은 값 (정지 예상)"},
        {"x": 0.2, "y": 0.2, "description": "작은 값 (정지 예상)"},
        {"x": 0.4, "y": 0.4, "description": "중간 값 (커스텀 예상)"},
        {"x": 0.6, "y": 0.6, "description": "큰 값 (커스텀 예상)"},
        {"x": 0.1, "y": 0.5, "description": "Y가 큰 값 (전진 예상)"},
        {"x": 0.1, "y": -0.5, "description": "Y가 큰 음수 (후진 예상)"},
        {"x": 0.5, "y": 0.1, "description": "X가 큰 값 (우회전 예상)"},
        {"x": -0.5, "y": 0.1, "description": "X가 큰 음수 (좌회전 예상)"}
    ]
    
    try:
        for i, test in enumerate(threshold_tests, 1):
            x = test["x"]
            y = test["y"]
            description = test["description"]
            
            print(f"\n임계값 테스트 {i}: {description}")
            print(f"좌표: ({x}, {y})")
            
            detected_case = motor.determine_case_from_coordinates(x, y)
            print(f"판단된 케이스: {detected_case}")
            
            # 테스트 데이터 생성
            test_data = {
                "serial": "AMR001",
                "x": str(x),
                "y": str(y),
                "timeStamp": "2024-01-01T12:00:00Z"
            }
            
            # AI 명령 처리 (백엔드로 자동 전송됨)
            motor.process_ai_command(test_data)
            
            # 잠시 대기
            time.sleep(1)
            
        print("\n모든 임계값 테스트 완료!")
        
    except Exception as e:
        print(f"임계값 테스트 중 오류 발생: {e}")
    finally:
        motor.disconnect_backend()
        motor.stop_all()

def test_serial_number_mismatch():
    """시리얼 번호 불일치 테스트 (백엔드 통신 포함)"""
    print("\n" + "=" * 60)
    print("시리얼 번호 불일치 테스트 (백엔드 통신 포함)")
    print("=" * 60)
    
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    motor.set_serial_number("AMR001")
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    # 다른 시리얼 번호로 테스트
    test_data = {
        "serial": "AMR002",  # 다른 시리얼 번호
        "x": "0.5",
        "y": "0.3",
        "img": "base64_encoded_image_data",
        "case": "forward",
        "timeStamp": "2024-01-01T12:00:00Z"
    }
    
    print(f"테스트 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
    motor.process_ai_command(test_data)
    
    motor.disconnect_backend()
    motor.stop_all()

def test_invalid_data():
    """잘못된 데이터 처리 테스트 (백엔드 통신 포함)"""
    print("\n" + "=" * 60)
    print("잘못된 데이터 처리 테스트 (백엔드 통신 포함)")
    print("=" * 60)
    
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    # 잘못된 데이터들
    invalid_cases = [
        None,  # None 데이터
        {},    # 빈 딕셔너리
        {
            "serial": "AMR001",
            "x": "invalid",  # 잘못된 X 좌표
            "y": "0.3",
            "case": "unknown_case",  # 알 수 없는 케이스
            "timeStamp": "2024-01-01T12:00:00Z"
        }
    ]
    
    try:
        for i, test_data in enumerate(invalid_cases, 1):
            print(f"\n잘못된 데이터 테스트 {i}:")
            if test_data:
                print(f"입력 데이터: {json.dumps(test_data, indent=2, ensure_ascii=False)}")
            else:
                print("입력 데이터: None")
                
            motor.process_ai_command(test_data)
            time.sleep(1)
            
    except Exception as e:
        print(f"테스트 중 오류 발생: {e}")
    finally:
        motor.disconnect_backend()
        motor.stop_all()

def test_motor_control():
    """모터 제어 테스트 - 10초 전진, 1초 정지, 10초 후진"""
    print("\n" + "=" * 60)
    print("모터 제어 테스트 - 10초 전진, 1초 정지, 10초 후진")
    print("=" * 60)
    
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    try:
        print("\n=== 모터 테스트 시작 ===")
        
        # 1. 전진 10초
        print("\n1. 전진 시작 (10초)")
        motor.differential_drive(50, 50)
        time.sleep(10)
        
        # 2. 정지 1초
        print("\n2. 정지 시작 (1초)")
        motor.stop_all()
        time.sleep(1)
        
        # 3. 후진 10초
        print("\n3. 후진 시작 (10초)")
        motor.differential_drive(-50, -50)
        time.sleep(10)
        
        # 4. 최종 정지
        print("\n4. 최종 정지")
        motor.stop_all()
        
        print("\n=== 모터 테스트 완료 ===")
        
    except Exception as e:
        print(f"모터 제어 테스트 오류: {e}")
    finally:
        motor.disconnect_backend()
        motor.stop_all()

def test_status_reporting():
    """상태 보고 테스트 (백엔드 통신 포함)"""
    print("\n" + "=" * 60)
    print("상태 보고 테스트 (백엔드 통신 포함)")
    print("=" * 60)
    
    motor = AIMotorController(
        debug=True, 
        api_url=None,
        backend_broker="192.168.100.141",
        backend_port=1883
    )
    
    # 백엔드 연결
    if not motor.connect_backend():
        print("백엔드 연결 실패")
        return
    
    try:
        # 모터 상태 조회
        status = motor.get_motor_status()
        print(f"초기 상태: {json.dumps(status, indent=2, ensure_ascii=False)}")
        
        # 모터 동작 후 상태 조회
        motor.set_motor_speed(0, motor.FORWARD, 50)
        motor.set_motor_speed(1, motor.BACKWARD, 50)
        
        status = motor.get_motor_status()
        print(f"동작 후 상태: {json.dumps(status, indent=2, ensure_ascii=False)}")
        
        # 백엔드로 상태 전송
        test_data = {
            "serial": "AMR001",
            "x": "0.5",
            "y": "0.3",
            "case": "test",
            "timeStamp": "2024-01-01T12:00:00Z"
        }
        
        motor.send_to_backend(test_data, status)
        
        # 정지
        motor.stop_all()
        
    except Exception as e:
        print(f"상태 보고 테스트 오류: {e}")
    finally:
        motor.disconnect_backend()
        motor.stop_all()

def main():
    """메인 함수"""
    print("AI 모터 컨트롤러 테스트 시작 (백엔드 통신 포함)")
    
    try:
        # 모터 제어 테스트만 실행 (10초 전진, 1초 정지, 10초 후진)
        test_motor_control()
        
        print("\n모든 테스트 완료!")
        
    except Exception as e:
        print(f"테스트 중 오류 발생: {e}")

if __name__ == "__main__":
    main()
