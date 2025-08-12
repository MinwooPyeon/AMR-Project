#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Flask 웹 서버를 통한 로봇 제어 스크립트
http://localhost:5001/command로 명령을 받아 로봇을 제어
"""

import time
import json
from flask import Flask, request, jsonify
from ai_motor_controller import AIMotorController
from threading import Lock

app = Flask(__name__)

# 명령 테이블 정의
COMMAND_TABLE = {
    0: 'STOP',
    1: 'MOVING_FORWARD',
    2: 'MOVING_BACKWARD',
    3: 'ROTATE_LEFT',
    4: 'ROTATE_RIGHT',
}

# 로봇 명령 상태 (스레드 안전)
robot_command = {'code': 0}  # 기본값: 정지
lock = Lock()

# 로봇 모터 컨트롤러 초기화
motor = None

def init_motor():
    """모터 컨트롤러 초기화"""
    global motor
    try:
        print("🔧 모터 컨트롤러 초기화 중...")
        motor = AIMotorController(debug=True)
        print("✅ 모터 컨트롤러 초기화 완료")
        return True
    except Exception as e:
        print(f"❌ 모터 컨트롤러 초기화 실패: {e}")
        print("💡 다음 사항을 확인하세요:")
        print("   1. I2C 연결 상태")
        print("   2. PCA9685 드라이버 연결")
        print("   3. 권한 설정 (sudo 권한 필요할 수 있음)")
        return False

def execute_movement(command_code):
    """명령 코드에 따라 로봇 이동 실행 (빠른 응답)"""
    if motor is None:
        return {"status": "error", "message": "모터가 초기화되지 않았습니다"}
    
    try:
        command_name = COMMAND_TABLE.get(command_code, 'UNKNOWN')
        print(f"🎮 명령 실행: {command_code} ({command_name})")
        
        # 빠른 응답을 위해 즉시 명령 실행
        if command_code == 0:  # STOP
            motor.stop_all()
            print("✅ 정지 명령 완료")
            return {"status": "success", "message": "로봇이 정지했습니다", "code": command_code}
            
        elif command_code == 1:  # MOVING_FORWARD
            motor.set_motor_speed(0, motor.FORWARD, 50)
            motor.set_motor_speed(1, motor.FORWARD, 50)
            print("✅ 직진 명령 완료")
            return {"status": "success", "message": "로봇이 직진을 시작했습니다", "code": command_code}
            
        elif command_code == 2:  # MOVING_BACKWARD
            motor.set_motor_speed(0, motor.BACKWARD, 50)
            motor.set_motor_speed(1, motor.BACKWARD, 50)
            print("✅ 후진 명령 완료")
            return {"status": "success", "message": "로봇이 후진을 시작했습니다", "code": command_code}
            
        elif command_code == 3:  # ROTATE_LEFT
            motor.set_motor_speed(0, motor.BACKWARD, 50)
            motor.set_motor_speed(1, motor.FORWARD, 50)
            print("✅ 좌회전 명령 완료")
            return {"status": "success", "message": "로봇이 좌회전을 시작했습니다", "code": command_code}
            
        elif command_code == 4:  # ROTATE_RIGHT
            motor.set_motor_speed(0, motor.FORWARD, 50)
            motor.set_motor_speed(1, motor.BACKWARD, 50)
            print("✅ 우회전 명령 완료")
            return {"status": "success", "message": "로봇이 우회전을 시작했습니다", "code": command_code}
            
        else:
            return {"status": "error", "message": f"알 수 없는 명령 코드: {command_code}"}
        
    except Exception as e:
        print(f"❌ 명령 실행 오류: {e}")
        motor.stop_all()
        return {"status": "error", "message": f"명령 실행 중 오류: {e}"}

@app.route('/', methods=['GET'])
def root():
    """루트 경로 - 서버 정보 및 사용법"""
    return jsonify({
        "server": "Flask Robot Control Server",
        "port": 5001,
        "status": "running",
        "usage": {
            "commands": "숫자 명령으로 로봇 제어",
            "examples": {
                "0": "정지 - http://localhost:5001/0",
                "1": "직진 - http://localhost:5001/1", 
                "2": "후진 - http://localhost:5001/2",
                "3": "좌회전 - http://localhost:5001/3",
                "4": "우회전 - http://localhost:5001/4"
            },
            "status": "현재 상태 확인 - http://localhost:5001/command",
            "server_status": "서버 상태 확인 - http://localhost:5001/status"
        },
        "available_commands": COMMAND_TABLE
    })

@app.route('/command', methods=['GET'])
def get_command():
    """GET 요청으로 로봇 제어 명령 설정 및 상태 확인"""
    try:
        # URL 파라미터에서 명령 코드 가져오기
        command_code = request.args.get('code', type=int)
        
        if command_code is not None:
            # 명령 코드가 제공된 경우 - 모터 제어 실행
            if command_code not in COMMAND_TABLE:
                return jsonify({"status": "error", "message": f"알 수 없는 명령 코드: {command_code}"}), 400
            
            # 명령 코드 업데이트
            with lock:
                robot_command['code'] = command_code
            
            print(f"🎮 명령 실행: {command_code} ({COMMAND_TABLE[command_code]})")
            
            # 명령 실행 (빠른 응답)
            result = execute_movement(command_code)
            print(f"🚀 응답 전송: {result}")
            return jsonify(result)
        
        else:
            # 명령 코드가 없는 경우 - 현재 상태만 반환
            with lock:
                code = robot_command['code']
                name = COMMAND_TABLE.get(code, 'UNKNOWN')
            return jsonify({'code': code, 'name': name})
        
    except Exception as e:
        print(f"❌ 명령 처리 오류: {e}")
        return jsonify({"status": "error", "message": f"서버 오류: {e}"}), 500

@app.route('/status', methods=['GET'])
def get_status():
    """서버 상태 확인"""
    return jsonify({
        "status": "running",
        "server": "Flask Robot Control Server",
        "port": 5001,
        "motor_initialized": motor is not None,
        "current_command": {
            "code": robot_command['code'],
            "name": COMMAND_TABLE.get(robot_command['code'], 'UNKNOWN')
        },
        "available_commands": COMMAND_TABLE
    })

@app.route('/test', methods=['GET'])
def test_movement():
    """테스트용 엔드포인트 - 모든 동작 테스트"""
    try:
        print("테스트 시퀀스 시작...")
        
        # 1. 직진 2초
        result = execute_movement(1)  # MOVING_FORWARD
        if "status" in result and result["status"] == "error":
            return jsonify(result), 500
        
        time.sleep(2)
        
        # 2. 정지
        result = execute_movement(0)  # STOP
        if "status" in result and result["status"] == "error":
            return jsonify(result), 500
        
        time.sleep(2)
        
        # 3. 후진 2초
        result = execute_movement(2)  # MOVING_BACKWARD
        if "status" in result and result["status"] == "error":
            return jsonify(result), 500
        
        time.sleep(2)
        
        # 4. 정지
        result = execute_movement(0)  # STOP
        if "status" in result and result["status"] == "error":
            return jsonify(result), 500
        
        time.sleep(2)
        
        # 5. 우회전 2초
        result = execute_movement(4)  # ROTATE_RIGHT
        if "status" in result and result["status"] == "error":
            return jsonify(result), 500
        
        time.sleep(2)
        
        # 6. 정지
        result = execute_movement(0)  # STOP
        if "status" in result and result["status"] == "error":
            return jsonify(result), 500
        
        time.sleep(10)
        
        # 7. 좌회전 2초
        result = execute_movement(3)  # ROTATE_LEFT
        if "status" in result and result["status"] == "error":
            return jsonify(result), 500
        
        time.sleep(2)
        
        # 8. 정지
        result = execute_movement(0)  # STOP
        if "status" in result and result["status"] == "error":
            return jsonify(result), 500
        
        return jsonify({
            "status": "success",
            "message": "모든 테스트 동작 완료"
        })
        
    except Exception as e:
        return jsonify({"status": "error", "message": f"테스트 중 오류: {e}"}), 500

def main():
    """메인 함수"""
    print("Flask 로봇 제어 서버 시작")
    print("=" * 50)
    print("서버 주소: http://localhost:5001")
    print("명령 엔드포인트: http://localhost:5001/command?code=1")
    print("상태 확인: http://localhost:5001/status")
    print("=" * 50)
    
    # 모터 초기화
    if not init_motor():
        print("⚠️  모터 초기화 실패로 서버를 시작할 수 없습니다")
        print("💡 I2C 연결과 모터 드라이버를 확인하세요")
        return
    
    print("✅ 모터 초기화 완료")
    print("🚀 Flask 서버 시작 중...")
    
    try:
        # Flask 서버 시작 (응답 시간 개선)
        app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
    except Exception as e:
        print(f"❌ 서버 시작 실패: {e}")
        print("💡 포트 5001이 이미 사용 중인지 확인하세요")
        print("   다른 프로세스가 실행 중이라면 종료 후 다시 시도하세요")

if __name__ == "__main__":
    main()
