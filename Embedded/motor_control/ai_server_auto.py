#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
AI 서버 (자동 명령 전송)
실시간으로 AI가 결정한 명령을 임베디드 장치에 전송합니다.
"""

import sys
import threading
import time
import random
from flask import Flask, jsonify, request

app = Flask(__name__)

# 전역 변수로 현재 AI 명령 저장
lock = threading.Lock()
current_ai_command = {'code': 0}  # 초기값: STOP

COMMAND_TABLE = {
    0: 'STOP',
    1: 'MOVING_FORWARD',
    2: 'MOVING_BACKWARD',
    3: 'ROTATE_LEFT',
    4: 'ROTATE_RIGHT'
}

# AI 자동 명령 시퀀스
AUTO_COMMAND_SEQUENCES = {
    'basic_test': [
        (1, 3),   # 전진 3초
        (0, 1),   # 정지 1초  
        (2, 2),   # 후진 2초
        (0, 1),   # 정지 1초
        (3, 2),   # 좌회전 2초
        (0, 1),   # 정지 1초
        (4, 2),   # 우회전 2초
        (0, 2),   # 정지 2초
    ],
    'square_pattern': [
        (1, 2),   # 전진 2초
        (0, 0.5), # 정지 0.5초
        (4, 1),   # 우회전 1초 (90도)
        (0, 0.5), # 정지 0.5초
        (1, 2),   # 전진 2초
        (0, 0.5), # 정지 0.5초
        (4, 1),   # 우회전 1초
        (0, 0.5), # 정지 0.5초
        (1, 2),   # 전진 2초
        (0, 0.5), # 정지 0.5초
        (4, 1),   # 우회전 1초
        (0, 0.5), # 정지 0.5초
        (1, 2),   # 전진 2초
        (0, 0.5), # 정지 0.5초
        (4, 1),   # 우회전 1초 (완전한 사각형)
        (0, 2),   # 정지 2초
    ],

    'random_walk': [
        # 랜덤 패턴은 코드에서 동적 생성
    ]
}

# AI 상태
ai_mode = 'manual'  # 'manual', 'auto', 'sequence'
current_sequence = None
sequence_index = 0
auto_thread = None
auto_running = False

@app.route('/command', methods=['GET'])
def get_command():
    """임베디드에서 호출: 현재 AI 명령 반환"""
    with lock:
        code = current_ai_command['code']
        name = COMMAND_TABLE.get(code, 'UNKNOWN')
    
    print(f"[GET] 임베디드 명령 요청: {code} - {name}")
    return jsonify({
        'code': code,
        'name': name
    })

@app.route('/ai_command', methods=['POST'])
def set_ai_command():
    """AI가 새로운 명령을 설정"""
    global current_ai_command
    
    data = request.get_json()
    if not data or 'code' not in data:
        return jsonify({"status": "error", "message": "Missing code parameter"}), 400
    
    code = data['code']
    if code not in COMMAND_TABLE:
        return jsonify({"status": "error", "message": "Invalid command code"}), 400
    
    with lock:
        old_code = current_ai_command['code']
        current_ai_command['code'] = code
    
    name = COMMAND_TABLE[code]
    print(f"[AI] 명령 변경: {old_code} → {code} ({name})")
    return jsonify({
        'status': 'ok',
        'code': code,
        'name': name
    })

@app.route('/ai_mode', methods=['POST'])
def set_ai_mode():
    """AI 모드 설정 (manual/auto/sequence)"""
    global ai_mode, current_sequence
    
    data = request.get_json()
    if not data or 'mode' not in data:
        return jsonify({"status": "error", "message": "Missing mode parameter"}), 400
    
    mode = data['mode']
    if mode not in ['manual', 'auto', 'sequence']:
        return jsonify({"status": "error", "message": "Invalid mode"}), 400
    
    ai_mode = mode
    
    if mode == 'sequence':
        sequence_name = data.get('sequence', 'basic_test')
        if sequence_name in AUTO_COMMAND_SEQUENCES:
            current_sequence = AUTO_COMMAND_SEQUENCES[sequence_name]
            start_auto_sequence()
        else:
            return jsonify({"status": "error", "message": "Invalid sequence name"}), 400
    elif mode == 'auto':
        start_auto_random()
    else:
        stop_auto_mode()
    
    print(f"[AI MODE] 모드 변경: {mode}")
    return jsonify({'status': 'ok', 'mode': mode})

@app.route('/status', methods=['GET'])
def get_status():
    """전체 상태 확인"""
    with lock:
        code = current_ai_command['code']
        name = COMMAND_TABLE.get(code, 'UNKNOWN')
    
    return jsonify({
        'current_command': {
            'code': code,
            'name': name
        },
        'available_commands': COMMAND_TABLE,
        'ai_mode': ai_mode,
        'auto_running': auto_running,
        'available_sequences': list(AUTO_COMMAND_SEQUENCES.keys()),
        'server_type': 'AI_SERVER_AUTO'
    })

def update_ai_command(code):
    """AI 명령 업데이트 (내부 함수)"""
    with lock:
        old_code = current_ai_command['code']
        current_ai_command['code'] = code
    
    name = COMMAND_TABLE[code]
    print(f"[AUTO] 명령 자동 변경: {old_code} → {code} ({name})")

def auto_sequence_thread():
    """자동 시퀀스 실행 스레드"""
    global sequence_index, auto_running
    
    while auto_running and current_sequence:
        if sequence_index >= len(current_sequence):
            print("[AUTO] 시퀀스 완료, 반복 시작")
            sequence_index = 0
        
        command_code, duration = current_sequence[sequence_index]
        update_ai_command(command_code)
        
        # 명령 실행 시간만큼 대기
        start_time = time.time()
        while auto_running and (time.time() - start_time) < duration:
            time.sleep(0.1)
        
        sequence_index += 1
    
    print("[AUTO] 자동 시퀀스 종료")

def auto_random_thread():
    """자동 랜덤 명령 스레드"""
    while auto_running:
        # 랜덤 명령 생성 (가중치 적용)
        weights = [30, 25, 20, 10, 10]  # STOP이 더 자주 나오도록
        command_code = random.choices(list(COMMAND_TABLE.keys()), weights=weights)[0]
        
        # 지속 시간 랜덤 (1-4초)
        duration = random.uniform(1, 4)
        
        update_ai_command(command_code)
        
        # 명령 실행 시간만큼 대기
        start_time = time.time()
        while auto_running and (time.time() - start_time) < duration:
            time.sleep(0.1)
    
    print("[AUTO] 자동 랜덤 모드 종료")

def start_auto_sequence():
    """자동 시퀀스 시작"""
    global auto_thread, auto_running, sequence_index
    
    stop_auto_mode()  # 기존 자동 모드 정지
    
    auto_running = True
    sequence_index = 0
    auto_thread = threading.Thread(target=auto_sequence_thread, daemon=True)
    auto_thread.start()
    print("[AUTO] 자동 시퀀스 모드 시작")

def start_auto_random():
    """자동 랜덤 모드 시작"""
    global auto_thread, auto_running
    
    stop_auto_mode()  # 기존 자동 모드 정지
    
    auto_running = True
    auto_thread = threading.Thread(target=auto_random_thread, daemon=True)
    auto_thread.start()
    print("[AUTO] 자동 랜덤 모드 시작")

def stop_auto_mode():
    """자동 모드 정지"""
    global auto_running, auto_thread
    
    auto_running = False
    if auto_thread and auto_thread.is_alive():
        auto_thread.join(timeout=1.0)
    
    # 정지 명령 전송
    update_ai_command(0)

def run_server():
    """Flask 서버 실행"""
    app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)

def run_control_interface():
    """실시간 직접 명령 입력 인터페이스"""
    print("실시간 로봇 제어")
    print("=" * 50)
    print("사용 가능한 명령:")
    for code, name in COMMAND_TABLE.items():
        print(f"  {code}: {name}")
    print("  s: 상태 확인")
    print("  q: 종료")
    print("=" * 50)
    
    # 수동 모드로 설정
    import requests
    try:
        requests.post("http://localhost:5001/ai_mode", json={'mode': 'manual'}, timeout=2)
        print("✅ 수동 제어 모드로 설정됨")
    except:
        print("⚠️  AI 모드 설정 실패 (계속 진행)")
    
    print("\n💡 명령을 직접 입력하세요 (0-4):")
    
    while True:
        try:
            user_input = input("🎮 명령: ").strip().lower()
            
            if user_input == 'q':
                print("👋 종료합니다.")
                # 정지 명령 후 종료
                try:
                    requests.post("http://localhost:5001/ai_command", json={'code': 0}, timeout=2)
                except:
                    pass
                break
            elif user_input == 's':
                # 상태 확인
                try:
                    response = requests.get("http://localhost:5001/status", timeout=2)
                    if response.status_code == 200:
                        data = response.json()
                        cmd = data['current_command']
                        print(f"📡 현재 상태: {cmd['code']} - {cmd['name']}")
                    else:
                        print("❌ 상태 확인 실패")
                except:
                    print("❌ 서버 연결 실패")
                continue
            
            # 숫자 명령 처리
            try:
                code = int(user_input)
                if code in COMMAND_TABLE:
                    # 명령 전송
                    try:
                        response = requests.post(
                            "http://localhost:5001/ai_command", 
                            json={'code': code}, 
                            timeout=2
                        )
                        if response.status_code == 200:
                            print(f"✅ {code} - {COMMAND_TABLE[code]} 명령 전송됨")
                        else:
                            print(f"❌ 명령 전송 실패: {response.status_code}")
                    except:
                        print("❌ 서버 연결 실패")
                else:
                    print(f"❌ 잘못된 명령: {code} (0-4만 사용 가능)")
            except ValueError:
                print("❌ 숫자를 입력하세요 (0-4, s, q)")
                
        except KeyboardInterrupt:
            print("\n👋 Ctrl+C로 종료됨")
            # 정지 명령 후 종료
            try:
                requests.post("http://localhost:5001/ai_command", json={'code': 0}, timeout=2)
            except:
                pass
            break
        except EOFError:
            print("\n👋 입력 스트림 종료")
            break

def main():
    print("🧠 AI 서버 (실시간 직접 제어)")
    print("=" * 50)
    print("모드 선택:")
    print("1. 서버만 실행")
    print("2. 서버 + 실시간 직접 제어")
    
    choice = input("선택 (1-2): ").strip()
    
    if choice == "1":
        print("🚀 AI 서버 시작 중...")
        print("http://localhost:5001 에서 실행 중")
        print("임베디드 장치가 실시간으로 명령을 받을 수 있습니다")
        print("Ctrl+C로 종료")
        run_server()
    elif choice == "2":
        print("🚀 AI 서버 + 실시간 직접 제어 시작")
        # 서버를 별도 스레드에서 실행
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
        # 서버 시작 대기
        time.sleep(2)
        
        run_control_interface()
    else:
        print("❌ 잘못된 선택")

if __name__ == "__main__":
    main()
