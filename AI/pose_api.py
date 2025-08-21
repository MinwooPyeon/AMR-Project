from flask import Flask, jsonify, request, abort
import threading

app = Flask(__name__)

# robot_pose를 외부에서 업데이트 가능하게 모듈화
robot_pose = {'x': None, 'y': None}
# --- Command 상태 (추가) ---
# 기본값 STOP(0)
robot_command = {'code': 0}
COMMAND_TABLE = {
    0: 'STOP',
    1: 'MOVING_FORWARD',
    2: 'MOVING_BACKWARD',
    3: 'ROTATE_LEFT',
    4: 'ROTATE_RIGHT',
}

lock = threading.Lock()

@app.route('/pose', methods=['GET'])
def get_robot_pose():
    return jsonify({
        'x': robot_pose['x'] if robot_pose['x'] is not None else 0.0,
        'y': robot_pose['y'] if robot_pose['y'] is not None else 0.0
    })

def run_api():
    app.run(host='0.0.0.0', port=5001)  # 포트 5001에서 서버 실행

# 백그라운드에서 실행할 수 있도록 함수 제공
def start_pose_api_server():
    threading.Thread(target=run_api, daemon=True).start()

# pose 값을 외부에서 업데이트 가능하게 함수 제공
def update_pose(x, y):
    robot_pose['x'] = x
    robot_pose['y'] = y


@app.route('/command', methods=['GET'])
def get_command():
    with lock:
        code = robot_command['code']
        name = COMMAND_TABLE.get(code, 'UNKNOWN')
    return jsonify({'code': code, 'name': name})

@app.route('/command', methods=['POST'])
def set_command():
    """
    JSON 예:
    { "code": 1 }  # 0~4
    혹은 폼/쿼리로 code=1 도 허용
    """
    code = None
    if request.is_json:
        data = request.get_json(silent=True) or {}
        code = data.get('code', None)
    else:
        # form or querystring fallback
        code = request.values.get('code', None)

    try:
        code = int(code)
    except (TypeError, ValueError):
        abort(400, 'code must be an integer in [0..4]')

    if code not in COMMAND_TABLE:
        abort(400, 'invalid code: must be one of 0 STOP, 1 FORWARD, 2 BACKWARD, 3 LEFT, 4 RIGHT')

    with lock:
        robot_command['code'] = code
        name = COMMAND_TABLE[code]
    return jsonify({'ok': True, 'code': code, 'name': name})

# 로컬에서 현재 명령 읽기 (로봇 제어 루프에서 import 후 사용 가능)
def read_current_command_code():
    with lock:
        return robot_command['code']