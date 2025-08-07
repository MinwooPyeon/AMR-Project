from flask import Flask, jsonify
import threading

app = Flask(__name__)

# robot_pose를 외부에서 업데이트 가능하게 모듈화
robot_pose = {'x': None, 'y': None}

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
