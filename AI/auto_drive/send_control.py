import sys
import requests
from flask import Flask, jsonify, request, abort
app = Flask(__name__)
API_URL = "http://127.0.0.1:5001/command"  # pose_api 서버 주소
COMMAND_TABLE = {
    0: 'STOP',
    1: 'MOVING_FORWARD',
    2: 'MOVING_BACKWARD',
    3: 'ROTATE_LEFT',
    4: 'ROTATE_RIGHT',
}
@app.route('/command', methods=['GET'])
def get_command():
    # URL 쿼리 파라미터에서 code 값 읽기
    code = request.args.get('code', type=int)

    # 유효성 검사
    if code not in [0, 1, 2, 3, 4]:
        return jsonify({"status": "error", "message": "Invalid command code"}), 400
    name = COMMAND_TABLE.get(code, 'UNKNOWN')
    # 실제 처리 로직 (예: 로봇 제어)
    # 여기서는 예시로 단순 응답
    return jsonify({
        'code': code,
        'name': name
    })
    
def main():
    while True:
        try:
            code_str = input("명령 코드 입력 (0=STOP,1=FORWARD,2=BACKWARD,3=LEFT,4=RIGHT,q=종료): ").strip()
            if code_str.lower() == 'q':
                break
            code = int(code_str)
            if code not in [0, 1, 2, 3, 4]:
                print("❌ 잘못된 코드")
                continue
            r = requests.get(API_URL, params={"code": code}, timeout=2)
            print("✅ 서버 응답:", r.json())
        except ValueError:
            print("❌ 숫자만 입력하세요.")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print("⚠️ 요청 실패:", e)

if __name__ == "__main__":
    main()