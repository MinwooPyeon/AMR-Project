# robot_control.py
import time
from pose_api import start_pose_api_server, read_current_command_code

def actuate(code):
    # TODO: 실제 모터/트위스트 발행으로 바꾸세요.
    if code == 0: print("STOP")
    elif code == 1: print("MOVING_FORWARD")
    elif code == 2: print("MOVING_BACKWARD")
    elif code == 3: print("ROTATE_LEFT")
    elif code == 4: print("ROTATE_RIGHT")

if __name__ == "__main__":
    start_pose_api_server()  # 5001 REST API 백그라운드 실행
    while True:
        code = read_current_command_code()
        actuate(code)
        time.sleep(0.5)  # 20Hz