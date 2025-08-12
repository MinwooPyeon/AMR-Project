import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# I2C 초기화
i2c = busio.I2C(board.SCL, board.SDA)

# PCA9685 객체 생성
pca = PCA9685(i2c)
pca.frequency = 50  # 서보에 맞는 주파수 설정

# 서보 객체 생성 (채널 0번) 채널 변경해서 테스트
servo0 = servo.Servo(pca.channels[0])

# 서보 테스트 동작
for angle in range(0, 180, 30):
    servo0.angle = angle
    time.sleep(0.5)

# 종료 전 각도 초기화
servo0.angle = 90