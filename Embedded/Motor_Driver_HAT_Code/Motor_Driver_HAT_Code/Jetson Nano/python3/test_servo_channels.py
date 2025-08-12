#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
MG996R 서보모터 채널 0-2 직접 PWM 펄스 폭 제어 테스트
Adafruit 라이브러리 대신 PCA9685 직접 제어
"""

import time
import math
from PCA9685 import PCA9685

def check_i2c_connection():
    """I2C 연결 상태 확인"""
    print("=" * 60)
    print("I2C 연결 상태 확인")
    print("=" * 60)
    
    try:
        # PCA9685 직접 초기화
        pca = PCA9685(0x60, debug=False)
        pca.setPWMFreq(50)  # 50Hz 설정
        print("✓ PCA9685 초기화 성공 (주소: 0x60)")
        print(f"✓ PWM 주파수: 50Hz")
        return True
        
    except Exception as e:
        print(f"✗ PCA9685 초기화 실패: {e}")
        return False

def angle_to_pulse(angle):
    """각도를 PWM 펄스 폭으로 변환 (RPi.GPIO 호환)"""
    # RPi.GPIO와 동일한 각도 범위: 0도 ~ 180도
    # 각도 범위 제한 (0~180도)
    if angle > 180:
        angle = 180
    
    # RPi.GPIO duty cycle 범위를 PCA9685 펄스 폭으로 변환
    # RPi.GPIO: SERVO_MIN_DUTY=3, SERVO_MAX_DUTY=12
    # duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)
    # duty = 3 + (degree * 9 / 180.0) = 3 + (degree * 0.05)
    
    # RPi.GPIO duty cycle을 펄스 폭으로 변환
    # 50Hz에서 duty cycle 3% = 0.6ms, duty cycle 12% = 2.4ms
    # 0도 → duty 3% → 0.6ms = 600μs
    # 180도 → duty 12% → 2.4ms = 2400μs
    pulse_min = 600   # 0.6ms = 600μs (0도, duty 3%)
    pulse_max = 2400  # 2.4ms = 2400μs (180도, duty 12%)
    
    # 각도를 펄스 폭으로 변환 (RPi.GPIO 방식)
    pulse_width = pulse_min + (angle / 180.0) * (pulse_max - pulse_min)
    
    # 펄스 폭을 4096 스텝으로 변환
    # 50Hz = 20ms 주기, 4096 스텝
    # 1μs = 4096 / 20000 = 0.2048 스텝
    pulse_steps = int(pulse_width * 0.2048)
    
    return pulse_steps

def set_servo_angle(pca, channel, angle, duration=1.0):
    """서보모터를 지정된 각도로 이동하고 지정된 시간만큼 대기"""
    pulse_steps = angle_to_pulse(angle)
    pca.setPWM(channel, 0, pulse_steps)
    print(f"  채널 {channel}: {angle}도 → 펄스 스텝: {pulse_steps} (대기: {duration}초)")
    time.sleep(duration)

def move_servo_sequence(pca, channel, sequence):
    """서보모터를 일련의 각도-시간 시퀀스로 제어"""
    """
    sequence 예시:
    [
        (90, 1.0),    # 90도로 이동, 1초 대기
        (45, 2.0),    # 45도로 이동, 2초 대기
        (135, 1.5),   # 135도로 이동, 1.5초 대기
        (90, 1.0),    # 90도로 복귀, 1초 대기
    ]
    """
    print(f"\n--- 채널 {channel} 시퀀스 제어 ---")
    
    for i, (angle, duration) in enumerate(sequence):
        print(f"  단계 {i+1}: {angle}도로 이동 ({duration}초 대기)")
        set_servo_angle(pca, channel, angle, duration)
    
    print(f"  채널 {channel} 시퀀스 완료")

def test_servo_90_start():
    """90도로 시작하는 서보모터 테스트"""
    print("=" * 60)
    print("MG996R 서보모터 90도 시작 테스트")
    print("=" * 60)
    
    try:
        # PCA9685 초기화
        pca = PCA9685(0x60, debug=False)
        pca.setPWMFreq(50)  # 50Hz 설정
        
        print("PCA9685 초기화 완료")
        print("PWM 주파수: 50Hz")
        print("90도 시작 테스트 모드")
        
        # 모든 채널을 90도로 초기화
        print("모든 채널을 90도로 초기화...")
        for channel in [0, 1, 2]:
            set_servo_angle(pca, channel, 90, 1.0)
        print("초기화 완료")
        
        # 초기화 대기
        print("서보모터 안정화 대기 중...")
        time.sleep(2)
        
        # 채널별 테스트 시퀀스 (90도에서 시작, 대칭 각도)
        test_sequences = {
            0: [(90, 1), (45, 0.7), (135, 0.1), (90, 0.5)],  # 채널 0: 90° → 45° → 135° → 90°
            1: [(90, 1), (45, 0.7), (135, 0.1), (90, 0.5)],  # 채널 1: 90° → 45° → 135° → 90°
            2: [(90, 1), (45, 0.7), (135, 0.1), (90, 0.5)]   # 채널 2: 90° → 45° → 135° → 90°
        }
        
        for channel, sequence in test_sequences.items():
            move_servo_sequence(pca, channel, sequence)
            time.sleep(1)  # 채널 간 대기
        
        # 모든 채널 정지
        print("\n모든 채널 정지")
        for channel in [0, 1, 2]:
            pca.setPWM(channel, 0, 0)
            print(f"  채널 {channel}: 정지")
        
        print("\n90도 시작 테스트 완료")
        
    except Exception as e:
        print(f"90도 시작 테스트 실패: {e}")
        import traceback
        traceback.print_exc()

def test_pulse_width_calculation():
    """펄스 폭 계산 테스트"""
    print("=" * 60)
    print("펄스 폭 계산 테스트")
    print("=" * 60)
    
    test_angles = [45, 90, 135]  # 대칭 각도들
    
    print("각도별 펄스 폭 계산:")
    for angle in test_angles:
        pulse_steps = angle_to_pulse(angle)
        pulse_width_us = pulse_steps / 0.2048  # 스텝을 μs로 변환
        print(f"  {angle:3d}도 → {pulse_steps:4d} 스텝 → {pulse_width_us:6.1f}μs")
    
    print("\nRPi.GPIO 호환 펄스 폭 (대칭 각도):")
    print("  45도  → 1050μs (1.05ms, duty 5.25%)")
    print("  90도  → 1500μs (1.5ms, duty 7.5%)")
    print("  135도 → 1950μs (1.95ms, duty 9.75%)")

def test_servo_connection():
    print("\n" + "=" * 60)
    print("MG996R 서보모터 연결 확인")
    print("=" * 60)
    
    print("다음 사항들을 확인하세요:")
    print("1. MG996R 서보모터가 채널 0, 1, 2에 연결되어 있는지 확인")
    print("2. 각 서보모터의 선이 제대로 연결되어 있는지 확인:")
    print("   - 빨간선: 5V 전원")
    print("   - 검은선: GND (접지)")
    print("   - 노란선/흰선: 신호선")
    print("3. 서보모터 드라이버의 채널 번호가 올바른지 확인")
    print("4. MG996R 서보모터가 손상되지 않았는지 확인")
    print("5. MG996R은 RPi.GPIO와 동일한 PWM 제어 방식으로 동작합니다:")
    print("   - 펄스 폭 0.6ms: 0도 (최소 위치, duty 3%)")
    print("   - 펄스 폭 1.5ms: 90도 (중앙 정지 위치, duty 7.5%)")
    print("   - 펄스 폭 2.4ms: 180도 (최대 위치, duty 12%)")
    print("6. 실제 동작 각도 범위: 0도 ~ 180도 (RPi.GPIO 호환)")
    print("\n추가 확인사항:")
    print("7. 전원 공급이 충분한지 확인 (5V, 최소 1A)")
    print("8. 서보모터가 과부하되지 않았는지 확인")
    print("9. 서보모터의 기계적 제한에 걸리지 않았는지 확인")
    print("10. PWM 주파수가 50Hz로 설정되어 있는지 확인")

if __name__ == "__main__":
    # 1. I2C 연결 상태 확인
    if not check_i2c_connection():
        print("\nI2C 연결 문제가 있습니다.")
        exit(1)
    
    # 2. 펄스 폭 계산 테스트
    test_pulse_width_calculation()
    
    # 3. 90도 시작 테스트
    test_servo_90_start()
    
    # 4. 연결 확인 가이드
    test_servo_connection()