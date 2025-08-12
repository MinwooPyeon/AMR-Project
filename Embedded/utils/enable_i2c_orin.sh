#!/bin/bash
# Jetson Orin Nano I2C 활성화 스크립트

echo "=== Jetson Orin Nano I2C 활성화 ==="

# 1. I2C 모듈 로드
echo "1. I2C 모듈 로드..."
sudo modprobe i2c-dev
sudo modprobe i2c-bcm2708

# 2. 사용자를 i2c 그룹에 추가
echo "2. 사용자를 i2c 그룹에 추가..."
sudo usermod -a -G i2c $USER

# 3. I2C 장치 권한 설정
echo "3. I2C 장치 권한 설정..."
for i in {0..9}; do
    if [ -e "/dev/i2c-$i" ]; then
        sudo chmod 666 /dev/i2c-$i
        echo "   /dev/i2c-$i 권한 설정 완료"
    fi
done

# 4. I2C 활성화 확인
echo "4. I2C 활성화 확인..."
ls -la /dev/i2c-*

# 5. I2C 장치 스캔
echo "5. I2C 장치 스캔..."
for i in {0..9}; do
    if [ -e "/dev/i2c-$i" ]; then
        echo "   I2C 버스 $i 스캔:"
        sudo i2cdetect -y $i
    fi
done

echo "=== I2C 활성화 완료 ==="
echo "재부팅 후 다시 시도하세요: sudo reboot"
