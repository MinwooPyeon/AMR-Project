# Jetson Orin Nano I2C 문제 해결 가이드

## 🔍 문제 진단

### 1. I2C 채널 확인
```bash
# I2C 채널 확인 스크립트 실행
python3 check_i2c_channels.py
```

### 2. I2C 활성화
```bash
# I2C 활성화 스크립트 실행
chmod +x enable_i2c_orin.sh
sudo ./enable_i2c_orin.sh
```

### 3. 수동 I2C 확인
```bash
# I2C 장치 확인
ls -la /dev/i2c-*

# I2C 버스 스캔
sudo i2cdetect -y 0
sudo i2cdetect -y 1
sudo i2cdetect -y 2
# ... (필요한 만큼)

# 권한 확인
ls -la /dev/i2c-1
```

## 🛠️ 해결 방법

### 1. I2C 권한 문제
```bash
# 사용자를 i2c 그룹에 추가
sudo usermod -a -G i2c $USER

# I2C 장치 권한 설정
sudo chmod 666 /dev/i2c-*

# 재부팅
sudo reboot
```

### 2. I2C 모듈 로드
```bash
# I2C 모듈 로드
sudo modprobe i2c-dev
sudo modprobe i2c-bcm2708

# 모듈 확인
lsmod | grep i2c
```

### 3. 하드웨어 연결 확인

#### Jetson Orin Nano 핀 연결
```
I2C1 (27, 28번 핀):
- 27번 핀: I2C1_SDA
- 28번 핀: I2C1_SCL

I2C2 (3, 5번 핀):
- 3번 핀: I2C2_SDA  
- 5번 핀: I2C2_SCL

I2C3 (23, 24번 핀):
- 23번 핀: I2C3_SDA
- 24번 핀: I2C3_SCL
```

#### 연결 확인 사항
1. **전원 연결**: VCC → 3.3V, GND → GND
2. **풀업 저항**: SDA, SCL에 4.7kΩ 풀업 저항 연결
3. **배선 길이**: 너무 긴 배선은 노이즈 발생 가능
4. **접촉 상태**: 핀과 소켓의 접촉 상태 확인

### 4. 코드 수정

#### 자동 I2C 버스 감지 사용
```python
# 기존 코드
controller = IMUAIMotorController(i2c_bus=1)

# 수정된 코드 (자동 감지)
controller = IMUAIMotorController()  # i2c_bus 파라미터 생략
```

#### 특정 버스 지정
```python
# I2C 버스 0 사용
controller = IMUAIMotorController(i2c_bus=0)

# I2C 버스 2 사용  
controller = IMUAIMotorController(i2c_bus=2)
```

## 🔧 일반적인 문제들

### 1. "No such file or directory" 오류
```bash
# I2C 장치가 없는 경우
sudo dtoverlay i2c1
sudo dtoverlay i2c2
sudo dtoverlay i2c3

# config.txt에 추가
echo "dtoverlay=i2c1" | sudo tee -a /boot/config.txt
echo "dtoverlay=i2c2" | sudo tee -a /boot/config.txt
echo "dtoverlay=i2c3" | sudo tee -a /boot/config.txt
```

### 2. "Permission denied" 오류
```bash
# 권한 문제 해결
sudo chmod 666 /dev/i2c-*
sudo usermod -a -G i2c $USER
newgrp i2c
```

### 3. "Device or resource busy" 오류
```bash
# 다른 프로세스가 I2C를 사용 중인 경우
sudo lsof /dev/i2c-*
sudo kill -9 [PID]
```

### 4. "Input/output error" 오류
```bash
# 하드웨어 연결 문제
# 1. 배선 재확인
# 2. 전원 공급 확인
# 3. I2C 주소 확인
# 4. 다른 I2C 버스 시도
```

## 📋 체크리스트

### 하드웨어 체크
- [ ] VCC → 3.3V 연결
- [ ] GND → GND 연결  
- [ ] SDA → 27번 핀 연결
- [ ] SCL → 28번 핀 연결
- [ ] 풀업 저항 연결 (4.7kΩ)
- [ ] 배선 접촉 상태 확인

### 소프트웨어 체크
- [ ] I2C 모듈 로드
- [ ] 사용자 i2c 그룹 추가
- [ ] I2C 장치 권한 설정
- [ ] I2C 버스 스캔 성공
- [ ] 코드에서 올바른 버스 사용

### 테스트 체크
- [ ] `check_i2c_channels.py` 실행 성공
- [ ] `i2cdetect` 명령 성공
- [ ] IMU 컨트롤러 초기화 성공
- [ ] 모터 드라이버 초기화 성공

## 🚨 주의사항

1. **전원 공급**: 안정적인 3.3V 전원 공급 확인
2. **배선 길이**: 너무 긴 배선은 신호 품질 저하
3. **노이즈**: 모터나 다른 장치의 전자기 간섭 주의
4. **온도**: 고온에서 I2C 통신 불안정 가능
5. **재부팅**: 설정 변경 후 반드시 재부팅

## 📞 추가 지원

문제가 지속되면:
1. `check_i2c_channels.py` 결과 공유
2. `i2cdetect` 출력 결과 공유  
3. 하드웨어 연결 사진 공유
4. 오류 메시지 전체 내용 공유
