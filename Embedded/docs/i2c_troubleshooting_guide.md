# I2C 통신 문제 해결 가이드

## 개요

이 문서는 Jetson Nano에서 I2C 통신 관련 문제를 진단하고 해결하는 방법을 설명합니다.

## 기본 진단

### 1. I2C 버스 확인

```bash
# I2C 버스 목록 확인
ls /dev/i2c*

# I2C 버스 정보 확인
i2cdetect -l
```

### 2. I2C 장치 스캔

```bash
# I2C-1 버스 스캔
i2cdetect -y 1

# I2C-0 버스 스캔 (필요한 경우)
i2cdetect -y 0
```

### 3. 권한 확인

```bash
# I2C 장치 권한 확인
ls -la /dev/i2c*

# 사용자 그룹 확인
groups $USER
```

## 일반적인 문제 및 해결 방법

### 문제 1: I2C 장치가 감지되지 않음

#### 증상

- `i2cdetect`에서 장치가 표시되지 않음
- Python에서 `OSError: [Errno 121] Remote I/O error` 발생

#### 해결 방법

1. **하드웨어 연결 확인**

   ```bash
   # 연결 상태 확인
   sudo i2cdetect -y 1
   ```

2. **I2C 활성화**

   ```bash
   # I2C 활성화
   sudo usermod -a -G i2c $USER
   sudo reboot
   ```

3. **권한 설정**

   ```bash
   # I2C 장치 권한 변경
   sudo chmod 666 /dev/i2c-1
   ```

4. **I2C 드라이버 로드**

   ```bash
   # I2C 드라이버 확인
   lsmod | grep i2c

   # 드라이버 수동 로드 (필요한 경우)
   sudo modprobe i2c-dev
   ```

### 문제 2: 권한 오류

#### 증상

- `Permission denied` 오류
- `[Errno 13] Permission denied` 발생

#### 해결 방법

```bash
# 사용자를 i2c 그룹에 추가
sudo usermod -a -G i2c $USER

# 권한 확인
ls -la /dev/i2c-1

# 권한 수정
sudo chmod 666 /dev/i2c-1

# 재부팅
sudo reboot
```

### 문제 3: 주소 충돌

#### 증상

- 여러 장치가 같은 주소를 사용
- 일부 장치만 감지됨

#### 해결 방법

1. **주소 확인**

   ```bash
   # 모든 I2C 장치 주소 확인
   i2cdetect -y 1
   ```

2. **장치별 주소 설정**

   ```python
   # IMU 센서 주소
   IMU_ADDRESS = 0x68

   # 모터 드라이버 주소
   MOTOR_ADDRESS = 0x40
   ```

### 문제 4: 통신 속도 문제

#### 증상

- 데이터 읽기/쓰기 실패
- 타임아웃 오류

#### 해결 방법

```python
import smbus2

# 낮은 속도로 설정
bus = smbus2.SMBus(1)

# 통신 속도 조정
try:
    # 기본 속도로 시도
    data = bus.read_byte_data(address, register)
except:
    # 낮은 속도로 재시도
    bus.close()
    bus = smbus2.SMBus(1)
    data = bus.read_byte_data(address, register)
```

## 고급 진단

### 1. I2C 로그 확인

```bash
# 커널 로그 확인
dmesg | grep i2c

# 시스템 로그 확인
journalctl -f | grep i2c
```

### 2. 하드웨어 진단

```bash
# 전압 확인
sudo i2cget -y 1 0x68 0x75  # IMU WHO_AM_I 레지스터

# 온도 확인
sudo i2cget -y 1 0x68 0x41  # IMU 온도 레지스터
```

### 3. Python 진단 스크립트

```python
#!/usr/bin/env python3

import smbus2
import time

def test_i2c_connection():
    """I2C 연결 테스트"""
    try:
        bus = smbus2.SMBus(1)
        print("I2C 버스 연결 성공")

        # IMU 테스트
        try:
            who_am_i = bus.read_byte_data(0x68, 0x75)
            print(f"IMU WHO_AM_I: 0x{who_am_i:02x}")
        except Exception as e:
            print(f"IMU 연결 실패: {e}")

        # 모터 드라이버 테스트
        try:
            mode1 = bus.read_byte_data(0x40, 0x00)
            print(f"Motor Driver MODE1: 0x{mode1:02x}")
        except Exception as e:
            print(f"모터 드라이버 연결 실패: {e}")

        bus.close()

    except Exception as e:
        print(f"I2C 연결 실패: {e}")

if __name__ == "__main__":
    test_i2c_connection()
```

## 설정 파일

### I2C 설정 (config/i2c_config.yaml)

```yaml
i2c:
  bus: 1
  timeout: 1.0
  retry_count: 3

devices:
  imu:
    address: 0x68
    name: "GY-BN008x IMU"
    enabled: true

  motor_driver:
    address: 0x40
    name: "PCA9685 Motor Driver"
    enabled: true
```

## 문제 해결 체크리스트

### 하드웨어 확인

- [ ] 전원 공급 확인
- [ ] 배선 연결 상태 확인
- [ ] I2C 주소 설정 확인
- [ ] 풀업 저항 확인

### 소프트웨어 확인

- [ ] I2C 드라이버 로드 확인
- [ ] 사용자 권한 확인
- [ ] Python 라이브러리 설치 확인
- [ ] 설정 파일 확인

### 통신 확인

- [ ] I2C 버스 스캔 결과 확인
- [ ] 장치별 통신 테스트
- [ ] 로그 메시지 확인
- [ ] 네트워크 연결 확인

## 추가 리소스

### 유용한 명령어

```bash
# I2C 도구 설치
sudo apt install i2c-tools

# I2C 버스 정보
i2cdetect -l

# 특정 장치 읽기
i2cget -y 1 0x68 0x75

# 특정 장치 쓰기
i2cset -y 1 0x68 0x6B 0x00
```

### 디버깅 스크립트

```python
# i2c_debug.py
import smbus2
import time

class I2CDebugger:
    def __init__(self, bus_number=1):
        self.bus = smbus2.SMBus(bus_number)

    def scan_devices(self):
        """모든 I2C 장치 스캔"""
        devices = []
        for address in range(128):
            try:
                self.bus.read_byte(address)
                devices.append(address)
            except:
                pass
        return devices

    def test_device(self, address):
        """특정 장치 테스트"""
        try:
            # 기본 읽기 테스트
            data = self.bus.read_byte(address)
            return True, f"장치 0x{address:02x} 연결 성공"
        except Exception as e:
            return False, f"장치 0x{address:02x} 연결 실패: {e}"

    def close(self):
        self.bus.close()

# 사용 예시
debugger = I2CDebugger()
devices = debugger.scan_devices()
print(f"감지된 장치: {[hex(d) for d in devices]}")

for device in [0x68, 0x40]:  # IMU, Motor Driver
    success, message = debugger.test_device(device)
    print(message)

debugger.close()
```

## 결론

I2C 문제는 대부분 하드웨어 연결이나 권한 설정과 관련이 있습니다. 체계적인 진단을 통해 대부분의 문제를 해결할 수 있습니다.
