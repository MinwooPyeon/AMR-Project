# AMR 시스템 설치 및 실행 가이드

## 1. 시스템 요구사항

### 필수 소프트웨어
- **Python 3.8+**
- **mosquitto MQTT 브로커**
- **Linux 시스템** (Ubuntu 20.04+ 권장)

### 하드웨어 요구사항
- **Jetson Orin Nano**
- **I2C 인터페이스** (센서 연결용)
- **GPIO 핀** (모터 제어용)

## 2. 설치 과정

### 2.1 시스템 패키지 설치

```bash
# 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# Python3 및 pip 설치
sudo apt install -y python3 python3-pip python3-venv

# mosquitto MQTT 브로커 설치
sudo apt install -y mosquitto mosquitto-clients

# 개발 도구 설치
sudo apt install -y build-essential cmake git curl wget
```

### 2.2 Python 의존성 설치

```bash
# 프로젝트 디렉토리로 이동
cd /path/to/your/amr/project

# 각 모듈의 의존성 설치
pip3 install -r sensors/requirements.txt
pip3 install -r mqtt_module/requirements.txt
# 기타 모듈 requirements.txt가 있다면 설치
```

### 2.3 실행 권한 설정

```bash
# 실행 스크립트에 권한 부여
chmod +x run_amr.sh
chmod +x process_manager.py
```

## 3. 실행 방법

### 3.1 수동 실행

```bash
# 의존성 확인
./run_amr.sh check

# 사용 가능한 모듈 목록 확인
./run_amr.sh list

# 시스템 시작
./run_amr.sh start

# 상태 확인
./run_amr.sh status

# 모니터링 모드로 시작 (자동 재시작)
./run_amr.sh monitor

# 시스템 중지
./run_amr.sh stop
```

### 3.2 systemd 서비스로 등록 

```bash
# 1. 서비스 파일 경로 수정
sudo nano systemd/amr-system.service
# WorkingDirectory와 ExecStart/ExecStop 경로를 실제 프로젝트 경로로 수정

# 2. 서비스 파일 복사
sudo cp systemd/amr-system.service /etc/systemd/system/

# 3. systemd 재로드
sudo systemctl daemon-reload

# 4. 서비스 활성화
sudo systemctl enable amr-system.service

# 5. 서비스 시작
sudo systemctl start amr-system.service

# 6. 서비스 상태 확인
sudo systemctl status amr-system.service
```

## 4. 로그 확인

### 4.1 시스템 로그
```bash
# 로그 파일 확인
tail -f logs/amr_system.log

# systemd 서비스 로그 (서비스로 등록한 경우)
sudo journalctl -u amr-system.service -f
```

### 4.2 개별 모듈 로그
```bash
# 프로세스 매니저 상태 확인
python3 process_manager.py status

# 특정 모듈 로그 확인
python3 process_manager.py start sensors  # 센서 모듈만 실행하여 로그 확인
```

## 5. 문제 해결

### 5.1 일반적인 문제들

#### mosquitto 브로커 연결 실패
```bash
# mosquitto 서비스 상태 확인
sudo systemctl status mosquitto

# mosquitto 서비스 시작
sudo systemctl start mosquitto

# 포트 확인
netstat -tlnp | grep 1883
```

#### Python 모듈 import 오류
```bash
# PYTHONPATH 설정 확인
echo $PYTHONPATH

# 의존성 재설치
pip3 install --force-reinstall -r sensors/requirements.txt
```

#### 권한 문제
```bash
# 디바이스 접근 권한 확인
ls -la /dev/i2c*
ls -la /dev/gpio*

# 사용자를 적절한 그룹에 추가
sudo usermod -a -G i2c,gpio $USER
```

### 5.2 디버깅 모드

```bash
# 상세 로그와 함께 실행
DEBUG=1 ./run_amr.sh start

# 개별 모듈 디버깅
python3 -u process_manager.py start sensors
```

## 6. 설정 파일

### 6.1 MQTT 설정 (`config/mosquitto.conf`)
```bash
# 기본 설정으로 충분하지만 필요시 수정
sudo nano config/mosquitto.conf
```

### 6.2 환경 변수 설정
```bash
# ~/.bashrc에 추가
export PYTHONPATH="/path/to/your/amr/project:$PYTHONPATH"
export MQTT_BROKER="localhost"
export MQTT_PORT="1883"
```

## 7. 성능 최적화

### 7.1 시스템 설정
```bash
# CPU 성능 모드 설정
sudo cpufreq-set -g performance

# 메모리 스왑 비활성화 (충분한 RAM이 있는 경우)
sudo swapoff -a
```

### 7.2 Python 최적화
```bash
# Python 최적화 플래그 설정
export PYTHONOPTIMIZE=1
export PYTHONUNBUFFERED=1
```

## 8. 백업 및 복구

### 8.1 설정 백업
```bash
# 설정 파일 백업
tar -czf amr_config_backup_$(date +%Y%m%d).tar.gz config/ logs/
```

### 8.2 시스템 복구
```bash
# 백업에서 복구
tar -xzf amr_config_backup_YYYYMMDD.tar.gz
```

## 9. 업데이트

### 9.1 코드 업데이트
```bash
# Git을 사용하는 경우
git pull origin main

# 의존성 재설치
pip3 install -r */requirements.txt

# 시스템 재시작
./run_amr.sh restart
```

## 10. 문제 확인

1. 로그 파일: `logs/amr_system.log`
2. 시스템 상태: `./run_amr.sh status`
3. 의존성 확인: `./run_amr.sh check`
