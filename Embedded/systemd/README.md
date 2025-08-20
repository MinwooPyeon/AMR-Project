# 시스템 서비스 설정

## 개요

AMR 시스템을 Linux 시스템 서비스로 등록하여 자동 시작 및 관리할 수 있도록 설정하는 모듈
systemd를 사용하여 AMR 시스템의 안정적인 운영을 보장

## 주요 기능

### 1. 자동 시작/중지
- **시스템 부팅 시 자동 시작**: AMR 시스템 자동 실행
- **서비스 중지**: 시스템 종료 시 안전한 중지
- **재시작 관리**: 오류 발생 시 자동 재시작

### 2. 서비스 모니터링
- **상태 모니터링**: 서비스 실행 상태 실시간 확인
- **로그 관리**: systemd journal을 통한 로그 수집
- **의존성 관리**: 다른 서비스와의 의존성 설정

### 3. 리소스 관리
- **메모리 제한**: 서비스별 메모리 사용량 제한
- **CPU 제한**: CPU 사용량 제한 및 우선순위 설정
- **네트워크 제한**: 네트워크 대역폭 제한

## 파일 구조

```
systemd/
├── README.md             
├── amr-system.service     # AMR 시스템 메인 서비스
├── amr-mqtt.service       # MQTT 브로커 서비스
├── amr-dashboard.service  # 웹 대시보드 서비스
└── install.sh             # 서비스 설치 스크립트
```

## 서비스 파일 설명

### 1. amr-system.service

AMR 시스템의 메인 서비스를 정의합니다:

```ini
[Unit]
Description=AMR Autonomous Mobile Robot System
After=network.target mqtt.service
Wants=mqtt.service

[Service]
Type=simple
User=amr
Group=amr
WorkingDirectory=/opt/amr
ExecStart=/usr/bin/python3 process_manager.py start-all
ExecStop=/usr/bin/python3 process_manager.py stop-all
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

# 리소스 제한
MemoryMax=2G
CPUQuota=200%

# 보안 설정
NoNewPrivileges=true
PrivateTmp=true
ProtectSystem=strict
ProtectHome=true

[Install]
WantedBy=multi-user.target
```

### 2. amr-mqtt.service

MQTT 브로커 서비스를 정의합니다:

```ini
[Unit]
Description=AMR MQTT Broker
After=network.target
Before=amr-system.service

[Service]
Type=simple
User=mosquitto
Group=mosquitto
ExecStart=/usr/sbin/mosquitto -c /etc/mosquitto/mosquitto.conf
Restart=always
RestartSec=5

# 보안 설정
NoNewPrivileges=true
PrivateTmp=true

[Install]
WantedBy=multi-user.target
```

### 3. amr-dashboard.service

웹 대시보드 서비스를 정의합니다:

```ini
[Unit]
Description=AMR Web Dashboard
After=network.target amr-system.service
Requires=amr-system.service

[Service]
Type=simple
User=amr
Group=amr
WorkingDirectory=/opt/amr/dashboard_module
ExecStart=/usr/bin/python3 web_dashboard.py
Restart=always
RestartSec=10

# 포트 설정
Environment=PORT=8000
Environment=HOST=0.0.0.0

[Install]
WantedBy=multi-user.target
```

## 설치 및 설정

### 1. 서비스 설치

```bash
# 서비스 파일 복사
sudo cp amr-system.service /etc/systemd/system/
sudo cp amr-mqtt.service /etc/systemd/system/
sudo cp amr-dashboard.service /etc/systemd/system/

# systemd 재로드
sudo systemctl daemon-reload

# 서비스 활성화
sudo systemctl enable amr-mqtt.service
sudo systemctl enable amr-system.service
sudo systemctl enable amr-dashboard.service
```

### 2. 자동 설치 스크립트

```bash
# 설치 스크립트 실행
chmod +x install.sh
sudo ./install.sh
```

### 3. 사용자 및 그룹 생성

```bash
# AMR 사용자 생성
sudo useradd -r -s /bin/false -d /opt/amr amr

# mosquitto 사용자 확인 (이미 존재할 수 있음)
sudo useradd -r -s /bin/false mosquitto 2>/dev/null || true

# 디렉토리 권한 설정
sudo mkdir -p /opt/amr
sudo chown -R amr:amr /opt/amr
sudo chmod 755 /opt/amr
```

## 서비스 관리

### 1. 서비스 시작/중지

```bash
# 서비스 시작
sudo systemctl start amr-system.service
sudo systemctl start amr-mqtt.service
sudo systemctl start amr-dashboard.service

# 서비스 중지
sudo systemctl stop amr-system.service
sudo systemctl stop amr-mqtt.service
sudo systemctl stop amr-dashboard.service

# 서비스 재시작
sudo systemctl restart amr-system.service
```

### 2. 서비스 상태 확인

```bash
# 서비스 상태 확인
sudo systemctl status amr-system.service
sudo systemctl status amr-mqtt.service
sudo systemctl status amr-dashboard.service

# 모든 AMR 관련 서비스 상태 확인
sudo systemctl status amr-*
```

### 3. 로그 확인

```bash
# 실시간 로그 확인
sudo journalctl -u amr-system.service -f
sudo journalctl -u amr-mqtt.service -f
sudo journalctl -u amr-dashboard.service -f

# 최근 로그 확인
sudo journalctl -u amr-system.service --since "1 hour ago"
```

## 설정 옵션

### 1. 환경 변수 설정

서비스별 환경 변수를 설정할 수 있습니다:

```bash
# 환경 변수 파일 생성
sudo mkdir -p /etc/amr
sudo tee /etc/amr/amr.env > /dev/null <<EOF
AMR_ENV=production
AMR_LOG_LEVEL=INFO
AMR_CONFIG_PATH=/opt/amr/config
AMR_DATA_PATH=/opt/amr/data
EOF

# 서비스 파일에 환경 변수 파일 추가
sudo systemctl edit amr-system.service
```

### 2. 리소스 제한 설정

```ini
# 메모리 제한
MemoryMax=2G
MemoryHigh=1.5G

# CPU 제한
CPUQuota=200%
CPUWeight=100

# 디스크 I/O 제한
IODeviceWeight=/dev/sda 100
```

### 3. 보안 설정

```ini
# 보안 강화 설정
NoNewPrivileges=true
PrivateTmp=true
PrivateDevices=true
ProtectSystem=strict
ProtectHome=true
ProtectKernelTunables=true
ProtectKernelModules=true
ProtectControlGroups=true
RestrictRealtime=true
RestrictSUIDSGID=true
```

## 모니터링 및 알림

### 1. 서비스 상태 모니터링

```bash
# 서비스 상태 스크립트
#!/bin/bash
SERVICES=("amr-system" "amr-mqtt" "amr-dashboard")

for service in "${SERVICES[@]}"; do
    if ! systemctl is-active --quiet "$service.service"; then
        echo "Service $service is not running!"
        # 알림 전송
        curl -X POST http://localhost:8000/api/alerts \
             -H "Content-Type: application/json" \
             -d "{\"type\": \"service_down\", \"service\": \"$service\"}"
    fi
done
```

### 2. 자동 복구 스크립트

```bash
#!/bin/bash
# 자동 복구 스크립트
SERVICE=$1

if ! systemctl is-active --quiet "$SERVICE.service"; then
    echo "Restarting $SERVICE service..."
    systemctl restart "$SERVICE.service"
    
    # 재시작 후 상태 확인
    sleep 10
    if systemctl is-active --quiet "$SERVICE.service"; then
        echo "$SERVICE service restarted successfully"
    else
        echo "Failed to restart $SERVICE service"
        # 관리자에게 알림
    fi
fi
```

## 문제 해결

### 1. 서비스 시작 실패

```bash
# 상세한 오류 정보 확인
sudo systemctl status amr-system.service
sudo journalctl -u amr-system.service -n 50

# 권한 문제 확인
sudo ls -la /opt/amr/
sudo id amr

# 의존성 확인
sudo systemctl list-dependencies amr-system.service
```

### 2. 포트 충돌

```bash
# 포트 사용 확인
sudo netstat -tlnp | grep :8000
sudo netstat -tlnp | grep :1883

# 충돌하는 프로세스 종료
sudo pkill -f "python.*web_dashboard"
sudo pkill -f mosquitto
```

### 3. 리소스 부족

```bash
# 시스템 리소스 확인
free -h
df -h
top

# 서비스별 리소스 사용량 확인
sudo systemctl show amr-system.service --property=MemoryCurrent,CPUUsageNSec
```

## 성능 최적화

### 1. 서비스 우선순위 설정

```ini
# 높은 우선순위 설정
Nice=-10
IOSchedulingClass=1
IOSchedulingPriority=4
```

### 2. 메모리 최적화

```ini
# 메모리 설정
MemoryMax=2G
MemoryHigh=1.5G
MemoryLow=1G
```

### 3. 네트워크 최적화

```ini
# 네트워크 설정
IPAddressAllow=192.168.1.0/24
IPAddressDeny=
```

## 백업 및 복구

### 1. 서비스 설정 백업

```bash
# 서비스 파일 백업
sudo cp /etc/systemd/system/amr-*.service /backup/systemd/

# 설정 파일 백업
sudo cp -r /opt/amr/config /backup/amr/
```

### 2. 서비스 복구

```bash
# 서비스 복구 스크립트
#!/bin/bash
BACKUP_DIR="/backup"

# 서비스 파일 복원
sudo cp $BACKUP_DIR/systemd/amr-*.service /etc/systemd/system/

# 설정 파일 복원
sudo cp -r $BACKUP_DIR/amr/config /opt/amr/

# systemd 재로드 및 서비스 재시작
sudo systemctl daemon-reload
sudo systemctl restart amr-*
```

## 보안 고려사항

### 1. 사용자 권한

- **최소 권한 원칙**: 서비스별 최소한의 권한만 부여
- **격리된 사용자**: 각 서비스별 독립적인 사용자 계정
- **권한 제한**: 시스템 리소스 접근 제한

### 2. 네트워크 보안

- **방화벽 설정**: 필요한 포트만 개방
- **네트워크 격리**: 내부 네트워크에서만 접근 허용
- **SSL/TLS**: 외부 통신 시 암호화 적용

### 3. 로그 보안

- **로그 암호화**: 중요 로그 파일 암호화
- **로그 회전**: 로그 파일 크기 및 보관 기간 제한
- **접근 제어**: 로그 파일 접근 권한 제한