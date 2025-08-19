# AMR 시스템 보안 가이드

## 개요

AMR(Autonomous Mobile Robot) 시스템의 보안 설정 및 관리 방법,
시스템의 무결성, 기밀성, 가용성을 보장하기 위한 종합적인 보안 솔루션

## 보안 위험 요소

### 1. 네트워크 보안 위험
- **무인 접근**: MQTT 브로커의 익명 접근 허용
- **암호화 부재**: 평문 통신으로 인한 데이터 노출
- **포트 스캔**: 공격자의 시스템 탐색
- **DDoS 공격**: 서비스 거부 공격

### 2. 인증 및 권한 위험
- **약한 비밀번호**: 예측 가능한 비밀번호 사용
- **세션 관리 부재**: 무제한 세션 유지
- **권한 상승**: 과도한 권한 부여

### 3. 데이터 보안 위험
- **평문 저장**: 민감한 데이터의 암호화 부재
- **무결성 검증 부재**: 데이터 변조 감지 불가
- **백업 보안**: 백업 데이터의 보호 부족

### 4. 시스템 보안 위험
- **프로세스 격리 부재**: 모듈 간 권한 분리 없음
- **리소스 제한 부재**: 무제한 리소스 사용
- **로깅 부족**: 보안 이벤트 추적 불가

## 보안 솔루션

### 1. 중앙 보안 관리 시스템

#### SecurityManager 클래스
```python
from security.security_manager import get_security_manager

# 보안 매니저 초기화
security_manager = get_security_manager()

# 사용자 인증
authenticated = security_manager.authenticate_user(
    username="admin", 
    password="admin123!", 
    client_ip="192.168.1.100"
)

# 데이터 암호화
encrypted_data = security_manager.encrypt_data("sensitive data")
decrypted_data = security_manager.decrypt_data(encrypted_data)

# 속도 제한 확인
allowed = security_manager.check_rate_limit("192.168.1.100")
```

#### 주요 기능
- **사용자 인증**: 로그인 시도 제한, 세션 관리
- **데이터 암호화**: AES-256 암호화로 민감한 데이터 보호
- **속도 제한**: DDoS 공격 방지를 위한 요청 제한
- **파일 무결성**: 중요 파일의 해시값 검증
- **보안 모니터링**: 실시간 보안 이벤트 감지

### 2. 보안 MQTT 통신

#### SecureMQTTClient 클래스
```python
from security.secure_mqtt_client import SecureMQTTClient

# 보안 MQTT 클라이언트 생성
client = SecureMQTTClient(
    client_id="amr_system",
    username="admin",
    password="admin123!"
)

# SSL/TLS 연결
if client.connect():
    # 토픽 구독
    client.subscribe("status/AMR001", callback=message_handler)
    
    # 암호화된 메시지 발행
    client.publish("command/AMR001", json.dumps({"action": "STOP"}))
```

#### 보안 기능
- **SSL/TLS 암호화**: 모든 통신 암호화
- **클라이언트 인증**: 인증서 기반 인증
- **토픽 검증**: 허용된 토픽 패턴만 사용
- **메시지 검증**: 명령 메시지의 유효성 검사
- **연결 모니터링**: 연결 상태 실시간 감시

### 3. 네트워크 보안

#### 방화벽 설정
```bash
# 방화벽 규칙 적용
sudo chmod +x security/firewall_rules.sh
sudo ./security/firewall_rules.sh
```

#### 방화벽 규칙
- **기본 정책**: 모든 연결 거부 (기본값)
- **허용 포트**: SSH(22), MQTT(1883), HTTP(8000), WebSocket(8080)
- **속도 제한**: SSH 4회/분, MQTT 100회/분
- **위험 포트 차단**: Telnet, FTP, SMTP 등
- **로그 기록**: 모든 차단된 패킷 로깅

### 4. 시스템 보안

#### 프로세스 격리
```bash
# 사용자 생성 스크립트 실행
sudo chmod +x security/setup_users.sh
sudo ./security/setup_users.sh
```

#### 리소스 제한
```bash
# 시스템 제한 설정 적용
sudo cp security/amr_limits.conf /etc/security/limits.d/
```

#### 파일 권한 설정
```bash
# 중요 파일 권한 설정
sudo chmod 600 security/encryption.key
sudo chmod 600 config/system_config.py
sudo chmod 600 logs/security.log
```

### 5. MQTT 브로커 보안

#### 인증 설정
```bash
# 비밀번호 해시 생성
mosquitto_passwd -c security/mqtt_passwords.txt admin
mosquitto_passwd -b security/mqtt_passwords.txt operator op123!
mosquitto_passwd -b security/mqtt_passwords.txt viewer view123!
```

#### 접근 제어
```bash
# ACL 파일 설정
# security/mqtt_acls.txt 파일 참조
```

## 보안 설정 단계

### 1단계: 기본 보안 설정
```bash
# 1. 보안 디렉토리 생성
mkdir -p security/certs
mkdir -p security/logs

# 2. 보안 패키지 설치
pip install -r security/requirements.txt

# 3. 방화벽 설정
sudo ./security/firewall_rules.sh

# 4. 사용자 생성
sudo ./security/setup_users.sh
```

### 2단계: 인증서 생성
```bash
# 1. CA 인증서 생성
openssl req -new -x509 -days 365 -extensions v3_ca \
    -keyout security/certs/ca.key -out security/certs/ca.crt

# 2. 서버 인증서 생성
openssl req -new -out security/certs/server.csr \
    -keyout security/certs/server.key

# 3. 서버 인증서 서명
openssl x509 -req -in security/certs/server.csr \
    -CA security/certs/ca.crt -CAkey security/certs/ca.key \
    -CAcreateserial -out security/certs/server.crt -days 365

# 4. 클라이언트 인증서 생성
openssl req -new -out security/certs/client.csr \
    -keyout security/certs/client.key

# 5. 클라이언트 인증서 서명
openssl x509 -req -in security/certs/client.csr \
    -CA security/certs/ca.crt -CAkey security/certs/ca.key \
    -CAcreateserial -out security/certs/client.crt -days 365
```

### 3단계: MQTT 보안 설정
```bash
# 1. MQTT 비밀번호 설정
mosquitto_passwd -c security/mqtt_passwords.txt admin

# 2. MQTT 설정 업데이트
# config/mosquitto.conf 파일에 보안 설정 추가됨

# 3. MQTT 브로커 재시작
sudo systemctl restart mosquitto
```

### 4단계: 시스템 통합
```python
# 1. 보안 매니저 초기화
from security.security_manager import get_security_manager
security_manager = get_security_manager()

# 2. 보안 MQTT 클라이언트 사용
from security.secure_mqtt_client import SecureMQTTClient
client = SecureMQTTClient("amr_system", "admin", "admin123!")

# 3. 보안 콜백 등록
def security_alert_handler(alert):
    print(f"Security Alert: {alert['type']} - {alert['message']}")

security_manager.add_security_callback(security_alert_handler)
```

## 보안 모니터링

### 1. 실시간 모니터링
```python
# 보안 상태 확인
status = security_manager.get_security_status()
print(f"Failed login attempts: {status['failed_login_attempts']}")
print(f"Active sessions: {status['active_sessions']}")
print(f"Security alerts: {status['security_alerts']}")

# 보안 경고 확인
alerts = security_manager.get_security_alerts()
for alert in alerts:
    print(f"Alert: {alert['type']} - {alert['message']}")
```

### 2. 로그 모니터링
```bash
# 보안 로그 확인
tail -f logs/security.log

# 방화벽 로그 확인
sudo tail -f /var/log/iptables.log

# MQTT 로그 확인
sudo tail -f /var/log/mosquitto/mosquitto.log
```

### 3. 시스템 상태 확인
```bash
# 프로세스 상태 확인
ps aux | grep amr

# 네트워크 연결 확인
netstat -tulpn | grep :1883

# 파일 권한 확인
ls -la security/
ls -la config/
```

## 보안 정책

### 1. 비밀번호 정책
- **최소 길이**: 8자 이상
- **특수 문자**: 최소 1개 포함
- **대소문자**: 대문자, 소문자 포함
- **숫자**: 최소 1개 포함
- **변경 주기**: 90일마다 변경

### 2. 세션 정책
- **세션 타임아웃**: 1시간
- **동시 세션**: 최대 3개
- **IP 제한**: 허용된 IP에서만 접근

### 3. 접근 제어 정책
- **최소 권한 원칙**: 필요한 최소 권한만 부여
- **역할 기반 접근**: 사용자 역할에 따른 권한 분리
- **감사 로그**: 모든 접근 시도 기록

### 4. 암호화 정책
- **전송 암호화**: SSL/TLS 필수
- **저장 암호화**: 민감한 데이터 AES-256 암호화
- **키 관리**: 암호화 키의 안전한 관리

## 보안 점검 체크리스트

### 정기 점검 항목
- [ ] 보안 로그 검토
- [ ] 파일 무결성 검사
- [ ] 사용자 계정 검토
- [ ] 네트워크 연결 상태 확인
- [ ] 인증서 만료일 확인
- [ ] 보안 패치 적용 상태 확인

### 월간 점검 항목
- [ ] 보안 정책 검토
- [ ] 접근 권한 검토
- [ ] 백업 데이터 보안 검사
- [ ] 보안 설정 변경 사항 검토

### 분기별 점검 항목
- [ ] 보안 아키텍처 검토
- [ ] 침입 탐지 시스템 점검
- [ ] 재해 복구 계획 검토
- [ ] 보안 교육 실시

## 문제 해결

### 일반적인 문제들

#### 1. MQTT 연결 실패
```bash
# SSL 인증서 확인
openssl s_client -connect localhost:8883 -showcerts

# MQTT 로그 확인
sudo tail -f /var/log/mosquitto/mosquitto.log
```

#### 2. 인증 실패
```bash
# 비밀번호 파일 확인
cat security/mqtt_passwords.txt

# ACL 파일 확인
cat security/mqtt_acls.txt
```

#### 3. 방화벽 문제
```bash
# 방화벽 규칙 확인
sudo iptables -L -n -v

# 특정 포트 확인
sudo netstat -tulpn | grep :1883
```

#### 4. 파일 권한 문제
```bash
# 파일 권한 확인
ls -la security/
ls -la config/

# 권한 수정
sudo chmod 600 security/encryption.key
```

## 지원

1. **보안 로그 확인**: `logs/security.log`
2. **시스템 로그 확인**: `/var/log/syslog`
3. **MQTT 로그 확인**: `/var/log/mosquitto/mosquitto.log`
4. **방화벽 로그 확인**: `/var/log/iptables.log`

## 참고 자료

- [MQTT 보안 가이드](https://mosquitto.org/documentation/security/)
- [Linux 보안 가이드](https://wiki.archlinux.org/title/Security)
- [Python 암호화 가이드](https://cryptography.io/en/latest/)
- [SSL/TLS 설정 가이드](https://ssl-config.mozilla.org/)
