# 펌웨어 보안 관리 가이드

## 개요

AMR 시스템의 펌웨어와 익스텐션을 안전하게 관리하기 위한 보안 시스템입니다.
펌웨어 무결성 검증, 익스텐션 관리, 백업 및 롤백 기능을 제공합니다.

## 주요 기능

### 1. 펌웨어 무결성 검증
- SHA-256 해시 검증
- RSA 디지털 서명 검증
- 파일 크기 및 확장자 검증

### 2. 익스텐션 관리
- 익스텐션 설치/제거
- 활성화/비활성화
- 설정 수정
- 호환성 검증

### 3. 백업 및 롤백
- 자동 백업 생성
- 펌웨어 롤백 기능
- 백업 목록 관리

## 사용 방법

### 1. 펌웨어 보안 매니저 초기화

```python
from security.firmware_security import get_firmware_security_manager

# 펌웨어 보안 매니저 인스턴스 생성
firmware_manager = get_firmware_security_manager()
```

### 2. 펌웨어 설치

```python
from pathlib import Path

# 펌웨어 파일 경로
firmware_path = Path('new_firmware_v1.2.3.bin')

# 펌웨어 설치 (백업 포함)
success, message = firmware_manager.install_firmware(firmware_path, backup=True)

if success:
    print(f"펌웨어 설치 성공: {message}")
else:
    print(f"펌웨어 설치 실패: {message}")
```

### 3. 펌웨어 무결성 검증

```python
# 펌웨어 무결성 검증
is_valid, error_msg = firmware_manager.verify_firmware_integrity(firmware_path)

if is_valid:
    print("펌웨어 무결성 검증 성공")
else:
    print(f"펌웨어 무결성 검증 실패: {error_msg}")
```

### 4. 익스텐션 설치

```python
# 익스텐션 파일 경로
extension_path = Path('safety_extension_v2.1.py')

# 익스텐션 설치
success, message = firmware_manager.install_extension(extension_path)

if success:
    print(f"익스텐션 설치 성공: {message}")
else:
    print(f"익스텐션 설치 실패: {message}")
```

### 5. 익스텐션 관리

```python
# 익스텐션 활성화
firmware_manager.enable_extension('safety_extension')

# 익스텐션 비활성화
firmware_manager.disable_extension('safety_extension')

# 익스텐션 제거
firmware_manager.remove_extension('safety_extension')

# 익스텐션 상태 조회
status = firmware_manager.get_extension_status()
print(f"총 익스텐션 수: {status['total_extensions']}")
print(f"활성화된 익스텐션 수: {status['enabled_extensions']}")
```

## API 사용 방법

### 1. 펌웨어 상태 조회

```bash
curl -X GET http://localhost:5001/api/firmware/status
```

### 2. 펌웨어 설치

```bash
curl -X POST http://localhost:5001/api/firmware/install \
  -F "firmware=@new_firmware.bin" \
  -F "backup=true"
```

### 3. 펌웨어 무결성 검증

```bash
curl -X POST http://localhost:5001/api/firmware/verify \
  -F "firmware=@firmware.bin"
```

### 4. 익스텐션 설치

```bash
curl -X POST http://localhost:5001/api/extensions/install \
  -F "extension=@safety_extension.py"
```

### 5. 익스텐션 수정

```bash
curl -X POST http://localhost:5001/api/extensions/safety_extension/modify \
  -H "Content-Type: application/json" \
  -d '{
    "description": "Updated safety extension",
    "version": "2.2.0",
    "enabled": true
  }'
```

### 6. 익스텐션 활성화/비활성화

```bash
# 활성화
curl -X POST http://localhost:5001/api/extensions/safety_extension/enable

# 비활성화
curl -X POST http://localhost:5001/api/extensions/safety_extension/disable
```

### 7. 익스텐션 제거

```bash
curl -X DELETE http://localhost:5001/api/extensions/safety_extension
```

### 8. 익스텐션 설정 관리

```bash
# 설정 조회
curl -X GET http://localhost:5001/api/extensions/safety_extension/config

# 설정 업데이트
curl -X POST http://localhost:5001/api/extensions/safety_extension/config \
  -H "Content-Type: application/json" \
  -d '{
    "safety_threshold": 0.8,
    "auto_stop_enabled": true,
    "notification_level": "high"
  }'
```

### 9. 백업 목록 조회

```bash
curl -X GET http://localhost:5001/api/firmware/backup
```

### 10. 펌웨어 롤백

```bash
curl -X POST http://localhost:5001/api/firmware/rollback/1640995200
```

### 11. 로그 조회

```bash
curl -X GET "http://localhost:5001/api/firmware/logs?lines=50"
```

## 보안 기능

### 1. 디지털 서명

펌웨어 파일은 RSA 디지털 서명으로 보호됩니다:

```python
# 서명 파일 생성 (개발자용)
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding

def sign_firmware(firmware_path: Path, private_key):
    with open(firmware_path, 'rb') as f:
        firmware_data = f.read()
    
    signature = private_key.sign(
        firmware_data,
        padding.PSS(
            mgf=padding.MGF1(hashes.SHA256()),
            salt_length=padding.PSS.MAX_LENGTH
        ),
        hashes.SHA256()
    )
    
    signature_file = firmware_path.with_suffix(firmware_path.suffix + '.sig')
    with open(signature_file, 'wb') as f:
        f.write(signature)
```

### 2. 해시 검증

모든 펌웨어 파일은 SHA-256 해시로 검증됩니다:

```python
import hashlib

def create_firmware_hash(firmware_path: Path):
    with open(firmware_path, 'rb') as f:
        firmware_data = f.read()
    
    firmware_hash = hashlib.sha256(firmware_data).hexdigest()
    
    hash_file = firmware_path.with_suffix(firmware_path.suffix + '.hash')
    with open(hash_file, 'w') as f:
        f.write(firmware_hash)
```

### 3. 호환성 검증

익스텐션은 현재 펌웨어 버전과의 호환성을 검증합니다:

```json
{
  "name": "safety_extension",
  "version": "2.1.0",
  "compatibility": {
    "firmware_version": "1.2.0",
    "min_firmware_version": "1.0.0",
    "max_firmware_version": "2.0.0"
  }
}
```

## 설정 파일

### 펌웨어 설정

```json
{
  "firmware_dir": "firmware",
  "backup_dir": "firmware/backup",
  "extensions_dir": "firmware/extensions",
  "signature_required": true,
  "backup_required": true,
  "rollback_enabled": true,
  "max_firmware_size": 104857600,
  "allowed_extensions": [".bin", ".hex", ".elf", ".py", ".so"]
}
```

### 익스텐션 메타데이터

```json
{
  "name": "safety_extension",
  "version": "2.1.0",
  "type": "py",
  "description": "Safety monitoring extension for AMR",
  "author": "AMR Team",
  "compatibility": {
    "firmware_version": "1.2.0"
  },
  "dependencies": ["numpy", "opencv-python"],
  "permissions": ["sensor_access", "motor_control"]
}
```

## 모니터링 및 로깅

### 로그 파일 위치

- 펌웨어 보안 로그: `logs/firmware_security.log`
- 시스템 로그: `logs/security.log`

### 로그 레벨

- INFO: 일반적인 작업 로그
- WARNING: 경고 메시지
- ERROR: 오류 메시지
- DEBUG: 디버그 정보

### 모니터링 지표

- 펌웨어 설치 성공률
- 익스텐션 활성화 상태
- 백업 생성 빈도
- 무결성 검증 실패 횟수

## 문제 해결

### 1. 펌웨어 설치 실패

**문제**: "Firmware signature verification failed"

**해결 방법**:
1. 서명 파일(.sig)이 있는지 확인
2. 서명 키가 올바른지 확인
3. 펌웨어 파일이 손상되지 않았는지 확인

### 2. 익스텐션 호환성 오류

**문제**: "Extension compatibility check failed"

**해결 방법**:
1. 현재 펌웨어 버전 확인
2. 익스텐션 호환성 정보 확인
3. 호환되는 익스텐션 버전 설치

### 3. 백업 생성 실패

**문제**: "Failed to create firmware backup"

**해결 방법**:
1. 디스크 공간 확인
2. 백업 디렉토리 권한 확인
3. 현재 펌웨어 파일 존재 여부 확인

## 보안 권장사항

### 1. 키 관리
- 개인키는 안전한 위치에 보관
- 정기적인 키 교체
- 키 백업 및 복구 계획 수립

### 2. 접근 제어
- 펌웨어 관리 권한 제한
- API 접근 인증 강화
- 로그 모니터링

### 3. 정기 점검
- 펌웨어 무결성 정기 검증
- 백업 파일 정기 점검
- 보안 로그 분석

## API 응답 형식

### 성공 응답

```json
{
  "success": true,
  "message": "Operation completed successfully",
  "data": {
    // 응답 데이터
  }
}
```

### 오류 응답

```json
{
  "success": false,
  "error": "Error description"
}
```

## 지원되는 파일 형식

### 펌웨어 파일
- `.bin`: 바이너리 펌웨어
- `.hex`: Intel HEX 형식
- `.elf`: ELF 실행 파일

### 익스텐션 파일
- `.py`: Python 스크립트
- `.so`: 공유 라이브러리
- `.bin`: 바이너리 익스텐션

이 가이드를 통해 AMR 시스템의 펌웨어와 익스텐션을 안전하게 관리할 수 있습니다.
