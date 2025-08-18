# 백업 모듈

AMR(Autonomous Mobile Robot) 프로젝트 백업 기능

## 개요

- **BackupManager**: 파일/디렉토리 복사 및 ZIP 압축을 포함한 핵심 백업 기능
- **BackupConfig**: 백업 경로, 스케줄, 제외 항목을 위한 설정 관리
- **AutoBackup**: 자동화된 백업 스케줄링 및 실행

## 설치

### 의존성

```bash
pip install -r requirements.txt
```

## 사용법

### 수동 백업

```python
from backup import BackupManager

manager = BackupManager()

# 특정 경로의 백업 생성
source_paths = ["ai", "config", "motor_control"]
backup_path = manager.create_backup(source_paths, "my_backup")

# ZIP 백업 생성
zip_backup = manager.create_zip_backup(source_paths, "compressed_backup.zip")
```

### 자동화된 백업

```python
from backup import AutoBackup

auto_backup = AutoBackup()

# 수동 백업 생성
auto_backup.create_manual_backup("pre_deployment")

# 자동화된 백업 서비스 시작
auto_backup.run()
```

### 설정

```python
from backup import BackupConfig

# 백업 경로 가져오기
paths = BackupConfig.get_backup_paths()

# 스케줄 설정 가져오기
schedule = BackupConfig.get_schedule_config()

# 경로 검증
valid_paths = BackupConfig.validate_paths(paths)
```

## 기능

### 백업 유형

- **파일 백업**: 직접 파일 복사
- **디렉토리 백업**: 재귀적 디렉토리 복사
- **ZIP 백업**: 압축 아카이브 생성

### 관리

- **백업 목록**: 사용 가능한 모든 백업 보기
- **복원**: 백업에서 복원
- **삭제**: 오래된 백업 제거
- **정리**: 오래된 백업 자동 정리

## 설정

### 백업 경로

백업에 포함되는 기본 경로:

- `ai/` - AI 모듈
- `config/` - 설정 파일
- `motor_control/` - 모터 제어 모듈
- `mqtt/` - MQTT 통신
- `ros2/` - ROS2 통합
- `sensor_sync/` - 센서 동기화
- `src/` - 소스 코드
- `include/` - 헤더 파일
- `tests/` - 테스트 파일
- `utils/` - 유틸리티 함수

### 제외 패턴

백업에서 제외되는 파일 및 디렉토리:

- `__pycache__/`
- `*.pyc`, `*.pyo`, `*.pyd`
- `.git/`
- `.vscode/`
- `node_modules/`
- `*.log`, `*.tmp`, `*.temp`

## 테스트

```bash
# 백업 매니저 테스트
python -m backup.backup_manager

# 자동 백업 서비스 테스트
python -m backup.auto_backup
```
