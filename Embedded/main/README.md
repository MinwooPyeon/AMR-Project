# 메인 시스템 모듈

AMR 시스템의 메인 통합 모듈들입니다.

## 파일 구조

- `amr_real_data_sync.py`: AMR 메인 시스템 통합 파일

## 주요 기능

- 모든 모듈 통합 및 조율
- 시스템 전체 상태 관리
- 백업 시스템 관리
- AI 상황 기반 기능 트리거

## 사용법

```python
from main import AMRRealDataSync

# AMR 시스템 생성
amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True, enable_backup=True)

# 동기화 시작
amr_sync.start_sync()

# 시스템 상태 확인
status = amr_sync.get_system_status()
```

## 실행 방법

```bash
# 메인 시스템 실행
python main/amr_real_data_sync.py
``` 