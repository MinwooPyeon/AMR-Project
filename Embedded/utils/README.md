# 유틸리티 모듈

AMR 시스템의 유틸리티 및 테스트 도구들을 담당하는 모듈들입니다.

## 파일 구조

- `logger.py`: 통일된 로그 시스템
- `organize_files.py`: 파일 정리 도구
- `run_sensor_sync.sh`: 센서 동기화 실행 스크립트
- `test_bidirectional_communication.py`: 양방향 통신 테스트

## 주요 기능

- 통일된 로그 시스템 (텍스트 기반 구분)
- MQTT 송신/수신 성공 로그 및 콜백
- 파일 정리 및 관리
- 시스템 실행 스크립트
- 테스트 도구
- 유틸리티 함수들

## 사용법

```bash
# 센서 동기화 실행
./utils/run_sensor_sync.sh

# 파일 정리
python utils/organize_files.py

# 양방향 통신 테스트
python utils/test_bidirectional_communication.py

# 로거 사용 예시
from utils.logger import mqtt_logger
mqtt_logger.info("정보 메시지")  # [INFO] 정보 메시지
mqtt_logger.warn("경고 메시지")  # [WARN] 경고 메시지
mqtt_logger.error("오류 메시지")  # [ERROR] 오류 메시지
mqtt_logger.mqtt_send_success("topic", {"data": "value"})  # [MQTT_SEND] MQTT 송신 성공: topic | 데이터: {'data': 'value'}
``` 