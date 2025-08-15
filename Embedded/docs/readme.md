# AMR 프로젝트 문서

AMR(Autonomous Mobile Robot) 프로젝트의 문서 모음

## 문서 목록

### 프로젝트 개요

- **[project_overview.md](project_overview.md)**: 프로젝트 전체 구조 및 기능 개요
- **[api_reference.md](api_reference.md)**: 모든 모듈의 API 참조 문서
- **[development_guide.md](development_guide.md)**: 개발 가이드 및 코딩 표준

### 하드웨어 가이드

- **[readme_imu_ai_motor.md](readme_imu_ai_motor.md)**: IMU 센서 및 모터 통합 시스템 가이드
- **[i2c_troubleshooting_guide.md](i2c_troubleshooting_guide.md)**: I2C 통신 문제 해결 가이드

### 통신 가이드

- **[mqtt_communication_guide.md](mqtt_communication_guide.md)**: MQTT 통신 가이드

### 빌드 시스템

- **[CMakeLists.txt](CMakeLists.txt)**: CMake 빌드 설정 파일

## 빠른 시작

### 1. 프로젝트 개요 확인

먼저 [project_overview.md](project_overview.md)를 읽어 프로젝트의 전체적인 구조와 기능을 파악하세요.

### 2. 개발 환경 설정

[development_guide.md](development_guide.md)의 "Development Environment Setup" 섹션을 따라 개발 환경을 구성하세요.

### 3. 하드웨어 연결

- IMU 센서 및 모터 제어: [readme_imu_ai_motor.md](readme_imu_ai_motor.md)

### 4. API 참조

개발 시 [api_reference.md](api_reference.md)를 참조하여 각 모듈의 API를 확인하세요.

## 문서 작성 가이드

### 새로운 문서 추가 시

1. **파일명**: 영어로 작성하고 snake_case 사용
2. **제목**: 한글로 작성
3. **내용**:
   - 개요 및 목적
   - 설치 및 설정 방법
   - 사용 예제
   - 문제 해결 방법
   - 참고 자료

### 문서 구조

```markdown
# 문서 제목

## 개요

문서의 목적과 범위 설명

## 설치

설치 및 설정 방법

## 사용법

구체적인 사용 예제

## 문제 해결

자주 발생하는 문제와 해결 방법

## 참고 자료

관련 링크 및 추가 정보
```

## 문서 업데이트

### 업데이트

- **주요 기능 추가 시**: 관련 문서 즉시 업데이트
- **API 변경 시**: API 참조 문서 업데이트
- **버그 수정 시**: 문제 해결 가이드 업데이트
- **분기별**: 전체 문서 검토 및 개선

### 버전 관리

- 문서 변경 시 Git 커밋 메시지에 `docs:` 접두사 사용
- 주요 변경사항은 CHANGELOG.md에 기록

## 문제 해결

### 문서 관련 문제

- **링크 오류**: 파일 경로 확인
- **이미지 누락**: 이미지 파일 경로 확인
- **코드 예제 오류**: 최신 코드와 동기화 확인

### 기술적 문제

- **하드웨어 연결**: [i2c_troubleshooting_guide.md](i2c_troubleshooting_guide.md) 참조
- **빌드 오류**: [CMakeLists.txt](CMakeLists.txt) 설정 확인
- **통신 문제**: [mqtt_communication_guide.md](mqtt_communication_guide.md) 참조

## 기여하기

### 문서 개선 제안

1. 이슈 생성으로 개선 사항 제안
2. 풀 리퀘스트로 직접 수정 제안
3. 오타나 오류 발견 시 즉시 수정

### 문서 작성 규칙

- **명확성**: 이해하기 쉽게 작성
- **완성성**: 필요한 모든 정보 포함
- **일관성**: 다른 문서와 스타일 통일
- **최신성**: 코드와 동기화 유지
