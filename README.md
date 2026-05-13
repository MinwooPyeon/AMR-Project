# SARS (Safety AMR Service)

> 디지털 트윈 기반 자율주행 AMR + 실시간 안전 관리 시스템

물류 환경에서 AMR 자율주행·위험 감지·실시간 알림을 End-to-End로 통합한 프로젝트입니다.
임베디드(ROS2) – 시뮬레이터(Unity) – 백엔드(Spring Boot) – 안드로이드(FCM/RTSP)까지 하나의 파이프라인으로 연결합니다.

| 항목 | 내용 |
|------|------|
| 개발 기간 | 2025.07.07 ~ 2025.08.20 |
| 팀원 | 6명 (임베디드·시뮬레이션·AI·백엔드·모바일) |
| 주요 성과 | 가상 센서 처리 117ms → 3.5ms, AI→서버→모바일 실시간 알림 파이프라인 |

---

## 목차

- [기술 스택](#기술-스택)
- [주요 기능](#주요-기능)
- [시스템 아키텍처](#시스템-아키텍처)
- [성과 요약](#성과-요약)
- [문제 해결](#문제-해결)
- [팀원 소개](#팀원-소개)

---

## 기술 스택

| 분야 | 기술 |
|------|------|
| Embedded | Python, C++, ROS2, MQTT, OpenCV / Jetson Orin Nano |
| Simulation | Unity, C#, UGUI, M2Mqtt, Burst, Job System |
| AI | PyTorch, YOLO (Ultralytics), OpenCV, ROS2, MQTT |
| Backend | Spring Boot (Kotlin), JPA, PostgreSQL + TimescaleDB, Spring Security, JWT, Redis |
| Infra | Docker Compose, Jenkins CI/CD |
| Android | Jetpack Compose, Coroutine, Dagger Hilt, Retrofit2, Room, FCM, ExoPlayer |

---

## 주요 기능

### AMR 자율주행 & 통신
- ROS2 기반 SLAM·경로 계획·모터 제어 노드
- MQTT 양방향 통신으로 상태/명령 실시간 교환

### 실시간 위험 감지 & 알림
- YOLO 기반 위험 상황 탐지(안전 장비 미착용, 물류 붕괴, 흡연, 쓰러짐) → 서버 수신 → 모바일 푸시 알림

### 디지털 트윈 시뮬레이터
- SLAM 맵 연계 Occupancy Grid 시각화
- Burst + Job System으로 가상 센서 처리 **117ms → 3.5ms** 최적화
- 다중 AMR 충돌 회피 (리더 선점·경로 재탐색)

### 모바일 모니터링
- RTSP 기반 실시간 카메라 뷰 (ExoPlayer)
- FCM + Room + Flow로 알림 이력 동기화/재현성 확보

---

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│           AMR (ROS2, Jetson Orin Nano)                  │
│     YOLO 위험 감지 / 모터 제어 / 센서 퓨전 / MQTT       │
└───────────────────────┬─────────────────────────────────┘
                        │ MQTT (TLS)
┌───────────────────────▼─────────────────────────────────┐
│           Backend (Spring Boot, PostgreSQL, Redis)       │
│     REST API / MQTT 브로커 / FCM 발송 / 스케줄러         │
└───────┬───────────────────────────────┬─────────────────┘
        │ WebSocket / REST              │ FCM / REST
┌───────▼───────────┐       ┌───────────▼─────────────────┐
│  Unity Simulator  │       │  Android (FCM/RTSP/Room)    │
│  (Digital Twin)   │       │  실시간 모니터링 & 알림       │
└───────────────────┘       └─────────────────────────────┘
```

- Jenkins 컨테이너에서 호스트 Docker를 제어하기 위해 Docker socket 바인딩 구성 → CD 성공

---

## 성과 요약

| 분야 | 핵심 성과 |
|------|-----------|
| 시뮬레이션 | 가상 센서 처리 **117ms → 3.5ms** 최적화 |
| AI | mAP50-95 기준 **90% 이상** 감지 정확도 |
| 알림 파이프라인 | AI → 서버 → 모바일 실시간 알림 전달 체계 완성 |
| 데이터 동기화 | FCM + Room + Flow로 알림 이력 싱크 보장 |
| 서버 성능 | Redis 캐싱으로 AMR 상태 조회 고속화 |
| CI/CD | Jenkins 컨테이너 + 도커 소켓 바인딩으로 배포 자동화 |

---

## 문제 해결

| 문제 | 원인 | 해결 |
|------|------|------|
| 웹캠/RTSP 지연 | 실시간 스트림 처리 병목 | IPC + RTSP 혼합, 멀티프로세싱/큐 적용 |
| Unity 프레임 드랍 | 단일 스레드 CPU 병목 | Burst + Job System 병렬화 → **117ms → 3.5ms** |
| Jenkins in Docker CD 실패 | 컨테이너 내부 Docker 접근 제한 | Jenkins 이미지에 Docker 설치 + `/var/run/docker.sock` 바인딩 |
| 알림 이력 손실/불일치 | 서버–로컬 비동기 처리 | 서버 저장 + Room 업서트 + Flow 구독으로 UI 자동 반영 |

---

## 팀원 소개

| 이름 | 역할 | 주요 업무 |
|------|------|-----------|
| 박주현 | 시뮬레이터 | Unity 디지털 트윈, SLAM 시각화, 센서 최적화 |
| 정남진 | 백엔드 | Spring Boot, Redis, FCM, CI/CD |
| 편민우 | 임베디드 | ROS2 제어 및 센서 융합, MQTT |
| 홍민기 | AI | YOLO 탐지 파이프라인, Jetson 최적화 |
| 박상윤 | 안드로이드 | RTSP 스트리밍, 알림 동기화 및 UI |
| 손병하 | 안드로이드 | 대시보드, 공장 지도 UI, 하드웨어 점검 |

---

## 서브 모듈 README

- [AI](./AI/README.md) — YOLO 모델 학습, 위험 감지 파이프라인
- [Android](./Android/README.md) — Jetpack Compose, RTSP, FCM, Room
- [Embedded](./Embedded/README.md) — ROS2, 모터 제어, MQTT, IMU
