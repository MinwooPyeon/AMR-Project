# 🚀 SARS (Safety AMR Service)

> **디지털 트윈 기반 자율주행 AMR + 실시간 안전 관리 시스템**  
물류 환경에서 **AMR 자율주행·위험 감지·실시간 알림**을 End-to-End로 통합한 프로젝트입니다.  
임베디드(ROS2)–시뮬레이터(Unity)–백엔드(Spring Boot)–안드로이드(FCM/RTSP)까지 하나의 파이프라인으로 유기적으로 연결합니다.  

**기간:** 2025.07.07~2025.08.20 / **팀:** 6명 (임베디드·시뮬레이션·AI·백엔드·모바일)

---

## 🧱 기술 스택

- **Embedded / Hardware**: Python, C++, **ROS2**, MQTT, OpenCV / Jetson Orin Nano, 모터 드라이버, 센서 모듈  
- **Simulation (Unity)**: Unity, C#, UGUI, **M2Mqtt**, Burst, Job System  
- **AI**: PyTorch, **YOLO(Ultralytics)**, OpenCV, ROS2, MQTT  
- **Backend & Infra**: Spring Boot(Kotlin), JPA, **PostgreSQL + Timescale**, Spring Security, JWT, **Redis**, **Docker/Compose**, Jenkins CI/CD  
- **Android**: Jetpack Compose, Coroutine, Dagger Hilt, Retrofit2, **Room**, **FCM**, **ExoPlayer**

---

## 🧩 주요 기능

### 1) AMR 자율주행 & 통신
- ROS2 기반 **SLAM·경로 계획·모터 제어** 노드
- MQTT 양방향 통신으로 상태/명령 실시간 교환  

### 2) 실시간 위험 감지 & 알림
- YOLO 기반 **위험 상황 탐지** → 서버 수신 → 모바일 **푸시 알림** 도달

### 3) 디지털 트윈 시뮬레이터
- SLAM 맵 연계 **Occupancy Grid 시각화**
- Burst + Job System으로 가상 센서 처리 **117ms → 3.5ms** 최적화  
- 다중 AMR **충돌 회피(리더 선점·경로 재탐색)** 로직 적용

### 4) 모바일 모니터링
- RTSP 기반 **실시간 카메라 뷰**
- **FCM + Room + Flow**로 **알림 이력 동기화/재현성** 확보

---

## 🏗 시스템 아키텍처

```
AMR (ROS2, Jetson) ←MQTT→ Backend (Spring, PostgreSQL, Redis)
│                                │
└──── RTSP/Telemetry ────────────┘
│
Unity Simulator (Digital Twin)
│
Android (FCM/RTSP/Room)
```

- Jenkins 컨테이너 내에서 호스트 Docker를 제어하기 위해 **Docker socket 바인딩** 구성 → **CD 성공**

---

## 📌 성과 요약

| 분야 | 핵심 성과 |
|---|---|
| 시뮬레이션 | 가상 센서 처리 **117ms → 3.5ms** 최적화 |
| 알림 파이프라인 | AI→서버→모바일 실시간 알림 전달 체계 완성 |
| 데이터 동기화 | FCM + Room + Flow로 **알림 이력 싱크** 보장 |
| 서버 성능 | Redis 캐싱으로 **AMR 상태 조회** 고속화 |
| CI/CD | Jenkins 컨테이너 + 도커 소켓 바인딩으로 **배포 자동화** |

---

## 👥 팀 & 역할

| 이름 | 역할 | 주요 업무 |
|------|------|-----------|
| 박주현 | 시뮬레이터 | Unity 기반 디지털 트윈, SLAM 시각화, 센서 최적화 |
| 정남진 | 백엔드 | Spring Boot, Redis, FCM, CI/CD 구축 |
| 편민우 | 임베디드 | ROS2 기반 제어 및 센서 융합, MQTT 통신 |
| 홍민기 | AI | YOLO 탐지 파이프라인, Jetson 최적화 |
| 박상윤 | 안드로이드 | RTSP 스트리밍, 알림 동기화 및 UI 개발 |
| 손병하 | 안드로이드 | 대시보드, 공장 지도 UI 개발. 하드웨어 점검. |


