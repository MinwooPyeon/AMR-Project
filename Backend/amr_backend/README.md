# SARS(Safety AMR System)백엔드 시스템

## 📖 프로젝트 소개

본 프로젝트는 AMR의 상태를 실시간으로 모니터링하고, 원격으로 제어하며, 이상 상황 발생 시 알림을 전송하는 백엔드 시스템입니다. MQTT를 통해 AMR과 통신하고, 실시간 상태를 저장하여 필요시 클라이언트에서
요청할 수 있도록 합니다. 또한, FCM을 이용하여 모바일 앱으로 푸시 알림을 보낼 수 있습니다.

## ✨ 주요 기능

* **AMR 상태 조회:** 모든 AMR 최신 상태 조회 및 개별 AMR 상세 정보 조회
* **실시간 모니터링:** MQTT를 통해 AMR의 위치, 속도, 상태 등 실시간 데이터 수신 후 데이터베이스에 저장 및 Redis caching
* **원격 제어:** AMR에 수동 제어 명령 전송
* **이상 상황 알림:** AMR에서 AI를 통해 발생한 위험 상황(예: 사람 쓰러짐)을 감지하고, FCM 푸시 알림을 통해 Android App에 알림 전송
* **알림 관리:** 알림 목록 조회 및 읽음 처리 기능
* **사용자 인증:** JWT 기반의 안전한 사용자 인증 및 인가

## 🛠️ 기술 스택

### Backend

- **Kotlin**
- **Spring Boot 3**
- **Spring Security** (JWT)
- **Spring Data JPA**
- **Spring Integration** (MQTT)

### Database

- **PostgreSQL w/ Timescale Extension**
- **Redis** (AMR 최신 정보 캐싱)

### DevOps

- **Docker**
- **Docker Compose**

### Messaging

- **Eclipse Mosquitto** (MQTT Broker)

### Notification

- **Firebase Cloud Messaging (FCM)**

### API Documentation

- **SpringDoc (Swagger UI)**

## 📁 프로젝트 구조

```
.
├── build.gradle.kts              # Gradle 빌드 스크립트
├── docker-compose.yaml           # 개발환경용 Docker Compose 설정
├── docker-compose.prov.yaml      # Production용 Docker Compose 설정
├── Dockerfile                    # Spring Boot 애플리케이션 Dockerfile
├── settings.gradle.kts           # Gradle 설정
├── env                           # 환경변수 설정 파일
├── mosquitto                     # Mosquitto MQTT 브로커 설정
├── sql                           # DB 초기화 스크립트
└── src                           # 소스 코드
    ├── main
    │   ├── kotlin
    │   │   └── com/example/amr_backend
    │   │       ├── AmrBackendApplication.kt  # Spring Boot Entrypoint
    │   │       ├── v1                        # API v1
    │   │       │   ├── config                # 설정 (JPA, Redis, Swagger)
    │   │       │   ├── controller            # API 컨트롤러
    │   │       │   ├── dto                   # 데이터 전송 객체
    │   │       │   ├── entity                # JPA 엔티티
    │   │       │   ├── exception             # 예외 처리
    │   │       │   ├── fcm                   # Firebase Cloud Messaging 관련 로직
    │   │       │   ├── listener              # 이벤트 리스너
    │   │       │   ├── mqtt                  # MQTT 메시지 핸들러 및 설정
    │   │       │   ├── repository            # 데이터베이스 레포지토리
    │   │       │   └── service               # 비즈니스 로직
    │   │       └── v2                        # API v2 (인증 기능 추가)
    │   │           ├── config                # Security 설정
    │   │           ├── controller            # 인증 관련 컨트롤러
    │   │           ├── dto                   # 인증 관련 DTO
    │   │           ├── entity                # 인증 관련 엔티티
    │   │           ├── filter                # JWT 필터
    │   │           ├── model                 # 모델 클래스
    │   │           ├── repository            # 인증 관련 레포지토리
    │   │           └── service               # 인증 관련 서비스
    │   └── resources
    │       └── application.yaml  # Spring Boot 설정 파일
    └── test                      # 테스트 코드
```

## 🚀 시작하기

### 1. 전제 조건

* Java 17
* Docker
* Docker Compose

### 2. 환경변수 설정

`env` 디렉토리 아래에 다음 `.env` 파일들을 생성하고 내용을 채웁니다.

- 개발환경용 env
    - `postgres.dev.env`: 개발환경용 PostgreSQL 데이터베이스 설정
    - `spring.dev.env`: Spring Boot 애플리케이션 설정 (로컬 실행용)
    - `spring-docker.dev.env`: Spring Boot 애플리케이션 설정 (Docker 실행용)
- Production용 env
    - `postgres.prod.env`: Production용 PostgreSQL 데이터베이스 설정
    - `spring.prod.env`: Production용 Spring Boot 애플리케이션 설정 (Docker 실행용)

**spring.dev.env 예시:**

```properties
POSTGRES_USERNAME=postgres
POSTGRES_PASSWORD=ssafy
POSTGRES_URL=jdbc:postgresql://localhost:5432/amr
FIREBASE_KEY_LOCATION=./firebase/sars-50307-firebase-adminsdk-fbsvc-36586deaa8.json
MQTT_URL=tcp://localhost:1883
REDIS_HOST=localhost
REDIS_PORT=6379
JWT_SECRET_KEY=a-string-secret-at-least-256-bits-long
JWT_ACCESS_TOKEN_EXPIRY_IN_MINUTES=15
JWT_REFRESH_TOKEN_EXPIRY_IN_MINUTES=43200
```

Docker 실행용 env 파일에서는 URL들을 컨테이너의 URL로 변경해주어야 합니다.

### 3. Firebase Admin SDK 키 파일

`firebase` 디렉토리에 Firebase Admin SDK 키 파일(`your-firebase-adminsdk.json`)을 추가합니다.

### 4. 실행

#### Helper Script 사용

프로젝트 루트 디렉토리에서 bash로 `run_on_local.sh`를 실행하여 자동으로 프로젝트 빌드 후 Docker Compose로 필요한 컨테이너 및 서버 컨테이너를 실행시킵니다.

```bash
./run_on_local.sh
```

#### Docker Compose 사용

수동으로 프로젝트 빌드 후 프로젝트 루트 디렉토리에서 다음 명령어를 실행합니다.

```bash
docker-compose up --build
```

#### 로컬에서 직접 실행

1. PostgreSQL, Redis, Mosquitto를 직접 설치하고 실행합니다.
2. IntelliJ IDEA 또는 터미널에서 Spring Boot 애플리케이션을 실행합니다.

```bash
./gradlew bootRun
```

## 📝 API 문서

애플리케이션 실행 후, 다음 URL에서 API 문서를 확인할 수 있습니다.

- **Swagger UI:** [http://localhost:8080/swagger-ui/index.html](http://localhost:8080/swagger-ui/index.html)

## 🗄️ 데이터베이스 스키마

주요 테이블은 다음과 같습니다.

* `amr`: AMR의 기본 정보를 저장합니다.
* `amr_status`: AMR의 시계열 상태 데이터를 저장합니다.
* `notification`: 발생한 알림 정보를 저장합니다.
* `fcm_token`: 클라이언트의 FCM 토큰을 저장합니다.
* `_user`: 사용자 정보를 저장합니다.
* `refresh_token`: JWT 리프레시 토큰을 저장합니다.

자세한 내용은 `sql/init.sql` 파일을 참고하세요.
