# SSAMR Android Application

SSAMR 프로젝트의 공식 안드로이드 애플리케이션입니다. 이 앱은 최신 안드로이드 기술 스택을 활용하여 사용자에게 실시간 스트리밍, 데이터 시각화 등 다양한 기능을 제공합니다.

## 🚀 주요 기능

- **실시간 AMR 데이터 처리**: AMR(원격 검침) 데이터를 실시간으로 처리하고 표시합니다.
- **실시간 영상 스트리밍**: RTSP 프로토콜을 지원하여 실시간으로 영상을 스트리밍합니다.
- **데이터 시각화**: 차트 라이브러리를 통해 데이터를 시각적으로 표현합니다.
- **푸시 알림**: Firebase Cloud Messaging (FCM)을 통한 실시간 알림 기능을 제공합니다.
- **로컬 데이터 저장**: Room 데이터베이스를 사용하여 앱의 주요 데이터를 로컬에 저장하고 관리합니다.
- **현대적인 UI**: Jetpack Compose를 사용하여 선언적이고 반응형 UI를 구현했습니다.

## 🛠️ 기술 스택 및 아키텍처

- **언어**: 100% Kotlin
- **UI**: Jetpack Compose, Material3
- **아키텍처**: MVI (Orbit MVI-lite) + Clean Architecture
- **비동기 처리**: Coroutines
- **의존성 주입**: Hilt
- **네트워킹**: Retrofit2 & OkHttp3
- **JSON 파싱**: Kotlinx Serialization
- **데이터베이스**: Room
- **미디어 재생**: Media3 (ExoPlayer)
- **이미지 로딩**: Coil
- **푸시 알림**: Firebase Cloud Messaging

## 📂 프로젝트 구조

```
.
└── app/src/main/java/com/android/ssamr/
    ├── core/               # 앱의 핵심 로직 및 유틸리티
    │   ├── common/         # 공통 상수, 헬퍼 클래스
    │   ├── data/           # Repository, DataSource 등 데이터 계층
    │   ├── domain/         # UseCase 등 도메인 로직
    │   ├── fcm/            # Firebase Cloud Messaging 서비스
    │   ├── network/        # Retrofit, OkHttp 등 네트워크 설정
    │   ├── permission/     # 권한 처리 관련 로직
    │   └── ui/             # 공통 UI 상태 관련 클래스
    │
    ├── feature/            # 기능별 모듈
    │   ├── amr/            # AMR 목록 조회
    │   ├── amrDetail/      # AMR 상세 정보
    │   ├── amrWebcam/      # AMR 웹캠 영상
    │   ├── dashboard/      # 대시보드
    │   ├── more/           # 더보기
    │   ├── notification/   # 알림 목록
    │   ├── notificationDetail/ # 알림 상세
    │   ├── report/         # 리포트 목록
    │   └── reportDetail/   # 리포트 상세
    │
    ├── main/               # 앱의 메인 진입점 (MainActivity, Application)
    │
    └── ui/                 # 전역 UI 요소
        └── theme/          # 앱 테마, 색상, 타이포그래피
```

## ⚙️ 설치 및 빌드 방법

1.  **저장소 복제**
    ```bash
    git clone [저장소 URL]
    ```

2.  **Firebase 설정**
    - Firebase 콘솔에서 `google-services.json` 파일을 다운로드합니다.
    - 다운로드한 파일을 `app/` 디렉터리에 위치시킵니다.

3.  **API Key 설정**
    - 프로젝트의 루트 디렉터리에 `local.properties` 파일을 생성합니다.
    - 파일에 아래와 같이 `BASE_URL`을 추가합니다.
      ```properties
      BASE_URL="로컬 ip 주소"
      ```

4.  **프로젝트 빌드**
    - Android Studio에서 프로젝트를 열고 Gradle 동기화를 진행합니다.
    - 빌드가 완료되면 앱을 실행할 수 있습니다.
