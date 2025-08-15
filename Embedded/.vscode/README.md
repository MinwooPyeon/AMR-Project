# VS Code 설정 파일

임베디드 AMR(Autonomous Mobile Robot) 프로젝트 VS Code 설정 파일

## 파일 설명

### 1. settings.json

- **파일 연관 설정**: C++, Python, ROS2, YAML 파일들의 언어 모드 설정
- **Python 설정**: 인터프리터 경로, 린팅, 포매팅 설정
- **C++ 설정**: 컴파일러, 표준, IntelliSense 설정
- **ROS2 설정**: ROS2 Humble 경로 설정
- **CMake 설정**: 빌드 디렉토리, 생성기 설정
- **터미널 설정**: 환경 변수 설정

### 2. extensions.json

VS Code 확장 프로그램

- **Python**: Python, Black, Flake8, isort, Pylint
- **C++**: C/C++ Tools, CMake Tools
- **ROS2**: ROS, ROS2 확장
- **기타**: GitLens, Material Icon Theme, YAML

### 3. tasks.json

빌드 및 테스트 작업

- **CMake Configure/Build**: C++ 프로젝트 빌드
- **Python Tests**: 테스트 실행
- **Code Formatting**: Black, isort를 이용한 코드 포매팅
- **ROS2 Launch**: ROS2 런치 파일 실행

### 4. launch.json

디버깅 설정

- **C++ 디버깅**: GDB를 이용한 C++ 프로그램 디버깅
- **Python 디버깅**: AI 서버, 모터 제어, MQTT 클라이언트 디버깅
- **원격 디버깅**: 원격 프로세스에 연결

### 5. c_cpp_properties.json

C++ IntelliSense 설정

- **Linux/Windows 환경**: 각 OS별 컴파일러 및 경로 설정
- **Include 경로**: 프로젝트 헤더 및 시스템 헤더 경로
- **컴파일러 플래그**: 디버그 정보, 경고 설정

## 사용법

### 1. 확장 프로그램 설치

VS Code를 열면 자동으로 권장 확장 프로그램 설치 팝업이 나타납니다.

### 2. 빌드 및 실행

- **빌드**: `Ctrl+Shift+P` → "Tasks: Run Task" → "CMake Build"
- **테스트**: `Ctrl+Shift+P` → "Tasks: Run Task" → 원하는 테스트 선택
- **디버깅**: `F5`를 눌러 디버깅 시작

### 3. 코드 포매팅

- **Python**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Format Python Code"
- **Import 정렬**: `Ctrl+Shift+P` → "Tasks: Run Task" → "Sort Python Imports"

### 4. 터미널 사용

- `Ctrl+`` (백틱)을 눌러 통합 터미널 열기
- ROS2 환경이 자동으로 설정됩니다

## 환경 설정

### Python 가상환경

```bash
# 가상환경 생성
python -m venv venv

# 가상환경 활성화 (Linux)
source venv/bin/activate

# 가상환경 활성화 (Windows)
venv\Scripts\activate

# 의존성 설치
pip install -r ai/requirements.txt
```

### ROS2 설정

```bash
# ROS2 환경 소스
source /opt/ros/humble/setup.bash

# 워크스페이스 빌드
colcon build
```

## 문제 해결

### 1. IntelliSense 오류

- `Ctrl+Shift+P` → "C/C++: Edit Configurations (UI)"에서 경로 확인
- 컴파일러 경로가 올바른지 확인

### 2. Python 인터프리터 오류

- `Ctrl+Shift+P` → "Python: Select Interpreter"에서 올바른 인터프리터 선택
- 가상환경이 활성화되어 있는지 확인

### 3. ROS2 명령어 인식 안됨

- 터미널에서 ROS2 환경이 소스되었는지 확인
- `echo $ROS_DISTRO`로 환경 변수 확인

## 추가 설정

### 키보드 단축키

- `Ctrl+Shift+P`: 명령 팔레트
- `F5`: 디버깅 시작
- `Ctrl+F5`: 디버깅 없이 실행
- `Ctrl+Shift+B`: 빌드 작업 실행

### 유용한 확장 프로그램

- **Auto Rename Tag**: HTML/XML 태그 자동 변경
- **Bracket Pair Colorizer**: 괄호 색상 구분
- **GitLens**: Git 히스토리 및 변경사항 표시
- **Path Intellisense**: 파일 경로 자동완성
- **Thunder Client**: REST API 테스트

## 참고 사항

- 이 설정들은 Linux 환경(Jetson)을 기준으로 작성되었습니다.
- Windows 환경에서는 일부 경로를 수정해야 할 수 있습니다.
- ROS2 Humble이 설치되어 있어야 합니다.
- Python 3.8 이상이 필요합니다.
