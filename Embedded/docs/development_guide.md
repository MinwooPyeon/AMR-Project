# 개발 가이드

## 개발 환경 설정

### 필수 요구사항

- **Python 3.7+**: 주요 개발 언어
- **C++17**: 성능이 중요한 모듈용
- **ROS2 Humble**: 로봇 운영 시스템
- **CMake 3.10+**: 빌드 시스템
- **Git**: 버전 관리

### 필요한 소프트웨어

#### Ubuntu 20.04+ 환경

```bash
# 시스템 업데이트
sudo apt update && sudo apt upgrade

# 기본 개발 도구
sudo apt install build-essential cmake git python3-pip

# Python 개발 도구
sudo apt install python3-dev python3-venv

# ROS2 Humble 설치
sudo apt install ros-humble-desktop

# 추가 의존성
sudo apt install libopencv-dev libeigen3-dev libboost-all-dev
```

#### Windows 환경

```bash
# WSL2 설치 후 Ubuntu 환경 사용 권장
# 또는 Windows용 Python 3.7+ 설치
```

### IDE 설정

#### VS Code 설정

```json
{
  "python.defaultInterpreterPath": "./venv/bin/python",
  "python.linting.enabled": true,
  "python.linting.pylintEnabled": true,
  "python.formatting.provider": "black",
  "editor.formatOnSave": true,
  "files.associations": {
    "*.yaml": "yaml",
    "*.yml": "yaml"
  }
}
```

#### PyCharm 설정

- Python 인터프리터를 가상환경으로 설정
- 코드 스타일을 PEP 8로 설정
- 자동 import 정렬 활성화

## 프로젝트 구조

### 디렉토리 구성

```
Embedded/
├── .vscode/                    # VS Code 설정
├── ai_module/                  # AI 관련 모듈
├── backup_module/              # 백업 시스템
├── communication_module/       # 통신 모듈
├── config/                     # 설정 파일들
├── display_module/             # 디스플레이 모듈
├── docs/                       # 문서
├── include/                    # C++ 헤더 파일들
├── launch/                     # ROS2 런치 파일들
├── motor_driver/               # 모터 드라이버 코드
├── motors/                     # 모터 제어 모듈
├── mqtt_module/                # MQTT 통신 모듈
├── ros2_module/                # ROS2 통합 모듈
├── sensors/                    # 센서 동기화 모듈
├── src/                        # C++ 소스 파일들
├── tests/                      # 테스트 파일들
└── utilities/                  # 유틸리티 스크립트
```

### 코드 구성 원칙

1. **모듈화**: 각 기능을 독립적인 모듈로 분리
2. **설정 분리**: 코드와 설정을 분리하여 관리
3. **의존성 최소화**: 모듈 간 의존성을 최소화
4. **테스트 가능성**: 모든 코드는 테스트 가능하도록 설계
5. **문서화**: 모든 공개 API에 대한 문서화

## 코딩 표준

### Python 코딩 표준

#### PEP 8 준수

```python
# 좋은 예
def calculate_distance(point1, point2):
    """두 점 사이의 거리를 계산합니다."""
    return math.sqrt((point2.x - point1.x) ** 2 + (point2.y - point1.y) ** 2)

# 나쁜 예
def calcDist(p1,p2):
    return math.sqrt((p2.x-p1.x)**2+(p2.y-p1.y)**2)
```

#### 타입 힌트 사용

```python
from typing import Optional, List, Dict, Any

def process_sensor_data(data: Dict[str, Any]) -> Optional[float]:
    """센서 데이터를 처리합니다."""
    if not data:
        return None
    return data.get('value', 0.0)
```

#### 예외 처리

```python
try:
    result = risky_operation()
except SpecificException as e:
    logger.error(f"특정 오류 발생: {e}")
    raise
except Exception as e:
    logger.error(f"예상치 못한 오류: {e}")
    raise
```

### C++ 코딩 표준

#### Google C++ Style Guide 준수

```cpp
// 좋은 예
class MotorController {
 public:
  explicit MotorController(int motor_id);
  bool SetSpeed(double speed);

 private:
  int motor_id_;
  double current_speed_;
};

// 나쁜 예
class motorController {
public:
    motorController(int motorId);
    bool setSpeed(double Speed);
private:
    int motorId;
    double currentSpeed;
};
```

#### RAII 원칙 준수

```cpp
class ResourceManager {
 public:
  ResourceManager() : resource_(AcquireResource()) {}
  ~ResourceManager() { ReleaseResource(resource_); }

  // 복사 생성자와 할당 연산자 삭제
  ResourceManager(const ResourceManager&) = delete;
  ResourceManager& operator=(const ResourceManager&) = delete;

 private:
  Resource* resource_;
};
```

### 일반 가이드라인

#### 네이밍 컨벤션

- **snake_case**: 변수, 함수, 파일명
- **PascalCase**: 클래스명
- **UPPER_SNAKE_CASE**: 상수
- **camelCase**: 사용하지 않음

#### 주석 작성

```python
def complex_algorithm(data: List[float]) -> float:
    """
    복잡한 알고리즘을 수행합니다.

    Args:
        data: 처리할 데이터 리스트

    Returns:
        계산된 결과값

    Raises:
        ValueError: 데이터가 비어있는 경우
    """
    if not data:
        raise ValueError("데이터가 비어있습니다")

    # 1단계: 데이터 정규화
    normalized_data = [x / max(data) for x in data]

    # 2단계: 가중 평균 계산
    result = sum(normalized_data) / len(normalized_data)

    return result
```

## 모듈 개발

### 새 모듈 생성

#### 1. 디렉토리 구조 생성

```bash
mkdir new_module
cd new_module
touch __init__.py
touch module_name.py
touch config.py
touch requirements.txt
touch readme.md
```

#### 2. 기본 클래스 구조

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class ModuleConfig:
    """모듈 설정 클래스"""
    enabled: bool = True
    debug: bool = False

class NewModule:
    """새 모듈 클래스"""

    def __init__(self, config: ModuleConfig):
        self.config = config
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

    def initialize(self) -> bool:
        """모듈을 초기화합니다."""
        try:
            self.logger.info("모듈 초기화 시작")
            # 초기화 로직
            return True
        except Exception as e:
            self.logger.error(f"초기화 실패: {e}")
            return False

    def cleanup(self):
        """리소스를 정리합니다."""
        self.logger.info("모듈 정리 완료")
```

#### 3. 설정 파일 생성

```yaml
# new_module_config.yaml
new_module:
  enabled: true
  debug: false
  parameters:
    timeout: 30
    retry_count: 3
```

### 모듈 통합

#### 1. 메인 애플리케이션에 통합

```python
from new_module import NewModule, ModuleConfig

def main():
    config = ModuleConfig(enabled=True, debug=False)
    module = NewModule(config)

    if module.initialize():
        # 모듈 사용
        pass
    else:
        logger.error("모듈 초기화 실패")
```

#### 2. 의존성 관리

```python
# requirements.txt
requests>=2.25.0
paho-mqtt>=1.6.1
numpy>=1.19.0
```

## 테스트

### 단위 테스트

#### Python 테스트

```python
# test_new_module.py
import unittest
from unittest.mock import Mock, patch
from new_module import NewModule, ModuleConfig

class TestNewModule(unittest.TestCase):

    def setUp(self):
        self.config = ModuleConfig(enabled=True, debug=False)
        self.module = NewModule(self.config)

    def test_initialization(self):
        """초기화 테스트"""
        result = self.module.initialize()
        self.assertTrue(result)

    def test_cleanup(self):
        """정리 테스트"""
        self.module.initialize()
        self.module.cleanup()
        # 정리 후 상태 확인

    @patch('new_module.some_external_dependency')
    def test_with_mock(self, mock_dependency):
        """모의 객체를 사용한 테스트"""
        mock_dependency.return_value = "mocked_result"
        result = self.module.some_method()
        self.assertEqual(result, "mocked_result")

if __name__ == '__main__':
    unittest.main()
```

#### C++ 테스트

```cpp
// test_new_module.cpp
#include <gtest/gtest.h>
#include "new_module.h"

class NewModuleTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.enabled = true;
    config_.debug = false;
    module_ = std::make_unique<NewModule>(config_);
  }

  ModuleConfig config_;
  std::unique_ptr<NewModule> module_;
};

TEST_F(NewModuleTest, Initialization) {
  EXPECT_TRUE(module_->Initialize());
}

TEST_F(NewModuleTest, Cleanup) {
  ASSERT_TRUE(module_->Initialize());
  module_->Cleanup();
  // 정리 후 상태 확인
}
```

### 통합 테스트

#### 모듈 간 상호작용 테스트

```python
# test_integration.py
import unittest
from ai_module import AIAlertPublisher
from communication_module import CommunicationManager
from config import ConfigManager

class TestIntegration(unittest.TestCase):

    def setUp(self):
        self.config_manager = ConfigManager()
        self.comm_manager = CommunicationManager()
        self.ai_publisher = AIAlertPublisher()

    def test_ai_to_communication_flow(self):
        """AI에서 통신 모듈로의 데이터 흐름 테스트"""
        # AI 알림 생성
        alert_data = self.ai_publisher.create_alert("warning", "테스트 알림")

        # 통신 모듈을 통한 전송
        result = self.comm_manager.send_json_message("mqtt", alert_data)

        self.assertTrue(result)
```

### 테스트 실행

#### Python 테스트 실행

```bash
# 모든 테스트 실행
python -m pytest tests/

# 특정 테스트 실행
python -m pytest tests/test_new_module.py::TestNewModule::test_initialization

# 커버리지와 함께 실행
python -m pytest --cov=src tests/
```

#### C++ 테스트 실행

```bash
# 빌드 및 테스트 실행
mkdir build && cd build
cmake ..
make
ctest --verbose
```

## 문서화

### 코드 문서화

#### Python Docstring

```python
def process_data(data: List[float], threshold: float = 0.5) -> List[float]:
    """
    데이터를 처리하여 임계값 이상의 값만 반환합니다.

    이 함수는 입력 데이터에서 지정된 임계값보다 큰 값들만
    필터링하여 반환합니다.

    Args:
        data: 처리할 데이터 리스트
        threshold: 필터링 임계값 (기본값: 0.5)

    Returns:
        임계값 이상의 데이터 리스트

    Raises:
        ValueError: 데이터가 비어있는 경우
        TypeError: 데이터 타입이 올바르지 않은 경우

    Example:
        >>> process_data([0.1, 0.6, 0.3, 0.8], 0.5)
        [0.6, 0.8]
    """
    if not data:
        raise ValueError("데이터가 비어있습니다")

    if not all(isinstance(x, (int, float)) for x in data):
        raise TypeError("모든 데이터는 숫자여야 합니다")

    return [x for x in data if x >= threshold]
```

#### C++ 문서화

````cpp
/**
 * @brief 데이터를 처리하여 임계값 이상의 값만 반환합니다.
 *
 * 이 함수는 입력 데이터에서 지정된 임계값보다 큰 값들만
 * 필터링하여 반환합니다.
 *
 * @param data 처리할 데이터 벡터
 * @param threshold 필터링 임계값 (기본값: 0.5)
 * @return 임계값 이상의 데이터 벡터
 * @throws std::invalid_argument 데이터가 비어있는 경우
 *
 * @example
 * ```cpp
 * std::vector<double> data = {0.1, 0.6, 0.3, 0.8};
 * auto result = ProcessData(data, 0.5);
 * // result = {0.6, 0.8}
 * ```
 */
std::vector<double> ProcessData(const std::vector<double>& data,
                               double threshold = 0.5);
````

### API 문서화

#### Sphinx 사용

```python
# conf.py
project = 'AMR Project'
copyright = '2024, S13P11D103'
author = 'S13P11D103 Team'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
]

# docs/index.rst
Welcome to AMR Project's documentation!
=======================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   modules/ai_module
   modules/communication_module
   modules/config_module
   modules/display_module
   modules/backup_module

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
```

## 버전 관리

### Git 워크플로우

#### 브랜치 전략

```bash
# 메인 브랜치
main          # 프로덕션 릴리스
develop       # 개발 통합 브랜치

# 기능 브랜치
feature/ai-integration     # AI 통합 기능
feature/motor-control      # 모터 제어 기능
feature/communication      # 통신 기능

# 수정 브랜치
hotfix/critical-bug        # 긴급 버그 수정
hotfix/security-patch      # 보안 패치
```

#### 커밋 메시지 규칙

```bash
# 형식: <type>(<scope>): <description>

# 예시
feat(ai): AI 알림 발행 기능 추가
fix(motor): 모터 속도 제어 버그 수정
docs(api): API 문서 업데이트
test(communication): 통신 모듈 테스트 추가
refactor(config): 설정 관리 코드 리팩토링
style(display): 디스플레이 모듈 코드 스타일 수정
```

### 릴리스 관리

#### 버전 번호 규칙

- **Major.Minor.Patch** 형식 사용
- **Major**: 호환되지 않는 변경사항
- **Minor**: 호환되는 새로운 기능
- **Patch**: 호환되는 버그 수정

#### 릴리스 태그

```bash
# 태그 생성
git tag -a v1.0.0 -m "첫 번째 안정 릴리스"
git tag -a v1.1.0 -m "AI 통합 기능 추가"
git tag -a v1.1.1 -m "모터 제어 버그 수정"

# 태그 푸시
git push origin --tags
```

## 성능 최적화

### Python 성능 최적화

#### 프로파일링

```python
import cProfile
import pstats

def profile_function():
    profiler = cProfile.Profile()
    profiler.enable()

    # 프로파일링할 코드
    result = expensive_function()

    profiler.disable()
    stats = pstats.Stats(profiler)
    stats.sort_stats('cumulative')
    stats.print_stats(10)

    return result
```

#### 메모리 최적화

```python
# 제너레이터 사용
def process_large_file(filename):
    with open(filename, 'r') as f:
        for line in f:
            yield process_line(line)

# 리스트 컴프리헨션 대신 제너레이터 표현식
# 나쁜 예
large_list = [x * 2 for x in range(1000000)]

# 좋은 예
large_generator = (x * 2 for x in range(1000000))
```

### C++ 성능 최적화

#### 컴파일러 최적화

```cmake
# CMakeLists.txt
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -Wall -Wextra")
```

#### 메모리 관리

```cpp
// 스마트 포인터 사용
std::unique_ptr<Resource> resource = std::make_unique<Resource>();

// 이동 의미론 활용
std::vector<Data> process_data(std::vector<Data>&& data) {
    // 이동 생성자 사용
    return std::move(data);
}
```

## 디버깅

### Python 디버깅

#### 로깅 설정

```python
import logging

# 로깅 설정
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('app.log'),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)
```

#### 디버거 사용

```python
import pdb

def debug_function():
    x = 10
    y = 20

    pdb.set_trace()  # 디버거 중단점

    result = x + y
    return result
```

### C++ 디버깅

#### GDB 사용

```bash
# 디버그 빌드
g++ -g -O0 -o program program.cpp

# GDB 실행
gdb program

# GDB 명령어
(gdb) break main
(gdb) run
(gdb) next
(gdb) print variable_name
(gdb) continue
```

#### Valgrind 메모리 검사

```bash
# 메모리 누수 검사
valgrind --leak-check=full ./program

# 스레드 오류 검사
valgrind --tool=helgrind ./program
```

## 배포

### 패키징

#### Python 패키지

```python
# setup.py
from setuptools import setup, find_packages

setup(
    name="amr-project",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        "requests>=2.25.0",
        "paho-mqtt>=1.6.1",
        "numpy>=1.19.0",
    ],
    author="S13P11D103 Team",
    description="AMR 프로젝트",
    python_requires=">=3.7",
)
```

#### Docker 컨테이너

```dockerfile
# Dockerfile
FROM python:3.9-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .

CMD ["python", "main.py"]
```

### 배포 스크립트

#### 자동화 스크립트

```bash
#!/bin/bash
# deploy.sh

set -e

echo "배포 시작..."

# 테스트 실행
echo "테스트 실행 중..."
python -m pytest tests/

# 빌드
echo "빌드 중..."
python setup.py build

# 배포
echo "배포 중..."
python setup.py install

echo "배포 완료!"
```

## 기여 가이드라인

### 기여 프로세스

1. **이슈 생성**: 버그 리포트 또는 기능 요청
2. **브랜치 생성**: `feature/issue-description` 형식
3. **개발**: 코드 작성 및 테스트
4. **테스트**: 모든 테스트 통과 확인
5. **문서화**: API 문서 및 README 업데이트
6. **풀 리퀘스트**: 코드 리뷰 요청
7. **병합**: 승인 후 메인 브랜치 병합

### 코드 리뷰 체크리스트

- [ ] 코드가 코딩 표준을 준수하는가?
- [ ] 적절한 테스트가 포함되어 있는가?
- [ ] 문서화가 충분한가?
- [ ] 성능에 영향을 주지 않는가?
- [ ] 보안 취약점이 없는가?
- [ ] 기존 기능을 깨뜨리지 않는가?

### 커뮤니케이션

- **이슈 템플릿** 사용
- **명확한 제목** 작성
- **상세한 설명** 제공
- **재현 단계** 명시
- **예상 동작** 설명
