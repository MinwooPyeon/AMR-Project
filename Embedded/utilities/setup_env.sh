#!/bin/bash
# Python 환경 설정 스크립트

# 프로젝트 루트 디렉토리 설정
export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Python 경로에 프로젝트 루트 추가
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH}"

echo "Python 환경 설정 완료:"
echo "  PROJECT_ROOT: $PROJECT_ROOT"
echo "  PYTHONPATH: $PYTHONPATH"

# 테스트 실행 예제
echo ""
echo "테스트 실행 방법:"
echo "  python tests/test_connection_setup.py"
echo "  python tests/angle_control_test.py"
echo "  python run_tests.py --test connection"
echo "  python run_tests.py --test angle" 