#!/bin/bash

# GY-BN008x IMU-모터 통합 시스템 빌드 스크립트

echo "=== GY-BN008x IMU-모터 통합 시스템 빌드 시작 ==="

# 현재 디렉토리 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "스크립트 디렉토리: $SCRIPT_DIR"

# 빌드 디렉토리 생성
BUILD_DIR="$SCRIPT_DIR/build"
echo "빌드 디렉토리: $BUILD_DIR"

if [ ! -d "$BUILD_DIR" ]; then
    echo "빌드 디렉토리 생성 중..."
    mkdir -p "$BUILD_DIR"
fi

# 빌드 디렉토리로 이동
cd "$BUILD_DIR"

# CMake 설정
echo "CMake 설정 중..."
cmake .. -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "오류: CMake 설정 실패"
    exit 1
fi

# 빌드 실행
echo "빌드 실행 중..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "오류: 빌드 실패"
    exit 1
fi

# 실행 파일 확인
if [ -f "imu_motor_test" ]; then
    echo "✓ imu_motor_test 빌드 성공"
    echo "실행 파일 크기: $(ls -lh imu_motor_test | awk '{print $5}')"
else
    echo "✗ imu_motor_test 빌드 실패"
    exit 1
fi

if [ -f "amr_main" ]; then
    echo "✓ amr_main 빌드 성공"
    echo "실행 파일 크기: $(ls -lh amr_main | awk '{print $5}')"
else
    echo "✗ amr_main 빌드 실패"
    exit 1
fi

echo ""
echo "=== 빌드 완료 ==="
echo "실행 방법:"
echo "  cd $BUILD_DIR"
echo "  ./imu_motor_test"
echo ""
echo "또는 Python 테스트 스크립트 사용:"
echo "  cd $SCRIPT_DIR"
echo "  python3 test_imu_motor_simple.py"
echo ""

# 권한 설정
chmod +x imu_motor_test
chmod +x amr_main

echo "실행 권한 설정 완료"
echo "빌드 완료!"
