#!/bin/bash

echo "=== AMR 센서 데이터 동기화 시스템 ==="
echo ""
echo "1. AI에서 수신 테스트"
echo "2. 백엔드와 양방향 통신 테스트"
echo "3. 모터 동작 테스트"
echo "4. 백업 시스템 테스트"
echo ""
echo "📡 데이터 구조:"
echo "  Embedded -> Backend:"
echo "    Topic: status/AMR001"
echo "    {"
echo '      "serial": "AMR001"'
echo '      "state": "RUNNING"'
echo '      "x": "10.0"'
echo '      "y": "10.0"'
echo '      "speed": "25.0"'
echo "    }"
echo ""
echo "  AI -> Embedded:"
echo "    Topic: /position"
echo "    {"
echo '      "MOVING_FORWARD": ""'
echo '      "ROTATE_LEFT": ""'
echo '      "ROTATE_RIGHT": ""'
echo '      "MOVING_BACKWARD": ""'
echo '      "STOP": ""'
echo '      "img": ".jpg"'
echo '      "situation": ""'
echo '      "x": "10.0"'
echo '      "y": "10.0"'
echo "    }"
echo ""
read -p "선택하세요 (1-4): " choice

case $choice in
    1)
        echo "AI에서 수신 테스트를 시작합니다..."
        python3 test_amr_with_ai_subscriber.py
        ;;
    2)
        echo "백엔드와 양방향 통신 테스트를 시작합니다..."
        python3 test_bidirectional_communication.py
        ;;
    3)
        echo "모터 동작 테스트를 시작합니다..."
        python3 amr_real_data_sync.py
        ;;
    4)
        echo "백업 시스템 테스트를 시작합니다..."
        python3 test_backup_system.py
        ;;
    *)
        echo "잘못된 선택입니다. 1-4 중에서 선택해주세요."
        ;;
esac 