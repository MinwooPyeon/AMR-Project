echo "=== AMR Sensor Data Synchronization System ==="
echo ""
echo "1. AI Reception Test"
echo "2. Backend Bidirectional Communication Test"
echo "3. Motor Operation Test"
echo "4. Backup System Test"
echo ""
echo "Data Structure:"
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
read -p "Select (1-4): " choice

case $choice in
    1)
        echo "Starting AI reception test..."
        python3 test_amr_with_ai_subscriber.py
        ;;
    2)
        echo "Starting backend bidirectional communication test..."
        python3 test_bidirectional_communication.py
        ;;
    3)
        echo "Starting motor operation test..."
        python3 amr_real_data_sync.py
        ;;
    4)
        echo "Starting backup system test..."
        python3 test_backup_system.py
        ;;
    *)
        echo "Invalid selection. Please choose 1-4."
        ;;
esac 