import sys
import os

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

def run_ai_communication_test():
    print("Running AI communication test...")
    try:
        from tests.ai_communication_test import main
        main()
    except Exception as e:
        print(f"Error occurred during test execution: {e}")

def run_angle_control_test():
    print("Running angle control test...")
    try:
        from tests.angle_control_test import main
        main()
    except Exception as e:
        print(f"Error occurred during test execution: {e}")

def run_imu_sensor_test():
    print("Running IMU sensor test...")
    try:
        from tests.imu_sensor_test import main
        main()
    except Exception as e:
        print(f"Error occurred during test execution: {e}")

def run_mqtt_test():
    print("Running MQTT test...")
    try:
        from tests.mqtt_test import main
        main()
    except Exception as e:
        print(f"Error occurred during test execution: {e}")

def run_obstacle_detection_test():
    print("Running obstacle detection test...")
    try:
        from tests.obstacle_detection_test import main
        main()
    except Exception as e:
        print(f"Error occurred during test execution: {e}")

def run_rotation_detection_test():
    print("Running rotation detection test...")
    try:
        from tests.rotation_detection_test import main
        main()
    except Exception as e:
        print(f"Error occurred during test execution: {e}")

def run_all_tests():
    print("Running all tests...")
    tests = [
        run_ai_communication_test,
        run_angle_control_test,
        run_imu_sensor_test,
        run_mqtt_test,
        run_obstacle_detection_test,
        run_rotation_detection_test
    ]
    
    for test in tests:
        try:
            test()
            print("-" * 50)
        except Exception as e:
            print(f"Error occurred during test execution: {e}")
            print("-" * 50)

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test execution script")
    parser.add_argument("--test", choices=["ai", "angle", "imu", "mqtt", "obstacle", "rotation", "all"], 
                       help="Select test to run")
    
    args = parser.parse_args()
    
    if args.test == "ai":
        run_ai_communication_test()
    elif args.test == "angle":
        run_angle_control_test()
    elif args.test == "imu":
        run_imu_sensor_test()
    elif args.test == "mqtt":
        run_mqtt_test()
    elif args.test == "obstacle":
        run_obstacle_detection_test()
    elif args.test == "rotation":
        run_rotation_detection_test()
    elif args.test == "all":
        run_all_tests()
    else:
        print("Usage:")
        print("  python run_tests.py --test ai        # AI communication test")
        print("  python run_tests.py --test angle     # Angle control test")
        print("  python run_tests.py --test imu       # IMU sensor test")
        print("  python run_tests.py --test mqtt      # MQTT test")
        print("  python run_tests.py --test obstacle  # Obstacle detection test")
        print("  python run_tests.py --test rotation  # Rotation detection test")
        print("  python run_tests.py --test all       # All tests") 