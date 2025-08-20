import sys
import os

project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

def run_ai_communication_test():
    print("AI communication test - Use pytest instead")
    print("Run: pytest tests/ -m ai")

def run_angle_control_test():
    print("Angle control test - Use pytest instead")
    print("Run: pytest tests/ -m motor")

def run_imu_sensor_test():
    print("IMU sensor test - Use pytest instead")
    print("Run: pytest tests/ -m sensor")

def run_mqtt_test():
    print("MQTT test - Use pytest instead")
    print("Run: pytest tests/ -m mqtt")

def run_obstacle_detection_test():
    print("Obstacle detection test - Use pytest instead")
    print("Run: pytest tests/ -m sensor")

def run_rotation_detection_test():
    print("Rotation detection test - Use pytest instead")
    print("Run: pytest tests/ -m sensor")

def run_all_tests():
    print("Running all tests...")
    
    import subprocess
    import sys
    
    try:
        print("Running unit tests...")
        result = subprocess.run([
            sys.executable, "-m", "pytest", 
            "tests/unit/", 
            "-v", 
            "--tb=short"
        ], capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ Unit tests passed")
        else:
            print("✗ Unit tests failed")
            print(result.stdout)
            print(result.stderr)
        
        print("\nRunning integration tests...")
        result = subprocess.run([
            sys.executable, "-m", "pytest", 
            "tests/integration/", 
            "-v", 
            "--tb=short"
        ], capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ Integration tests passed")
        else:
            print("✗ Integration tests failed")
            print(result.stdout)
            print(result.stderr)
        
        print("\nRunning E2E tests...")
        result = subprocess.run([
            sys.executable, "-m", "pytest", 
            "tests/e2e/", 
            "-v", 
            "--tb=short"
        ], capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ E2E tests passed")
        else:
            print("✗ E2E tests failed")
            print(result.stdout)
            print(result.stderr)
            
    except Exception as e:
        print(f"Error occurred during test execution: {e}")

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