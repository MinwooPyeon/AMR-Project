import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

os.environ['PYTHONPATH'] = current_dir + ':' + os.environ.get('PYTHONPATH', '')

try:
    from main.amr_real_data_sync import AMRRealDataSync
    print("AMR system module import successful")
    
    amr_sync = AMRRealDataSync("AMR001", enable_mqtt=True, enable_backup=True)
    print("AMR system initialization completed")
    
    print("\nMotor status check:")
    motor_speeds = amr_sync.get_motor_speeds()
    print(f"  - Left motor speed: {motor_speeds.get('left_speed', 0)}%")
    print(f"  - Right motor speed: {motor_speeds.get('right_speed', 0)}%")
    print(f"  - Motor running: {'Yes' if motor_speeds.get('is_running', False) else 'No'}")
    
    print("\nAMR system execution completed")
    
except ImportError as e:
    print(f"Module import error: {e}")
    print("Solution:")
    print("   1. Check current directory: pwd")
    print("   2. Set Python path: export PYTHONPATH=\".\"")
    print("   3. Run again: python3 run_amr.py")
    
except Exception as e:
    print(f"Execution error: {e}")
    print("Debug information:")
    print(f"   - Current directory: {os.getcwd()}")
    print(f"   - Python path: {sys.path}")
    print(f"   - File path: {os.path.abspath(__file__)}")

if __name__ == "__main__":
    print("AMR system starting...") 