echo "=== GY-BN008x IMU-Motor Integration System Build Start ==="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Script directory: $SCRIPT_DIR"

BUILD_DIR="$SCRIPT_DIR/build"
echo "Build directory: $BUILD_DIR"

if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
fi

cd "$BUILD_DIR"

echo "Configuring CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "Error: CMake configuration failed"
    exit 1
fi

echo "Building..."
make -j$(nproc)

if [ $? -ne 0 ]; then
    echo "Error: Build failed"
    exit 1
fi

if [ -f "imu_motor_test" ]; then
    echo "imu_motor_test build successful"
    echo "Executable size: $(ls -lh imu_motor_test | awk '{print $5}')"
else
    echo "imu_motor_test build failed"
    exit 1
fi

if [ -f "amr_main" ]; then
    echo "amr_main build successful"
    echo "Executable size: $(ls -lh amr_main | awk '{print $5}')"
else
    echo "amr_main build failed"
    exit 1
fi

echo ""
echo "=== Build Complete ==="
echo "Run method:"
echo "  cd $BUILD_DIR"
echo "  ./imu_motor_test"
echo ""
echo "Or use Python test script:"
echo "  cd $SCRIPT_DIR"
echo "  python3 test_imu_motor_simple.py"
echo ""

chmod +x imu_motor_test
chmod +x amr_main

echo "Execution permissions set"
echo "Build complete!"
