export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH}"

echo "Python environment setup complete:"
echo "  PROJECT_ROOT: $PROJECT_ROOT"
echo "  PYTHONPATH: $PYTHONPATH"

echo ""
echo "Test execution methods:"
echo "  python tests/test_connection_setup.py"
echo "  python tests/angle_control_test.py"
echo "  python run_tests.py --test connection"
echo "  python run_tests.py --test angle" 