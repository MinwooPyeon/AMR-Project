# AMR 시스템 실행 스크립트
# 사용법: ./run_amr.sh [start|stop|status|monitor|check|list]

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/amr_system.log"

PID_FILE="$SCRIPT_DIR/amr_system.pid"

log() {
    echo -e "${GREEN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1" | tee -a "$LOG_FILE"
}

error_log() {
    echo -e "${RED}[$(date '+%Y-%m-%d %H:%M:%S')] ERROR:${NC} $1" | tee -a "$LOG_FILE"
}

warn_log() {
    echo -e "${YELLOW}[$(date '+%Y-%m-%d %H:%M:%S')] WARNING:${NC} $1" | tee -a "$LOG_FILE"
}

info_log() {
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')] INFO:${NC} $1" | tee -a "$LOG_FILE"
}

start_system() {
    log "Starting AMR System..."
    
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if ps -p "$PID" > /dev/null 2>&1; then
            warn_log "AMR System is already running (PID: $PID)"
            return 1
        else
            warn_log "Stale PID file found, removing..."
            rm -f "$PID_FILE"
        fi
    fi
    
    if ! command -v python3 &> /dev/null; then
        error_log "Python3 is not installed"
        return 1
    fi
    
    log "Starting process manager..."
    python3 process_manager.py start-all &
    MANAGER_PID=$!
    
    echo "$MANAGER_PID" > "$PID_FILE"
    
    sleep 3
    if ps -p "$MANAGER_PID" > /dev/null 2>&1; then
        log "AMR System started successfully (PID: $MANAGER_PID)"
        return 0
    else
        error_log "Failed to start AMR System"
        rm -f "$PID_FILE"
        return 1
    fi
}

stop_system() {
    log "Stopping AMR System..."
    
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if ps -p "$PID" > /dev/null 2>&1; then
            log "Stopping process manager (PID: $PID)..."
            kill "$PID"
            
            for i in {1..10}; do
                if ! ps -p "$PID" > /dev/null 2>&1; then
                    break
                fi
                sleep 1
            done
            
            if ps -p "$PID" > /dev/null 2>&1; then
                warn_log "Force killing process..."
                kill -9 "$PID"
            fi
            
            rm -f "$PID_FILE"
            log "AMR System stopped"
        else
            warn_log "Process not running, cleaning up PID file"
            rm -f "$PID_FILE"
        fi
    else
        warn_log "PID file not found"
    fi
    
    pkill -f "process_manager.py" 2>/dev/null || true
    pkill -f "mosquitto" 2>/dev/null || true
}

check_status() {
    log "Checking AMR System status..."
    
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if ps -p "$PID" > /dev/null 2>&1; then
            log "AMR System is running (PID: $PID)"
            python3 process_manager.py status
            return 0
        else
            warn_log "PID file exists but process is not running"
            rm -f "$PID_FILE"
            return 1
        fi
    else
        log "AMR System is not running"
        return 1
    fi
}

monitor_system() {
    log "Starting AMR System in monitoring mode..."
    
    if ! start_system; then
        error_log "Failed to start system for monitoring"
        return 1
    fi
    
    log "Starting monitoring..."
    python3 process_manager.py monitor &
    MONITOR_PID=$!
    
    trap 'cleanup_monitor' INT TERM
    
    wait "$MONITOR_PID"
}

cleanup_monitor() {
    log "Stopping monitoring..."
    kill "$MONITOR_PID" 2>/dev/null || true
    stop_system
    exit 0
}

check_dependencies() {
    log "Checking system dependencies..."
    python3 process_manager.py check
}

list_modules() {
    log "Listing available modules..."
    python3 process_manager.py list
}

show_help() {
    echo "AMR System Management Script"
    echo ""
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  start     - Start AMR system"
    echo "  stop      - Stop AMR system"
    echo "  restart   - Restart AMR system"
    echo "  status    - Show system status"
    echo "  monitor   - Start system with monitoring"
    echo "  check     - Check dependencies"
    echo "  list      - List available modules"
    echo "  help      - Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 start      # Start the system"
    echo "  $0 monitor    # Start with monitoring"
    echo "  $0 status     # Check status"
    echo "  $0 stop       # Stop the system"
}

case "${1:-help}" in
    start)
        start_system
        ;;
    stop)
        stop_system
        ;;
    restart)
        log "Restarting AMR System..."
        stop_system
        sleep 2
        start_system
        ;;
    status)
        check_status
        ;;
    monitor)
        monitor_system
        ;;
    check)
        check_dependencies
        ;;
    list)
        list_modules
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        error_log "Unknown command: $1"
        show_help
        exit 1
        ;;
esac
