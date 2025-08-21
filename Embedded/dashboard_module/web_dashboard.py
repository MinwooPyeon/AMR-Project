#!/usr/bin/env python3
"""
웹 기반 실시간 AMR 대시보드 시스템
실시간 상태 모니터링, 제어 인터페이스, 데이터 시각화
"""

import os
import sys
import time
import json
import logging
import threading
from typing import Dict, List, Optional, Callable, Any
from pathlib import Path
from datetime import datetime, timedelta
import asyncio
import websockets
from flask import Flask, render_template, jsonify, request, Response
from flask_socketio import SocketIO, emit
import cv2
import numpy as np
from PIL import Image
import base64
import io

# 프로젝트 루트 추가
sys.path.append(str(Path(__file__).parent.parent))
from config.system_config import get_config
from security.security_manager import get_security_manager

class WebDashboard:
    """웹 기반 실시간 대시보드 클래스"""
    
    def __init__(self):
        self.config = get_config()
        self.security_manager = get_security_manager()
        self.logger = self._setup_logger()
        
        # 대시보드 설정
        self.dashboard_config = {
            'host': '0.0.0.0',
            'port': 8080,
            'debug': False,
            'ssl_enabled': True,
            'ssl_cert': 'security/certs/server.crt',
            'ssl_key': 'security/certs/server.key',
            'update_interval': 1.0,  # 1초
            'max_clients': 100,
            'enable_video_stream': True,
            'video_fps': 10,
            'enable_websocket': True,
            'enable_rest_api': True
        }
        
        # 대시보드 상태
        self.dashboard_status = {
            'is_running': False,
            'connected_clients': 0,
            'total_requests': 0,
            'last_update': time.time(),
            'system_status': 'offline',
            'active_modules': []
        }
        
        # 실시간 데이터
        self.realtime_data = {
            'robot_status': {},
            'sensor_data': {},
            'battery_status': {},
            'motor_status': {},
            'ai_status': {},
            'security_status': {},
            'system_metrics': {}
        }
        
        # 클라이언트 관리
        self.connected_clients = {}
        self.client_callbacks = {}
        
        # 웹 애플리케이션
        self.app = None
        self.socketio = None
        self.websocket_server = None
        
        # 스레드 안전을 위한 락
        self.lock = threading.Lock()
        
        # 데이터 업데이트 스레드
        self.update_thread = None
        self.is_updating = False
        
        # 초기화
        self._initialize_dashboard()
    
    def _setup_logger(self) -> logging.Logger:
        """로거 설정"""
        logger = logging.getLogger('web_dashboard')
        logger.setLevel(logging.INFO)
        
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(name)s - %(levelname)s: %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        
        return logger
    
    def _initialize_dashboard(self):
        """대시보드 초기화"""
        try:
            # Flask 애플리케이션 생성
            self._setup_flask_app()
            
            # 웹소켓 서버 설정
            if self.dashboard_config['enable_websocket']:
                self._setup_websocket_server()
            
            # 템플릿 및 정적 파일 설정
            self._setup_templates()
            
            # 라우트 설정
            self._setup_routes()
            
            # 데이터 업데이트 시작
            self._start_data_updates()
            
            self.logger.info("Web dashboard initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Dashboard initialization failed: {e}")
            raise
    
    def _setup_flask_app(self):
        """Flask 애플리케이션 설정"""
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'amr_dashboard_secret_key'
        
        # SocketIO 설정
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            async_mode='threading'
        )
        
        # SSL 설정
        if self.dashboard_config['ssl_enabled']:
            cert_file = Path(self.dashboard_config['ssl_cert'])
            key_file = Path(self.dashboard_config['ssl_key'])
            
            if cert_file.exists() and key_file.exists():
                self.app.config['SSL_CONTEXT'] = (
                    str(cert_file), str(key_file)
                )
    
    def _setup_websocket_server(self):
        """웹소켓 서버 설정"""
        # SocketIO 이벤트 핸들러 설정
        self.socketio.on_event('connect', self._on_client_connect)
        self.socketio.on_event('disconnect', self._on_client_disconnect)
        self.socketio.on_event('get_status', self._on_get_status)
        self.socketio.on_event('send_command', self._on_send_command)
        self.socketio.on_event('subscribe_data', self._on_subscribe_data)
    
    def _setup_templates(self):
        """템플릿 및 정적 파일 설정"""
        # 템플릿 디렉토리 생성
        template_dir = Path('dashboard_module/templates')
        static_dir = Path('dashboard_module/static')
        
        template_dir.mkdir(parents=True, exist_ok=True)
        static_dir.mkdir(parents=True, exist_ok=True)
        
        # 기본 HTML 템플릿 생성
        self._create_main_template()
        
        # CSS 및 JavaScript 파일 생성
        self._create_static_files()
    
    def _create_main_template(self):
        """메인 HTML 템플릿 생성"""
        template_content = """<!DOCTYPE html>
<html lang="ko">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AMR 실시간 대시보드</title>
    <link rel="stylesheet" href="{{ url_for('static', filename='css/style.css') }}">
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
    <div class="dashboard-container">
        <header class="dashboard-header">
            <h1>AMR 실시간 대시보드</h1>
            <div class="status-indicator">
                <span id="connection-status" class="status-offline">오프라인</span>
                <span id="last-update">마지막 업데이트: --</span>
            </div>
        </header>
        
        <div class="dashboard-content">
            <div class="main-panel">
                <div class="status-cards">
                    <div class="status-card" id="robot-status">
                        <h3>로봇 상태</h3>
                        <div class="status-value" id="robot-status-value">--</div>
                    </div>
                    <div class="status-card" id="battery-status">
                        <h3>배터리</h3>
                        <div class="status-value" id="battery-percentage">--%</div>
                        <div class="battery-bar">
                            <div class="battery-fill" id="battery-fill"></div>
                        </div>
                    </div>
                    <div class="status-card" id="motor-status">
                        <h3>모터</h3>
                        <div class="status-value" id="motor-status-value">--</div>
                    </div>
                    <div class="status-card" id="ai-status">
                        <h3>AI 상태</h3>
                        <div class="status-value" id="ai-status-value">--</div>
                    </div>
                </div>
                
                <div class="control-panel">
                    <h3>제어 패널</h3>
                    <div class="control-buttons">
                        <button id="btn-forward" class="control-btn">전진</button>
                        <button id="btn-backward" class="control-btn">후진</button>
                        <button id="btn-left" class="control-btn">좌회전</button>
                        <button id="btn-right" class="control-btn">우회전</button>
                        <button id="btn-stop" class="control-btn stop-btn">정지</button>
                    </div>
                    <div class="speed-control">
                        <label for="speed-slider">속도: <span id="speed-value">50</span>%</label>
                        <input type="range" id="speed-slider" min="0" max="100" value="50">
                    </div>
                </div>
            </div>
            
            <div class="side-panel">
                <div class="data-panel">
                    <h3>센서 데이터</h3>
                    <div class="sensor-data" id="sensor-data">
                        <div>IMU X: <span id="imu-x">--</span></div>
                        <div>IMU Y: <span id="imu-y">--</span></div>
                        <div>IMU Z: <span id="imu-z">--</span></div>
                        <div>온도: <span id="temperature">--°C</span></div>
                    </div>
                </div>
                
                <div class="chart-panel">
                    <h3>전력 소비</h3>
                    <canvas id="power-chart"></canvas>
                </div>
                
                <div class="log-panel">
                    <h3>시스템 로그</h3>
                    <div class="log-container" id="log-container"></div>
                </div>
            </div>
        </div>
        
        <div class="video-panel" id="video-panel">
            <h3>실시간 영상</h3>
            <div class="video-container">
                <img id="video-stream" src="/video_feed" alt="실시간 영상">
            </div>
        </div>
    </div>
    
    <script src="{{ url_for('static', filename='js/dashboard.js') }}"></script>
</body>
</html>"""
        
        template_file = Path('dashboard_module/templates/index.html')
        with open(template_file, 'w', encoding='utf-8') as f:
            f.write(template_content)
    
    def _create_static_files(self):
        """정적 파일 생성"""
        # CSS 파일
        css_content = """/* AMR 대시보드 스타일 */
* {
    margin: 0;
    padding: 0;
    box-sizing: border-box;
}

body {
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    min-height: 100vh;
    color: #333;
}

.dashboard-container {
    max-width: 1400px;
    margin: 0 auto;
    padding: 20px;
}

.dashboard-header {
    background: rgba(255, 255, 255, 0.95);
    padding: 20px;
    border-radius: 10px;
    margin-bottom: 20px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.dashboard-header h1 {
    color: #2c3e50;
    font-size: 2em;
}

.status-indicator {
    display: flex;
    flex-direction: column;
    align-items: flex-end;
}

.status-online {
    color: #27ae60;
    font-weight: bold;
}

.status-offline {
    color: #e74c3c;
    font-weight: bold;
}

.dashboard-content {
    display: grid;
    grid-template-columns: 2fr 1fr;
    gap: 20px;
    margin-bottom: 20px;
}

.main-panel {
    background: rgba(255, 255, 255, 0.95);
    border-radius: 10px;
    padding: 20px;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.status-cards {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
    gap: 15px;
    margin-bottom: 20px;
}

.status-card {
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    color: white;
    padding: 20px;
    border-radius: 10px;
    text-align: center;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.status-card h3 {
    margin-bottom: 10px;
    font-size: 1.1em;
}

.status-value {
    font-size: 1.5em;
    font-weight: bold;
}

.battery-bar {
    width: 100%;
    height: 10px;
    background: rgba(255, 255, 255, 0.3);
    border-radius: 5px;
    margin-top: 10px;
    overflow: hidden;
}

.battery-fill {
    height: 100%;
    background: #27ae60;
    transition: width 0.3s ease;
}

.control-panel {
    background: #f8f9fa;
    padding: 20px;
    border-radius: 10px;
}

.control-panel h3 {
    margin-bottom: 15px;
    color: #2c3e50;
}

.control-buttons {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 10px;
    margin-bottom: 15px;
}

.control-btn {
    padding: 12px;
    border: none;
    border-radius: 5px;
    background: #3498db;
    color: white;
    font-weight: bold;
    cursor: pointer;
    transition: background 0.3s ease;
}

.control-btn:hover {
    background: #2980b9;
}

.stop-btn {
    background: #e74c3c;
    grid-column: span 3;
}

.stop-btn:hover {
    background: #c0392b;
}

.speed-control {
    display: flex;
    flex-direction: column;
    gap: 10px;
}

.speed-control label {
    font-weight: bold;
    color: #2c3e50;
}

.speed-control input[type="range"] {
    width: 100%;
}

.side-panel {
    display: flex;
    flex-direction: column;
    gap: 20px;
}

.data-panel, .chart-panel, .log-panel {
    background: rgba(255, 255, 255, 0.95);
    border-radius: 10px;
    padding: 20px;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.data-panel h3, .chart-panel h3, .log-panel h3 {
    margin-bottom: 15px;
    color: #2c3e50;
}

.sensor-data div {
    margin-bottom: 8px;
    padding: 8px;
    background: #f8f9fa;
    border-radius: 5px;
}

.log-container {
    height: 200px;
    overflow-y: auto;
    background: #f8f9fa;
    padding: 10px;
    border-radius: 5px;
    font-family: monospace;
    font-size: 0.9em;
}

.log-entry {
    margin-bottom: 5px;
    padding: 5px;
    border-radius: 3px;
}

.log-info { background: #d1ecf1; }
.log-warning { background: #fff3cd; }
.log-error { background: #f8d7da; }

.video-panel {
    background: rgba(255, 255, 255, 0.95);
    border-radius: 10px;
    padding: 20px;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.video-panel h3 {
    margin-bottom: 15px;
    color: #2c3e50;
}

.video-container {
    text-align: center;
}

#video-stream {
    max-width: 100%;
    border-radius: 10px;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

@media (max-width: 768px) {
    .dashboard-content {
        grid-template-columns: 1fr;
    }
    
    .status-cards {
        grid-template-columns: repeat(2, 1fr);
    }
    
    .control-buttons {
        grid-template-columns: repeat(2, 1fr);
    }
    
    .stop-btn {
        grid-column: span 2;
    }
}"""
        
        css_file = Path('dashboard_module/static/css/style.css')
        css_file.parent.mkdir(parents=True, exist_ok=True)
        with open(css_file, 'w', encoding='utf-8') as f:
            f.write(css_content)
        
        # JavaScript 파일
        js_content = """// AMR 대시보드 JavaScript
class AMRDashboard {
    constructor() {
        this.socket = null;
        this.powerChart = null;
        this.isConnected = false;
        this.updateInterval = null;
        
        this.initialize();
    }
    
    initialize() {
        this.setupSocket();
        this.setupEventListeners();
        this.setupCharts();
        this.startPeriodicUpdates();
    }
    
    setupSocket() {
        this.socket = io();
        
        this.socket.on('connect', () => {
            this.isConnected = true;
            this.updateConnectionStatus(true);
            console.log('Connected to dashboard server');
        });
        
        this.socket.on('disconnect', () => {
            this.isConnected = false;
            this.updateConnectionStatus(false);
            console.log('Disconnected from dashboard server');
        });
        
        this.socket.on('status_update', (data) => {
            this.updateDashboard(data);
        });
        
        this.socket.on('sensor_data', (data) => {
            this.updateSensorData(data);
        });
        
        this.socket.on('battery_update', (data) => {
            this.updateBatteryStatus(data);
        });
        
        this.socket.on('motor_status', (data) => {
            this.updateMotorStatus(data);
        });
        
        this.socket.on('ai_status', (data) => {
            this.updateAIStatus(data);
        });
        
        this.socket.on('system_log', (data) => {
            this.addLogEntry(data);
        });
    }
    
    setupEventListeners() {
        // 제어 버튼 이벤트
        document.getElementById('btn-forward').addEventListener('click', () => {
            this.sendCommand('move_forward');
        });
        
        document.getElementById('btn-backward').addEventListener('click', () => {
            this.sendCommand('move_backward');
        });
        
        document.getElementById('btn-left').addEventListener('click', () => {
            this.sendCommand('turn_left');
        });
        
        document.getElementById('btn-right').addEventListener('click', () => {
            this.sendCommand('turn_right');
        });
        
        document.getElementById('btn-stop').addEventListener('click', () => {
            this.sendCommand('stop');
        });
        
        // 속도 슬라이더 이벤트
        const speedSlider = document.getElementById('speed-slider');
        const speedValue = document.getElementById('speed-value');
        
        speedSlider.addEventListener('input', (e) => {
            const speed = e.target.value;
            speedValue.textContent = speed;
            this.sendCommand('set_speed', { speed: parseInt(speed) });
        });
    }
    
    setupCharts() {
        const ctx = document.getElementById('power-chart').getContext('2d');
        this.powerChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: '전력 소비 (W)',
                    data: [],
                    borderColor: 'rgb(75, 192, 192)',
                    backgroundColor: 'rgba(75, 192, 192, 0.2)',
                    tension: 0.1
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: true
                    }
                },
                animation: {
                    duration: 0
                }
            }
        });
    }
    
    sendCommand(command, data = {}) {
        if (this.isConnected) {
            this.socket.emit('send_command', {
                command: command,
                data: data,
                timestamp: Date.now()
            });
        }
    }
    
    updateConnectionStatus(connected) {
        const statusElement = document.getElementById('connection-status');
        if (connected) {
            statusElement.textContent = '온라인';
            statusElement.className = 'status-online';
        } else {
            statusElement.textContent = '오프라인';
            statusElement.className = 'status-offline';
        }
    }
    
    updateDashboard(data) {
        // 로봇 상태 업데이트
        if (data.robot_status) {
            document.getElementById('robot-status-value').textContent = 
                data.robot_status.state || '--';
        }
        
        // 마지막 업데이트 시간
        document.getElementById('last-update').textContent = 
            `마지막 업데이트: ${new Date().toLocaleTimeString()}`;
    }
    
    updateSensorData(data) {
        if (data.imu) {
            document.getElementById('imu-x').textContent = data.imu.x?.toFixed(2) || '--';
            document.getElementById('imu-y').textContent = data.imu.y?.toFixed(2) || '--';
            document.getElementById('imu-z').textContent = data.imu.z?.toFixed(2) || '--';
        }
        
        if (data.temperature !== undefined) {
            document.getElementById('temperature').textContent = 
                `${data.temperature.toFixed(1)}°C`;
        }
    }
    
    updateBatteryStatus(data) {
        const percentage = data.percentage || 0;
        document.getElementById('battery-percentage').textContent = `${percentage.toFixed(1)}%`;
        
        const batteryFill = document.getElementById('battery-fill');
        batteryFill.style.width = `${percentage}%`;
        
        // 배터리 색상 변경
        if (percentage > 50) {
            batteryFill.style.background = '#27ae60';
        } else if (percentage > 20) {
            batteryFill.style.background = '#f39c12';
        } else {
            batteryFill.style.background = '#e74c3c';
        }
    }
    
    updateMotorStatus(data) {
        const status = data.status || '--';
        document.getElementById('motor-status-value').textContent = status;
    }
    
    updateAIStatus(data) {
        const status = data.status || '--';
        document.getElementById('ai-status-value').textContent = status;
    }
    
    updatePowerChart(powerData) {
        if (this.powerChart && powerData) {
            const timestamp = new Date().toLocaleTimeString();
            
            this.powerChart.data.labels.push(timestamp);
            this.powerChart.data.datasets[0].data.push(powerData.power || 0);
            
            // 최근 20개 데이터만 표시
            if (this.powerChart.data.labels.length > 20) {
                this.powerChart.data.labels.shift();
                this.powerChart.data.datasets[0].data.shift();
            }
            
            this.powerChart.update();
        }
    }
    
    addLogEntry(logData) {
        const logContainer = document.getElementById('log-container');
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${logData.level || 'info'}`;
        
        const timestamp = new Date(logData.timestamp).toLocaleTimeString();
        logEntry.textContent = `[${timestamp}] ${logData.message}`;
        
        logContainer.appendChild(logEntry);
        logContainer.scrollTop = logContainer.scrollHeight;
        
        // 최근 50개 로그만 유지
        while (logContainer.children.length > 50) {
            logContainer.removeChild(logContainer.firstChild);
        }
    }
    
    startPeriodicUpdates() {
        this.updateInterval = setInterval(() => {
            if (this.isConnected) {
                this.socket.emit('get_status');
            }
        }, 1000);
    }
    
    stop() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
        }
        if (this.socket) {
            this.socket.disconnect();
        }
    }
}

// 대시보드 초기화
document.addEventListener('DOMContentLoaded', () => {
    window.dashboard = new AMRDashboard();
});

// 페이지 언로드 시 정리
window.addEventListener('beforeunload', () => {
    if (window.dashboard) {
        window.dashboard.stop();
    }
});"""
        
        js_file = Path('dashboard_module/static/js/dashboard.js')
        js_file.parent.mkdir(parents=True, exist_ok=True)
        with open(js_file, 'w', encoding='utf-8') as f:
            f.write(js_content)
    
    def _setup_routes(self):
        """Flask 라우트 설정"""
        
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.app.route('/api/status')
        def api_status():
            return jsonify(self.get_dashboard_status())
        
        @self.app.route('/api/robot_status')
        def api_robot_status():
            return jsonify(self.realtime_data['robot_status'])
        
        @self.app.route('/api/sensor_data')
        def api_sensor_data():
            return jsonify(self.realtime_data['sensor_data'])
        
        @self.app.route('/api/battery_status')
        def api_battery_status():
            return jsonify(self.realtime_data['battery_status'])
        
        @self.app.route('/api/motor_status')
        def api_motor_status():
            return jsonify(self.realtime_data['motor_status'])
        
        @self.app.route('/api/ai_status')
        def api_ai_status():
            return jsonify(self.realtime_data['ai_status'])
        
        @self.app.route('/api/security_status')
        def api_security_status():
            return jsonify(self.realtime_data['security_status'])
        
        @self.app.route('/api/send_command', methods=['POST'])
        def api_send_command():
            try:
                data = request.get_json()
                command = data.get('command')
                command_data = data.get('data', {})
                
                result = self._process_command(command, command_data)
                return jsonify({'success': True, 'result': result})
                
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 400
        
        @self.app.route('/video_feed')
        def video_feed():
            if self.dashboard_config['enable_video_stream']:
                return Response(
                    self._generate_video_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame'
                )
            else:
                return "Video stream disabled", 404
    
    def _on_client_connect(self):
        """클라이언트 연결 이벤트"""
        client_id = request.sid
        with self.lock:
            self.connected_clients[client_id] = {
                'connected_at': time.time(),
                'last_activity': time.time()
            }
            self.dashboard_status['connected_clients'] = len(self.connected_clients)
        
        self.logger.info(f"Client connected: {client_id}")
        
        # 초기 상태 전송
        emit('status_update', self.realtime_data)
    
    def _on_client_disconnect(self):
        """클라이언트 연결 해제 이벤트"""
        client_id = request.sid
        with self.lock:
            if client_id in self.connected_clients:
                del self.connected_clients[client_id]
                self.dashboard_status['connected_clients'] = len(self.connected_clients)
        
        self.logger.info(f"Client disconnected: {client_id}")
    
    def _on_get_status(self):
        """상태 요청 이벤트"""
        emit('status_update', self.realtime_data)
    
    def _on_send_command(self, data):
        """명령 전송 이벤트"""
        try:
            command = data.get('command')
            command_data = data.get('data', {})
            
            result = self._process_command(command, command_data)
            emit('command_result', {'success': True, 'result': result})
            
        except Exception as e:
            emit('command_result', {'success': False, 'error': str(e)})
    
    def _on_subscribe_data(self, data):
        """데이터 구독 이벤트"""
        data_type = data.get('type')
        if data_type in self.client_callbacks:
            self.client_callbacks[data_type].append(request.sid)
    
    def _process_command(self, command: str, data: Dict) -> Dict:
        """명령 처리"""
        # 보안 검사
        if not self.security_manager.check_rate_limit('dashboard'):
            raise Exception("Rate limit exceeded")
        
        # 명령 처리 로직
        if command == 'move_forward':
            return self._handle_move_command('forward', data)
        elif command == 'move_backward':
            return self._handle_move_command('backward', data)
        elif command == 'turn_left':
            return self._handle_turn_command('left', data)
        elif command == 'turn_right':
            return self._handle_turn_command('right', data)
        elif command == 'stop':
            return self._handle_stop_command()
        elif command == 'set_speed':
            return self._handle_speed_command(data)
        else:
            raise Exception(f"Unknown command: {command}")
    
    def _handle_move_command(self, direction: str, data: Dict) -> Dict:
        """이동 명령 처리"""
        speed = data.get('speed', 50)
        
        # 모터 제어 콜백 호출
        if hasattr(self, 'motor_callback'):
            self.motor_callback({
                'action': 'move',
                'direction': direction,
                'speed': speed
            })
        
        return {
            'command': f'move_{direction}',
            'speed': speed,
            'status': 'executing'
        }
    
    def _handle_turn_command(self, direction: str, data: Dict) -> Dict:
        """회전 명령 처리"""
        angle = data.get('angle', 90)
        speed = data.get('speed', 30)
        
        # 모터 제어 콜백 호출
        if hasattr(self, 'motor_callback'):
            self.motor_callback({
                'action': 'turn',
                'direction': direction,
                'angle': angle,
                'speed': speed
            })
        
        return {
            'command': f'turn_{direction}',
            'angle': angle,
            'speed': speed,
            'status': 'executing'
        }
    
    def _handle_stop_command(self) -> Dict:
        """정지 명령 처리"""
        # 모터 제어 콜백 호출
        if hasattr(self, 'motor_callback'):
            self.motor_callback({
                'action': 'stop'
            })
        
        return {
            'command': 'stop',
            'status': 'stopped'
        }
    
    def _handle_speed_command(self, data: Dict) -> Dict:
        """속도 설정 명령 처리"""
        speed = data.get('speed', 50)
        
        # 모터 제어 콜백 호출
        if hasattr(self, 'motor_callback'):
            self.motor_callback({
                'action': 'set_speed',
                'speed': speed
            })
        
        return {
            'command': 'set_speed',
            'speed': speed,
            'status': 'updated'
        }
    
    def _generate_video_frames(self):
        """비디오 프레임 생성"""
        # 실제 구현에서는 카메라에서 프레임을 받아서 처리
        # 여기서는 시뮬레이션된 프레임 생성
        
        while True:
            # 시뮬레이션된 프레임 생성
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # 현재 시간 표시
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, timestamp, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # 로봇 상태 표시
            status = self.realtime_data.get('robot_status', {}).get('state', 'Unknown')
            cv2.putText(frame, f"Status: {status}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # JPEG로 인코딩
            _, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(1.0 / self.dashboard_config['video_fps'])
    
    def _start_data_updates(self):
        """데이터 업데이트 시작"""
        self.is_updating = True
        self.update_thread = threading.Thread(
            target=self._data_update_loop,
            daemon=True
        )
        self.update_thread.start()
    
    def _data_update_loop(self):
        """데이터 업데이트 루프"""
        while self.is_updating:
            try:
                # 실시간 데이터 업데이트
                self._update_realtime_data()
                
                # 웹소켓으로 데이터 전송
                if self.socketio:
                    self.socketio.emit('status_update', self.realtime_data)
                
                # 업데이트 간격 대기
                time.sleep(self.dashboard_config['update_interval'])
                
            except Exception as e:
                self.logger.error(f"Data update error: {e}")
                time.sleep(5)
    
    def _update_realtime_data(self):
        """실시간 데이터 업데이트"""
        with self.lock:
            # 시스템 메트릭 업데이트
            self.realtime_data['system_metrics'] = {
                'cpu_usage': self._get_cpu_usage(),
                'memory_usage': self._get_memory_usage(),
                'disk_usage': self._get_disk_usage(),
                'network_usage': self._get_network_usage(),
                'uptime': self._get_uptime()
            }
            
            # 마지막 업데이트 시간
            self.dashboard_status['last_update'] = time.time()
    
    def _get_cpu_usage(self) -> float:
        """CPU 사용률 반환"""
        try:
            import psutil
            return psutil.cpu_percent(interval=1)
        except:
            return 0.0
    
    def _get_memory_usage(self) -> float:
        """메모리 사용률 반환"""
        try:
            import psutil
            memory = psutil.virtual_memory()
            return memory.percent
        except:
            return 0.0
    
    def _get_disk_usage(self) -> float:
        """디스크 사용률 반환"""
        try:
            import psutil
            disk = psutil.disk_usage('/')
            return (disk.used / disk.total) * 100
        except:
            return 0.0
    
    def _get_network_usage(self) -> Dict:
        """네트워크 사용량 반환"""
        try:
            import psutil
            network = psutil.net_io_counters()
            return {
                'bytes_sent': network.bytes_sent,
                'bytes_recv': network.bytes_recv
            }
        except:
            return {'bytes_sent': 0, 'bytes_recv': 0}
    
    def _get_uptime(self) -> float:
        """시스템 가동 시간 반환"""
        try:
            import psutil
            return time.time() - psutil.boot_time()
        except:
            return 0.0
    
    def update_robot_status(self, status: Dict):
        """로봇 상태 업데이트"""
        with self.lock:
            self.realtime_data['robot_status'] = status
        
        if self.socketio:
            self.socketio.emit('robot_status', status)
    
    def update_sensor_data(self, data: Dict):
        """센서 데이터 업데이트"""
        with self.lock:
            self.realtime_data['sensor_data'] = data
        
        if self.socketio:
            self.socketio.emit('sensor_data', data)
    
    def update_battery_status(self, data: Dict):
        """배터리 상태 업데이트"""
        with self.lock:
            self.realtime_data['battery_status'] = data
        
        if self.socketio:
            self.socketio.emit('battery_update', data)
    
    def update_motor_status(self, data: Dict):
        """모터 상태 업데이트"""
        with self.lock:
            self.realtime_data['motor_status'] = data
        
        if self.socketio:
            self.socketio.emit('motor_status', data)
    
    def update_ai_status(self, data: Dict):
        """AI 상태 업데이트"""
        with self.lock:
            self.realtime_data['ai_status'] = data
        
        if self.socketio:
            self.socketio.emit('ai_status', data)
    
    def update_security_status(self, data: Dict):
        """보안 상태 업데이트"""
        with self.lock:
            self.realtime_data['security_status'] = data
        
        if self.socketio:
            self.socketio.emit('security_status', data)
    
    def add_system_log(self, level: str, message: str):
        """시스템 로그 추가"""
        log_data = {
            'level': level,
            'message': message,
            'timestamp': time.time()
        }
        
        if self.socketio:
            self.socketio.emit('system_log', log_data)
    
    def set_motor_callback(self, callback: Callable):
        """모터 제어 콜백 설정"""
        self.motor_callback = callback
    
    def get_dashboard_status(self) -> Dict[str, Any]:
        """대시보드 상태 반환"""
        with self.lock:
            return self.dashboard_status.copy()
    
    def start(self):
        """대시보드 시작"""
        try:
            self.dashboard_status['is_running'] = True
            self.dashboard_status['system_status'] = 'online'
            
            # SSL 설정 확인
            ssl_context = None
            if self.dashboard_config['ssl_enabled']:
                cert_file = Path(self.dashboard_config['ssl_cert'])
                key_file = Path(self.dashboard_config['ssl_key'])
                
                if cert_file.exists() and key_file.exists():
                    ssl_context = (str(cert_file), str(key_file))
            
            # Flask 애플리케이션 실행
            self.socketio.run(
                self.app,
                host=self.dashboard_config['host'],
                port=self.dashboard_config['port'],
                debug=self.dashboard_config['debug'],
                ssl_context=ssl_context
            )
            
        except Exception as e:
            self.logger.error(f"Failed to start dashboard: {e}")
            raise
    
    def stop(self):
        """대시보드 중지"""
        self.is_updating = False
        self.dashboard_status['is_running'] = False
        self.dashboard_status['system_status'] = 'offline'
        
        if self.update_thread:
            self.update_thread.join(timeout=5)
        
        self.logger.info("Dashboard stopped")

# 전역 대시보드 인스턴스
web_dashboard = WebDashboard()

def get_web_dashboard() -> WebDashboard:
    """웹 대시보드 인스턴스 반환"""
    return web_dashboard

if __name__ == "__main__":
    # 웹 대시보드 테스트
    dashboard = get_web_dashboard()
    
    print("=== Web Dashboard Test ===")
    print(f"Dashboard Status: {dashboard.get_dashboard_status()}")
    
    # 모터 콜백 함수 설정
    def motor_callback(command):
        print(f"Motor Command: {command}")
    
    dashboard.set_motor_callback(motor_callback)
    
    # 대시보드 시작
    print("Starting web dashboard...")
    print(f"Access at: http://localhost:{dashboard.dashboard_config['port']}")
    
    try:
        dashboard.start()
    except KeyboardInterrupt:
        print("\nStopping dashboard...")
        dashboard.stop()
    
    print("Web dashboard test completed!")
