import os
import sys
import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from flask import Flask, request, jsonify, send_file
from werkzeug.utils import secure_filename
import threading

sys.path.append(str(Path(__file__).parent.parent))
from security.firmware_security import get_firmware_security_manager

app = Flask(__name__)
app.config['MAX_CONTENT_LENGTH'] = 100 * 1024 * 1024  # 100MB

# 펌웨어 보안 매니저
firmware_manager = get_firmware_security_manager()

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@app.route('/api/firmware/status', methods=['GET'])
def get_firmware_status():
    """현재 펌웨어 상태 조회"""
    try:
        status = {
            'current_firmware': firmware_manager.current_firmware,
            'extension_status': firmware_manager.get_extension_status(),
            'config': firmware_manager.firmware_config
        }
        return jsonify({'success': True, 'data': status})
    except Exception as e:
        logger.error(f"Failed to get firmware status: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/firmware/install', methods=['POST'])
def install_firmware():
    """펌웨어 설치"""
    try:
        if 'firmware' not in request.files:
            return jsonify({'success': False, 'error': 'No firmware file provided'}), 400
        
        file = request.files['firmware']
        if file.filename == '':
            return jsonify({'success': False, 'error': 'No file selected'}), 400
        
        # 파일 저장
        filename = secure_filename(file.filename)
        temp_path = Path('/tmp') / filename
        file.save(str(temp_path))
        
        # 백업 여부 확인
        backup = request.form.get('backup', 'true').lower() == 'true'
        
        # 펌웨어 설치
        success, message = firmware_manager.install_firmware(temp_path, backup=backup)
        
        # 임시 파일 삭제
        temp_path.unlink(missing_ok=True)
        
        if success:
            return jsonify({'success': True, 'message': message})
        else:
            return jsonify({'success': False, 'error': message}), 400
            
    except Exception as e:
        logger.error(f"Firmware installation failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/firmware/verify', methods=['POST'])
def verify_firmware():
    """펌웨어 무결성 검증"""
    try:
        if 'firmware' not in request.files:
            return jsonify({'success': False, 'error': 'No firmware file provided'}), 400
        
        file = request.files['firmware']
        if file.filename == '':
            return jsonify({'success': False, 'error': 'No file selected'}), 400
        
        # 파일 저장
        filename = secure_filename(file.filename)
        temp_path = Path('/tmp') / filename
        file.save(str(temp_path))
        
        # 무결성 검증
        success, message = firmware_manager.verify_firmware_integrity(temp_path)
        
        # 임시 파일 삭제
        temp_path.unlink(missing_ok=True)
        
        if success:
            return jsonify({'success': True, 'message': message})
        else:
            return jsonify({'success': False, 'error': message}), 400
            
    except Exception as e:
        logger.error(f"Firmware verification failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/extensions', methods=['GET'])
def get_extensions():
    """설치된 익스텐션 목록 조회"""
    try:
        status = firmware_manager.get_extension_status()
        return jsonify({'success': True, 'data': status})
    except Exception as e:
        logger.error(f"Failed to get extensions: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/extensions/install', methods=['POST'])
def install_extension():
    """익스텐션 설치"""
    try:
        if 'extension' not in request.files:
            return jsonify({'success': False, 'error': 'No extension file provided'}), 400
        
        file = request.files['extension']
        if file.filename == '':
            return jsonify({'success': False, 'error': 'No file selected'}), 400
        
        # 파일 저장
        filename = secure_filename(file.filename)
        temp_path = Path('/tmp') / filename
        file.save(str(temp_path))
        
        # 익스텐션 설치
        success, message = firmware_manager.install_extension(temp_path)
        
        # 임시 파일 삭제
        temp_path.unlink(missing_ok=True)
        
        if success:
            return jsonify({'success': True, 'message': message})
        else:
            return jsonify({'success': False, 'error': message}), 400
            
    except Exception as e:
        logger.error(f"Extension installation failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/extensions/<extension_name>/enable', methods=['POST'])
def enable_extension(extension_name):
    """익스텐션 활성화"""
    try:
        success = firmware_manager.enable_extension(extension_name)
        if success:
            return jsonify({'success': True, 'message': f'Extension {extension_name} enabled'})
        else:
            return jsonify({'success': False, 'error': f'Extension {extension_name} not found'}), 404
    except Exception as e:
        logger.error(f"Extension enable failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/extensions/<extension_name>/disable', methods=['POST'])
def disable_extension(extension_name):
    """익스텐션 비활성화"""
    try:
        success = firmware_manager.disable_extension(extension_name)
        if success:
            return jsonify({'success': True, 'message': f'Extension {extension_name} disabled'})
        else:
            return jsonify({'success': False, 'error': f'Extension {extension_name} not found'}), 404
    except Exception as e:
        logger.error(f"Extension disable failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/extensions/<extension_name>', methods=['DELETE'])
def remove_extension(extension_name):
    """익스텐션 제거"""
    try:
        success = firmware_manager.remove_extension(extension_name)
        if success:
            return jsonify({'success': True, 'message': f'Extension {extension_name} removed'})
        else:
            return jsonify({'success': False, 'error': f'Extension {extension_name} not found'}), 404
    except Exception as e:
        logger.error(f"Extension removal failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/extensions/<extension_name>/modify', methods=['POST'])
def modify_extension(extension_name):
    """익스텐션 수정"""
    try:
        if extension_name not in firmware_manager.extension_registry:
            return jsonify({'success': False, 'error': f'Extension {extension_name} not found'}), 404
        
        # 수정할 내용 받기
        data = request.get_json()
        if not data:
            return jsonify({'success': False, 'error': 'No modification data provided'}), 400
        
        # 익스텐션 정보 업데이트
        extension_info = firmware_manager.extension_registry[extension_name]
        
        # 허용된 필드만 수정
        allowed_fields = ['description', 'version', 'enabled']
        for field in allowed_fields:
            if field in data:
                extension_info[field] = data[field]
        
        # 레지스트리 저장
        firmware_manager._save_extension_registry()
        
        return jsonify({
            'success': True, 
            'message': f'Extension {extension_name} modified successfully',
            'data': extension_info
        })
        
    except Exception as e:
        logger.error(f"Extension modification failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/extensions/<extension_name>/config', methods=['GET', 'POST'])
def extension_config(extension_name):
    """익스텐션 설정 관리"""
    try:
        if extension_name not in firmware_manager.extension_registry:
            return jsonify({'success': False, 'error': f'Extension {extension_name} not found'}), 404
        
        extension_info = firmware_manager.extension_registry[extension_name]
        config_file = Path(extension_info['path']).with_suffix('.config')
        
        if request.method == 'GET':
            # 설정 파일 읽기
            if config_file.exists():
                with open(config_file, 'r') as f:
                    config = json.load(f)
                return jsonify({'success': True, 'data': config})
            else:
                return jsonify({'success': True, 'data': {}})
        
        elif request.method == 'POST':
            # 설정 파일 업데이트
            data = request.get_json()
            if not data:
                return jsonify({'success': False, 'error': 'No configuration data provided'}), 400
            
            with open(config_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            return jsonify({
                'success': True, 
                'message': f'Configuration updated for {extension_name}'
            })
            
    except Exception as e:
        logger.error(f"Extension config failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/firmware/backup', methods=['GET'])
def list_backups():
    """백업 목록 조회"""
    try:
        backup_dir = firmware_manager.firmware_config['backup_dir']
        backups = []
        
        for backup_file in backup_dir.glob('*.bin'):
            info_file = backup_file.with_suffix('.json')
            if info_file.exists():
                with open(info_file, 'r') as f:
                    backup_info = json.load(f)
                backups.append(backup_info)
        
        return jsonify({'success': True, 'data': backups})
    except Exception as e:
        logger.error(f"Failed to list backups: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/firmware/rollback/<backup_timestamp>', methods=['POST'])
def rollback_firmware(backup_timestamp):
    """펌웨어 롤백"""
    try:
        backup_dir = firmware_manager.firmware_config['backup_dir']
        backup_file = backup_dir / f"backup_{backup_timestamp}.bin"
        
        if not backup_file.exists():
            return jsonify({'success': False, 'error': 'Backup not found'}), 404
        
        # 롤백 실행
        success, message = firmware_manager.install_firmware(backup_file, backup=False)
        
        if success:
            return jsonify({'success': True, 'message': f'Rollback to {backup_timestamp} successful'})
        else:
            return jsonify({'success': False, 'error': message}), 400
            
    except Exception as e:
        logger.error(f"Firmware rollback failed: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/firmware/logs', methods=['GET'])
def get_firmware_logs():
    """펌웨어 관련 로그 조회"""
    try:
        log_file = Path('logs/firmware_security.log')
        if not log_file.exists():
            return jsonify({'success': True, 'data': []})
        
        # 최근 로그 읽기
        lines = request.args.get('lines', 100, type=int)
        with open(log_file, 'r') as f:
            log_lines = f.readlines()[-lines:]
        
        return jsonify({'success': True, 'data': log_lines})
    except Exception as e:
        logger.error(f"Failed to get firmware logs: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=False)
