import os
import sys
import hashlib
import hmac
import json
import logging
import threading
import time
from typing import Dict, List, Optional, Tuple
from pathlib import Path
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import rsa, padding
from cryptography.hazmat.primitives import serialization
from cryptography.exceptions import InvalidSignature
import base64

sys.path.append(str(Path(__file__).parent.parent))
from security.security_manager import SecurityManager

class FirmwareSecurityManager:
    """
    펌웨어 보안 관리 클래스
    - 펌웨어 무결성 검증
    - 펌웨어 업데이트 보안
    - 익스텐션 관리
    """
    
    def __init__(self):
        self.security_manager = SecurityManager()
        self.logger = logging.getLogger('firmware_security')
        
        # 펌웨어 관련 설정
        self.firmware_config = {
            'firmware_dir': Path('firmware'),
            'backup_dir': Path('firmware/backup'),
            'extensions_dir': Path('firmware/extensions'),
            'signature_required': True,
            'backup_required': True,
            'rollback_enabled': True,
            'max_firmware_size': 100 * 1024 * 1024,  # 100MB
            'allowed_extensions': ['.bin', '.hex', '.elf', '.py', '.so']
        }
        
        # 펌웨어 상태 관리
        self.current_firmware = None
        self.firmware_history = []
        self.extension_registry = {}
        
        # 보안 키 관리
        self.signing_key = None
        self.verification_key = None
        
        self.lock = threading.Lock()
        
        self._initialize_firmware_security()
    
    def _initialize_firmware_security(self):
        """펌웨어 보안 초기화"""
        try:
            # 디렉토리 생성
            self.firmware_config['firmware_dir'].mkdir(exist_ok=True)
            self.firmware_config['backup_dir'].mkdir(exist_ok=True)
            self.firmware_config['extensions_dir'].mkdir(exist_ok=True)
            
            # 키 생성 또는 로드
            self._setup_crypto_keys()
            
            # 현재 펌웨어 정보 로드
            self._load_current_firmware_info()
            
            # 익스텐션 레지스트리 로드
            self._load_extension_registry()
            
            self.logger.info("Firmware security manager initialized successfully")
            
        except Exception as e:
            self.logger.error(f"Firmware security initialization failed: {e}")
            raise
    
    def _setup_crypto_keys(self):
        """암호화 키 설정"""
        key_file = Path('security/firmware_keys.pem')
        
        if key_file.exists():
            # 기존 키 로드
            with open(key_file, 'rb') as f:
                private_key = serialization.load_pem_private_key(
                    f.read(),
                    password=None
                )
                self.signing_key = private_key
                self.verification_key = private_key.public_key()
        else:
            # 새 키 생성
            self.signing_key = rsa.generate_private_key(
                public_exponent=65537,
                key_size=2048
            )
            self.verification_key = self.signing_key.public_key()
            
            # 키 저장
            with open(key_file, 'wb') as f:
                f.write(self.signing_key.private_bytes(
                    encoding=serialization.Encoding.PEM,
                    format=serialization.PrivateFormat.PKCS8,
                    encryption_algorithm=serialization.NoEncryption()
                ))
    
    def verify_firmware_integrity(self, firmware_path: Path) -> Tuple[bool, str]:
        """
        펌웨어 무결성 검증
        
        Args:
            firmware_path: 펌웨어 파일 경로
            
        Returns:
            (검증 성공 여부, 오류 메시지)
        """
        try:
            if not firmware_path.exists():
                return False, "Firmware file not found"
            
            # 파일 크기 검증
            file_size = firmware_path.stat().st_size
            if file_size > self.firmware_config['max_firmware_size']:
                return False, f"Firmware file too large: {file_size} bytes"
            
            # 파일 확장자 검증
            if firmware_path.suffix not in self.firmware_config['allowed_extensions']:
                return False, f"Invalid file extension: {firmware_path.suffix}"
            
            # 해시 검증
            if not self._verify_firmware_hash(firmware_path):
                return False, "Firmware hash verification failed"
            
            # 서명 검증
            if self.firmware_config['signature_required']:
                if not self._verify_firmware_signature(firmware_path):
                    return False, "Firmware signature verification failed"
            
            return True, "Firmware integrity verified"
            
        except Exception as e:
            self.logger.error(f"Firmware integrity verification failed: {e}")
            return False, str(e)
    
    def _verify_firmware_hash(self, firmware_path: Path) -> bool:
        """펌웨어 해시 검증"""
        try:
            # 해시 파일 경로
            hash_file = firmware_path.with_suffix(firmware_path.suffix + '.hash')
            
            if not hash_file.exists():
                self.logger.warning(f"Hash file not found: {hash_file}")
                return False
            
            # 현재 파일 해시 계산
            with open(firmware_path, 'rb') as f:
                current_hash = hashlib.sha256(f.read()).hexdigest()
            
            # 저장된 해시 읽기
            with open(hash_file, 'r') as f:
                stored_hash = f.read().strip()
            
            return current_hash == stored_hash
            
        except Exception as e:
            self.logger.error(f"Hash verification failed: {e}")
            return False
    
    def _verify_firmware_signature(self, firmware_path: Path) -> bool:
        """펌웨어 서명 검증"""
        try:
            # 서명 파일 경로
            signature_file = firmware_path.with_suffix(firmware_path.suffix + '.sig')
            
            if not signature_file.exists():
                self.logger.warning(f"Signature file not found: {signature_file}")
                return False
            
            # 펌웨어 파일 읽기
            with open(firmware_path, 'rb') as f:
                firmware_data = f.read()
            
            # 서명 읽기
            with open(signature_file, 'rb') as f:
                signature = f.read()
            
            # 서명 검증
            self.verification_key.verify(
                signature,
                firmware_data,
                padding.PSS(
                    mgf=padding.MGF1(hashes.SHA256()),
                    salt_length=padding.PSS.MAX_LENGTH
                ),
                hashes.SHA256()
            )
            
            return True
            
        except InvalidSignature:
            self.logger.error("Invalid firmware signature")
            return False
        except Exception as e:
            self.logger.error(f"Signature verification failed: {e}")
            return False
    
    def install_firmware(self, firmware_path: Path, backup: bool = True) -> Tuple[bool, str]:
        """
        펌웨어 설치
        
        Args:
            firmware_path: 설치할 펌웨어 파일 경로
            backup: 백업 생성 여부
            
        Returns:
            (설치 성공 여부, 오류 메시지)
        """
        with self.lock:
            try:
                # 무결성 검증
                is_valid, error_msg = self.verify_firmware_integrity(firmware_path)
                if not is_valid:
                    return False, error_msg
                
                # 백업 생성
                if backup and self.firmware_config['backup_required']:
                    if not self._create_firmware_backup():
                        return False, "Failed to create firmware backup"
                
                # 펌웨어 설치
                if not self._install_firmware_file(firmware_path):
                    return False, "Failed to install firmware"
                
                # 펌웨어 정보 업데이트
                self._update_firmware_info(firmware_path)
                
                self.logger.info(f"Firmware installed successfully: {firmware_path}")
                return True, "Firmware installed successfully"
                
            except Exception as e:
                self.logger.error(f"Firmware installation failed: {e}")
                return False, str(e)
    
    def _create_firmware_backup(self) -> bool:
        """펌웨어 백업 생성"""
        try:
            if self.current_firmware and self.current_firmware['path'].exists():
                backup_path = self.firmware_config['backup_dir'] / f"backup_{int(time.time())}.bin"
                
                # 현재 펌웨어 복사
                import shutil
                shutil.copy2(self.current_firmware['path'], backup_path)
                
                # 백업 정보 저장
                backup_info = {
                    'path': backup_path,
                    'timestamp': time.time(),
                    'original_path': str(self.current_firmware['path']),
                    'hash': self.current_firmware['hash']
                }
                
                with open(backup_path.with_suffix('.json'), 'w') as f:
                    json.dump(backup_info, f, indent=2)
                
                self.logger.info(f"Firmware backup created: {backup_path}")
                return True
                
        except Exception as e:
            self.logger.error(f"Backup creation failed: {e}")
            return False
    
    def _install_firmware_file(self, firmware_path: Path) -> bool:
        """펌웨어 파일 설치"""
        try:
            # 설치 경로
            install_path = self.firmware_config['firmware_dir'] / firmware_path.name
            
            # 기존 파일 백업
            if install_path.exists():
                backup_path = install_path.with_suffix('.bak')
                import shutil
                shutil.move(str(install_path), str(backup_path))
            
            # 새 펌웨어 복사
            import shutil
            shutil.copy2(firmware_path, install_path)
            
            # 실행 권한 설정 (필요한 경우)
            if firmware_path.suffix in ['.py', '.sh']:
                os.chmod(install_path, 0o755)
            
            return True
            
        except Exception as e:
            self.logger.error(f"Firmware file installation failed: {e}")
            return False
    
    def install_extension(self, extension_path: Path) -> Tuple[bool, str]:
        """
        펌웨어 익스텐션 설치
        
        Args:
            extension_path: 익스텐션 파일 경로
            
        Returns:
            (설치 성공 여부, 오류 메시지)
        """
        with self.lock:
            try:
                # 익스텐션 무결성 검증
                is_valid, error_msg = self.verify_firmware_integrity(extension_path)
                if not is_valid:
                    return False, error_msg
                
                # 익스텐션 정보 추출
                extension_info = self._extract_extension_info(extension_path)
                if not extension_info:
                    return False, "Failed to extract extension information"
                
                # 호환성 검증
                if not self._verify_extension_compatibility(extension_info):
                    return False, "Extension compatibility check failed"
                
                # 익스텐션 설치
                install_path = self.firmware_config['extensions_dir'] / extension_path.name
                import shutil
                shutil.copy2(extension_path, install_path)
                
                # 익스텐션 레지스트리 업데이트
                self.extension_registry[extension_info['name']] = {
                    'path': str(install_path),
                    'version': extension_info['version'],
                    'installed_at': time.time(),
                    'enabled': True
                }
                
                # 레지스트리 저장
                self._save_extension_registry()
                
                self.logger.info(f"Extension installed successfully: {extension_info['name']}")
                return True, "Extension installed successfully"
                
            except Exception as e:
                self.logger.error(f"Extension installation failed: {e}")
                return False, str(e)
    
    def _extract_extension_info(self, extension_path: Path) -> Optional[Dict]:
        """익스텐션 정보 추출"""
        try:
            # 메타데이터 파일 확인
            meta_file = extension_path.with_suffix('.meta')
            
            if meta_file.exists():
                with open(meta_file, 'r') as f:
                    return json.load(f)
            
            # 파일명에서 정보 추출
            filename = extension_path.stem
            if '_' in filename:
                name, version = filename.rsplit('_', 1)
                return {
                    'name': name,
                    'version': version,
                    'type': extension_path.suffix[1:],
                    'description': f"Extension {name} version {version}"
                }
            
            return {
                'name': filename,
                'version': '1.0.0',
                'type': extension_path.suffix[1:],
                'description': f"Extension {filename}"
            }
            
        except Exception as e:
            self.logger.error(f"Extension info extraction failed: {e}")
            return None
    
    def _verify_extension_compatibility(self, extension_info: Dict) -> bool:
        """익스텐션 호환성 검증"""
        try:
            # 현재 펌웨어 버전 확인
            if not self.current_firmware:
                return True  # 펌웨어가 없으면 모든 익스텐션 허용
            
            # 호환성 정보 확인
            if 'compatibility' in extension_info:
                required_version = extension_info['compatibility'].get('firmware_version')
                if required_version and self.current_firmware['version'] < required_version:
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Extension compatibility check failed: {e}")
            return False
    
    def enable_extension(self, extension_name: str) -> bool:
        """익스텐션 활성화"""
        try:
            if extension_name in self.extension_registry:
                self.extension_registry[extension_name]['enabled'] = True
                self._save_extension_registry()
                self.logger.info(f"Extension enabled: {extension_name}")
                return True
            return False
        except Exception as e:
            self.logger.error(f"Extension enable failed: {e}")
            return False
    
    def disable_extension(self, extension_name: str) -> bool:
        """익스텐션 비활성화"""
        try:
            if extension_name in self.extension_registry:
                self.extension_registry[extension_name]['enabled'] = False
                self._save_extension_registry()
                self.logger.info(f"Extension disabled: {extension_name}")
                return True
            return False
        except Exception as e:
            self.logger.error(f"Extension disable failed: {e}")
            return False
    
    def remove_extension(self, extension_name: str) -> bool:
        """익스텐션 제거"""
        try:
            if extension_name in self.extension_registry:
                extension_info = self.extension_registry[extension_name]
                
                # 파일 삭제
                extension_path = Path(extension_info['path'])
                if extension_path.exists():
                    extension_path.unlink()
                
                # 레지스트리에서 제거
                del self.extension_registry[extension_name]
                self._save_extension_registry()
                
                self.logger.info(f"Extension removed: {extension_name}")
                return True
            return False
        except Exception as e:
            self.logger.error(f"Extension removal failed: {e}")
            return False
    
    def get_extension_status(self) -> Dict:
        """익스텐션 상태 조회"""
        return {
            'total_extensions': len(self.extension_registry),
            'enabled_extensions': len([e for e in self.extension_registry.values() if e['enabled']]),
            'extensions': self.extension_registry
        }
    
    def _load_current_firmware_info(self):
        """현재 펌웨어 정보 로드"""
        try:
            info_file = self.firmware_config['firmware_dir'] / 'firmware_info.json'
            if info_file.exists():
                with open(info_file, 'r') as f:
                    self.current_firmware = json.load(f)
        except Exception as e:
            self.logger.error(f"Failed to load firmware info: {e}")
    
    def _update_firmware_info(self, firmware_path: Path):
        """펌웨어 정보 업데이트"""
        try:
            # 해시 계산
            with open(firmware_path, 'rb') as f:
                firmware_hash = hashlib.sha256(f.read()).hexdigest()
            
            # 정보 업데이트
            self.current_firmware = {
                'path': str(firmware_path),
                'name': firmware_path.name,
                'version': self._extract_version_from_filename(firmware_path.name),
                'hash': firmware_hash,
                'installed_at': time.time(),
                'size': firmware_path.stat().st_size
            }
            
            # 정보 저장
            info_file = self.firmware_config['firmware_dir'] / 'firmware_info.json'
            with open(info_file, 'w') as f:
                json.dump(self.current_firmware, f, indent=2)
                
        except Exception as e:
            self.logger.error(f"Failed to update firmware info: {e}")
    
    def _extract_version_from_filename(self, filename: str) -> str:
        """파일명에서 버전 추출"""
        try:
            # 버전 패턴 찾기 (예: firmware_v1.2.3.bin)
            import re
            version_match = re.search(r'v(\d+\.\d+\.\d+)', filename)
            if version_match:
                return version_match.group(1)
            return '1.0.0'
        except:
            return '1.0.0'
    
    def _load_extension_registry(self):
        """익스텐션 레지스트리 로드"""
        try:
            registry_file = self.firmware_config['extensions_dir'] / 'registry.json'
            if registry_file.exists():
                with open(registry_file, 'r') as f:
                    self.extension_registry = json.load(f)
        except Exception as e:
            self.logger.error(f"Failed to load extension registry: {e}")
    
    def _save_extension_registry(self):
        """익스텐션 레지스트리 저장"""
        try:
            registry_file = self.firmware_config['extensions_dir'] / 'registry.json'
            with open(registry_file, 'w') as f:
                json.dump(self.extension_registry, f, indent=2)
        except Exception as e:
            self.logger.error(f"Failed to save extension registry: {e}")

# 전역 인스턴스
_firmware_security_manager = None

def get_firmware_security_manager() -> FirmwareSecurityManager:
    """펌웨어 보안 매니저 인스턴스 반환"""
    global _firmware_security_manager
    if _firmware_security_manager is None:
        _firmware_security_manager = FirmwareSecurityManager()
    return _firmware_security_manager
