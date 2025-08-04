#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
백엔드 MQTT Subscriber
AMR에서 전송하는 데이터를 받고, 백엔드에서 명령을 보낼 수 있는 시스템
"""

import json
import time
import threading
import logging
from typing import Dict, Optional, Callable, Any
import paho.mqtt.client as mqtt

# 로깅 설정
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class BackendMQTTSubscriber:
    """백엔드 MQTT Subscriber 클래스"""
    
    def __init__(self, mqtt_broker: str = "192.168.100.141", mqtt_port: int = 1883):
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.mqtt_client_id = f"backend_subscriber_{int(time.time())}"
        
        # MQTT 클라이언트
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_connected = False
        
        # MQTT 콜백 설정
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_publish = self._on_mqtt_publish
        
        # 데이터 저장
        self.latest_amr_data = {}
        self.data_lock = threading.Lock()
        
        # 콜백 함수들
        self.amr_data_callback: Optional[Callable[[Dict], None]] = None
        self.command_callback: Optional[Callable[[Dict], None]] = None
        
        # 통계 정보
        self.stats_lock = threading.Lock()
        self.total_received = 0
        self.last_received_time = 0
        
        logger.info(f"Backend MQTT Subscriber 초기화 완료 - Broker: {mqtt_broker}:{mqtt_port}")
    
    def connect_mqtt(self) -> bool:
        """MQTT 브로커에 연결"""
        try:
            logger.info(f"MQTT 브로커에 연결 중: {self.mqtt_broker}:{self.mqtt_port}")
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            
            # 연결 대기
            timeout = 10
            start_time = time.time()
            while not self.mqtt_connected and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if self.mqtt_connected:
                logger.info("MQTT 브로커 연결 성공")
                return True
            else:
                logger.error("MQTT 연결 시간 초과")
                return False
                
        except Exception as e:
            logger.error(f"MQTT 연결 실패: {e}")
            return False
    
    def disconnect_mqtt(self):
        """MQTT 연결 해제"""
        if self.mqtt_connected:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_connected = False
            logger.info("MQTT 연결 해제")
    
    def subscribe_to_amr_data(self, robot_id: str = "AMR001"):
        """AMR 데이터 구독"""
        topic = f"status/{robot_id}"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            logger.info(f"AMR 데이터 구독 성공: {topic}")
            return True
        else:
            logger.error(f"AMR 데이터 구독 실패: {result[0]}")
            return False
    
    def subscribe_to_commands(self, robot_id: str = "AMR001"):
        """명령 구독 (AMR에서 받을 명령)"""
        topic = f"command/{robot_id}"
        result = self.mqtt_client.subscribe(topic, qos=1)
        
        if result[0] == mqtt.MQTT_ERR_SUCCESS:
            logger.info(f"명령 구독 성공: {topic}")
            return True
        else:
            logger.error(f"명령 구독 실패: {result[0]}")
            return False
    
    def publish_command(self, robot_id: str, command: Dict[str, Any]) -> bool:
        """AMR에 명령 전송"""
        if not self.mqtt_connected:
            logger.warning("MQTT 연결이 없어 명령을 전송할 수 없습니다")
            return False
        
        try:
            # 명령에 타임스탬프 추가
            command["timestamp"] = time.time()
            command["source"] = "backend"
            
            # JSON 데이터를 문자열로 변환
            json_str = json.dumps(command, ensure_ascii=False)
            
            # MQTT 토픽 설정
            topic = f"command/{robot_id}"
            
            # 데이터 전송
            result = self.mqtt_client.publish(topic, json_str, qos=1)
            
            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                logger.info(f"명령 전송 성공: {command}")
                return True
            else:
                logger.error(f"명령 전송 실패: {result.rc}")
                return False
                
        except Exception as e:
            logger.error(f"명령 전송 중 오류: {e}")
            return False
    
    def set_amr_data_callback(self, callback: Callable[[Dict], None]):
        """AMR 데이터 콜백 설정"""
        self.amr_data_callback = callback
        logger.info("AMR 데이터 콜백 설정 완료")
    
    def set_command_callback(self, callback: Callable[[Dict], None]):
        """명령 콜백 설정"""
        self.command_callback = callback
        logger.info("명령 콜백 설정 완료")
    
    def get_latest_amr_data(self) -> Dict:
        """최신 AMR 데이터 조회"""
        with self.data_lock:
            return self.latest_amr_data.copy()
    
    def get_reception_stats(self) -> Dict[str, Any]:
        """수신 통계 정보 조회"""
        with self.stats_lock:
            stats = {
                "total_received": self.total_received,
                "last_received_time": self.last_received_time,
                "mqtt_connected": self.mqtt_connected,
                "latest_data": self.get_latest_amr_data()
            }
        return stats
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT 연결 콜백"""
        if rc == 0:
            self.mqtt_connected = True
            logger.info(f"MQTT 브로커 연결 성공: {self.mqtt_broker}:{self.mqtt_port}")
        else:
            logger.error(f"MQTT 연결 실패. 코드: {rc}")
            self.mqtt_connected = False
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT 연결 해제 콜백"""
        self.mqtt_connected = False
        if rc != 0:
            logger.warning(f"MQTT 연결이 예기치 않게 끊어졌습니다. 코드: {rc}")
        else:
            logger.info("MQTT 연결이 정상적으로 해제되었습니다.")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """MQTT 메시지 수신 콜백"""
        try:
            # JSON 파싱
            data = json.loads(msg.payload.decode('utf-8'))
            topic = msg.topic
            
            logger.debug(f"메시지 수신 - 토픽: {topic}, 데이터: {data}")
            
            # 통계 업데이트
            with self.stats_lock:
                self.total_received += 1
                self.last_received_time = time.time()
            
            # 토픽에 따른 처리
            if topic.startswith("status/"):
                # AMR 상태 데이터
                with self.data_lock:
                    self.latest_amr_data = data
                
                # 콜백 호출
                if self.amr_data_callback:
                    self.amr_data_callback(data)
                    
            elif topic.startswith("command/"):
                # 명령 데이터
                if self.command_callback:
                    self.command_callback(data)
            
        except json.JSONDecodeError as e:
            logger.error(f"JSON 파싱 오류: {e}")
        except Exception as e:
            logger.error(f"메시지 처리 오류: {e}")
    
    def _on_mqtt_publish(self, client, userdata, mid):
        """MQTT 발행 완료 콜백"""
        logger.debug(f"MQTT 메시지 발행 완료. ID: {mid}")

def test_backend_mqtt_subscriber():
    """백엔드 MQTT Subscriber 테스트"""
    print("=== 백엔드 MQTT Subscriber 테스트 ===")
    print("AMR에서 전송하는 데이터를 수신하고 명령을 보낼 수 있습니다.")
    print("MQTT 브로커: 192.168.100.141:1883")
    print("=" * 60)
    
    # 백엔드 MQTT Subscriber 생성
    backend = BackendMQTTSubscriber("192.168.100.141", 1883)
    
    # AMR 데이터 콜백 설정
    def amr_data_callback(data):
        print(f"\r📡 AMR 데이터 수신: "
              f"시리얼={data.get('serial', 'N/A')} | "
              f"상태={data.get('status', 'N/A')} | "

              f"위치=({data.get('x', 0):.1f}, {data.get('y', 0):.1f}) | "
              f"속도={data.get('speed', 0):.1f}", end="")
    
    # 명령 콜백 설정
    def command_callback(data):
        print(f"\n📨 명령 수신: {data}")
    
    backend.set_amr_data_callback(amr_data_callback)
    backend.set_command_callback(command_callback)
    
    # MQTT 연결
    print("MQTT 브로커에 연결 중...")
    if not backend.connect_mqtt():
        print("❌ MQTT 연결 실패")
        return
    
    print("✅ MQTT 연결 성공")
    
    # AMR 데이터 구독
    print("AMR 데이터 구독 중...")
    if not backend.subscribe_to_amr_data("AMR001"):
        print("❌ AMR 데이터 구독 실패")
        return
    
    print("✅ AMR 데이터 구독 성공")
    
    # 명령 구독
    print("명령 구독 중...")
    if not backend.subscribe_to_commands("AMR001"):
        print("❌ 명령 구독 실패")
        return
    
    print("✅ 명령 구독 성공")
    
    print("\n데이터 수신 대기 중... (30초)")
    print("Ctrl+C로 종료하거나 명령을 입력하세요:")
    print("  - 'MOVE_FORWARD': 전진 명령")
    print("  - 'MOVE_BACKWARD': 후진 명령")
    print("  - 'ROTATE_LEFT': 좌회전 명령")
    print("  - 'ROTATE_RIGHT': 우회전 명령")
    print("  - 'stop': 정지 명령")
    print("  - 'custom': 사용자 정의 명령")
    
    start_time = time.time()
    
    try:
        while time.time() - start_time < 30:
            # 사용자 입력 확인 (비동기)
            try:
                import select
                import sys
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    command = input().strip()
                    
                    if command == "MOVE_FORWARD":
                        backend.publish_command("AMR001", {
                            "action": "MOVE_FORWARD",
                            "speed": 50.0
                        })
                    elif command == "MOVE_BACKWARD":
                        backend.publish_command("AMR001", {
                            "action": "MOVE_BACKWARD",
                            "speed": 50.0
                        })
                    elif command == "ROTATE_LEFT":
                        backend.publish_command("AMR001", {
                            "action": "ROTATE_LEFT",
                            "speed": 50.0
                        })
                    elif command == "ROTATE_RIGHT":
                        backend.publish_command("AMR001", {
                            "action": "ROTATE_RIGHT",
                            "speed": 50.0
                        })
                    elif command == "stop":
                        backend.publish_command("AMR001", {
                            "action": "stop_motor"
                        })
                    elif command == "custom":
                        print("사용자 정의 명령을 입력하세요 (JSON 형식):")
                        try:
                            custom_cmd = json.loads(input())
                            backend.publish_command("AMR001", custom_cmd)
                        except json.JSONDecodeError:
                            print("잘못된 JSON 형식입니다.")
                    else:
                        print(f"알 수 없는 명령: {command}")
                        
            except (EOFError, KeyboardInterrupt):
                break
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  테스트 중단됨")
    
    # 최종 통계 출력
    stats = backend.get_reception_stats()
    print(f"\n📊 수신 통계:")
    for key, value in stats.items():
        if key != "latest_data":
            print(f"  - {key}: {value}")
    
    # 최신 데이터 출력
    latest_data = stats.get("latest_data", {})
    if latest_data:
        print(f"\n📋 최신 AMR 데이터:")
        print(json.dumps(latest_data, indent=2, ensure_ascii=False))
    
    # 연결 해제
    backend.disconnect_mqtt()
    print("\n✅ 백엔드 MQTT Subscriber 정리 완료")

if __name__ == "__main__":
    test_backend_mqtt_subscriber() 