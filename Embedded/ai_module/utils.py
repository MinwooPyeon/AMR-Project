import json
import base64
import os
import logging
from datetime import datetime
from typing import Dict, Any, Optional
from PIL import Image
import io

from .config import AIConfig

def setup_logger(name: str, level: str = None) -> logging.Logger:
    logger = logging.getLogger(name)
    
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(AIConfig.LOG_FORMAT)
        handler.setFormatter(formatter)
        logger.addHandler(handler)
    
    logger.setLevel(getattr(logging, level or AIConfig.LOG_LEVEL))
    return logger

def encode_image_to_base64(image_path: str) -> Optional[str]:
    try:
        with open(image_path, "rb") as image_file:
            encoded_string = base64.b64encode(image_file.read()).decode('utf-8')
            return encoded_string
    except Exception as e:
        logging.error(f"Image encoding failed: {e}")
        return None

def decode_base64_to_image(base64_string: str, save_path: str = None) -> Optional[str]:
    try:
        image_data = base64.b64decode(base64_string)
        image = Image.open(io.BytesIO(image_data))
        
        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            image.save(save_path)
            return save_path
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"ai_image_{timestamp}.jpg"
            filepath = os.path.join(AIConfig.AI_IMAGES_PATH, filename)
            os.makedirs(AIConfig.AI_IMAGES_PATH, exist_ok=True)
            image.save(filepath)
            return filepath
            
    except Exception as e:
        logging.error(f"Image decoding failed: {e}")
        return None

def create_json_message(data: Dict[str, Any]) -> str:
    try:
        return json.dumps(data, ensure_ascii=False, indent=2)
    except Exception as e:
        logging.error(f"JSON message creation failed: {e}")
        return "{}"

def parse_json_message(message: str) -> Optional[Dict[str, Any]]:
    try:
        return json.loads(message)
    except json.JSONDecodeError as e:
        logging.error(f"JSON parsing failed: {e}")
        return None

def validate_ai_data(data: Dict[str, Any]) -> bool:
    required_fields = ["serial", "x", "y", "img", "case", "timeStamp"]
    
    for field in required_fields:
        if field not in data:
            logging.error(f"Required field missing: {field}")
            return False
    
    try:
        float(data["x"])
        float(data["y"])
    except (ValueError, TypeError):
        logging.error("Coordinate values are not numbers")
        return False
    
    return True

def generate_timestamp() -> str:
    return datetime.now().isoformat()

def get_file_modified_time(file_path: str) -> float:
    try:
        return os.path.getmtime(file_path)
    except OSError:
        return 0.0
