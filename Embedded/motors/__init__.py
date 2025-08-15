"""
Motors Package - Integrated Motor Control System
"""

from .ai_motor_controller import AIMotorController
from .amr_motor_controller import AMRMotorController
from .integrated_motor_control import IntegratedMotorControl
from .motor_control_enhanced import MotorDriverHAT
from .servo_motor_controller import ServoMotorController
from .PCA9685 import PCA9685

__all__ = [
    'AIMotorController',
    'AMRMotorController', 
    'IntegratedMotorControl',
    'MotorDriverHAT',
    'ServoMotorController',
    'PCA9685'
]

__version__ = '2.0.0'
__author__ = 'AMR Development Team' 