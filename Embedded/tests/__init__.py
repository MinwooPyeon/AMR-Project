#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AMR 시스템 테스트 패키지
"""

__version__ = "1.0.0"
__author__ = "AMR Team"

# 테스트 카테고리별 import
from .unit import *
from .integration import *
from .e2e import *
