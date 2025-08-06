#!/bin/bash

# 1. APT 패키지 최신화
sudo apt update && sudo apt upgrade -y

# 2. ROS2 (Humble) 기반 의존성
sudo apt install -y \
    ros-humble-rclcpp \
    ros-humble-rclcpp-action \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav2-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rosbridge-server \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox

# 3. OpenCV
sudo apt install -y \
    libopencv-dev \
    libopencv-core-dev \
    libopencv-imgproc-dev \
    libopencv-highgui-dev

# 4. WebSocket 
sudo apt install -y \
    libwebsocketpp-dev \
    libjsoncpp-dev \
    libasio-dev

# 5. GPIO / PWM 제어용 
sudo apt install -y libgpiod-dev

# 6. C++ 전용 프로젝트로 변경됨
# Python 의존성 제거됨

echo "C++ 의존성 설치 완료"
echo "이제 C++ 전용 프로젝트입니다."
