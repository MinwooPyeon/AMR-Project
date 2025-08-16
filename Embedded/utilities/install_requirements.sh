sudo apt update && sudo apt upgrade -y

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

sudo apt install -y \
    libopencv-dev \
    libopencv-core-dev \
    libopencv-imgproc-dev \
    libopencv-highgui-dev

sudo apt install -y \
    libwebsocketpp-dev \
    libjsoncpp-dev \
    libasio-dev

sudo apt install -y libgpiod-dev

echo "C++ dependencies installation complete"
echo "This is now a C++ only project."
