cmake_minimum_required(VERSION 3.10)
project(amr_project)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
# ls /usr/lib/aarch64-linux-gnu/cmake/ 로 위치 확인

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# MQTT 관련 패키지 & 시스템 라이브러리
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
find_package(nlohmann_json REQUIRED)

# WebSocket 라이브러리
find_package(websocketpp REQUIRED)
find_package(jsoncpp REQUIRED)

# libgpiod 라이브러리
pkg_check_modules(GPIOD REQUIRED libgpiod)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${MOSQUITTO_INCLUDE_DIRS}
  ${GPIOD_INCLUDE_DIRS}
)

add_executable(amr_main
  main.cpp
  src/backend_ws_client.cpp
  src/mqtt/mqtt_publisher.cpp
  src/mqtt/state_publisher.cpp
  src/mqtt/robot_data_publisher.cpp
  src/mqtt/image_publisher.cpp
  src/mqtt/device_data_builder.cpp
  src/lidar_sensor.cpp
  src/led_driver.cpp
  src/module/frame.cpp
  src/module/motor_driver.cpp
  src/module/motor_controller.cpp
  src/motor_cmd_interface.cpp
  src/backup_manager.cpp
  src/ros/sensor_data_publisher.cpp
  src/ros/motor_cmd_subscriber.cpp
  src/battery/battery_publisher.cpp
  src/camera/webcam_publisher.cpp
  src/slam/visual_lidar_slam_node.cpp
  src/navigation/navigation_client.cpp
  src/sysfsInterface/sysfs_gpio.cpp
  src/sysfsInterface/sysfs_pwm.cpp
  src/sysfsInterface/jetson_gpio.cpp
  src/i2c/MyI2CImplementation.cpp
  src/i2c/i2c_address_manager.cpp
  src/motor_test.cpp
)



ament_target_dependencies(amr_main
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  rclcpp_action
  nav2_msgs
  OpenCV
)

target_link_libraries(amr_main 
  websocketpp::websocketpp
  JsonCpp::JsonCpp
  ${MOSQUITTO_LIBRARIES}
  nlohmann_json::nlohmann_json
  ${GPIOD_LIBRARIES}
)

install(TARGETS amr_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
