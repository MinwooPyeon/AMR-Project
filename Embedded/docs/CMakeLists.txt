cmake_minimum_required(VERSION 3.10)
project(amr_project)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
# ls /usr/lib/aarch64-linux-gnu/cmake/ 로 위치 확인

# ROS2 환경 확인 및 설정
if(DEFINED ENV{ROS_DISTRO})
    find_package(ament_cmake REQUIRED)
    set(ROS2_AVAILABLE TRUE)
    message(STATUS "ROS2 환경이 감지되었습니다: $ENV{ROS_DISTRO}")
else()
    message(WARNING "ROS2 환경이 감지되지 않았습니다. ROS2 관련 기능은 비활성화됩니다.")
    set(ROS2_AVAILABLE FALSE)
    # ROS2 없이도 빌드할 수 있도록 기본 설정
    set(ament_cmake_FOUND FALSE)
endif()

# ROS2 관련 패키지들 조건부 처리
if(ROS2_AVAILABLE)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(rclcpp_action REQUIRED)
    find_package(nav2_msgs REQUIRED)
else()
    message(STATUS "ROS2 패키지들을 건너뜁니다.")
endif()

# OpenCV 조건부 처리
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    message(STATUS "OpenCV가 발견되었습니다: ${OpenCV_VERSION}")
else()
    message(WARNING "OpenCV가 발견되지 않았습니다. OpenCV 관련 기능은 비활성화됩니다.")
    set(OpenCV_LIBS "")
endif()

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

# 기본 소스 파일들
set(SOURCES
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
  src/battery/battery_publisher.cpp
  src/camera/webcam_publisher.cpp
  src/slam/visual_lidar_slam_node.cpp
  src/navigation/navigation_client.cpp
  src/sysfsInterface/sysfs_gpio.cpp
  src/sysfsInterface/sysfs_pwm.cpp
  src/sysfsInterface/jetson_gpio.cpp
  src/i2c/MyI2CImplementation.cpp
  src/i2c/i2c_address_manager.cpp
  src/imu_sensor.cpp
  src/obstacle_detector.cpp
  src/angle_controller.cpp
  src/ros/angle_control_node.cpp
  src/motor_test.cpp
  src/imu_motor_integration.cpp
)

# ROS2 관련 소스 파일들 조건부 추가
if(ROS2_AVAILABLE)
    list(APPEND SOURCES
      src/ros/sensor_data_publisher.cpp
      src/ros/motor_cmd_subscriber.cpp
      src/ros/imu_data_publisher.cpp
    )
    message(STATUS "ROS2 관련 소스 파일들을 포함합니다.")
else()
    message(STATUS "ROS2 관련 소스 파일들을 제외합니다.")
endif()

add_executable(amr_main ${SOURCES})



# ROS2 의존성 조건부 처리
if(ROS2_AVAILABLE)
    ament_target_dependencies(amr_main
      rclcpp
      sensor_msgs
      std_msgs
      cv_bridge
      rclcpp_action
      nav2_msgs
    )
else()
    message(STATUS "ROS2 의존성을 건너뜁니다.")
endif()

# OpenCV는 ROS2와 독립적으로 처리
target_link_libraries(amr_main ${OpenCV_LIBS})

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

# IMU-모터 통합 테스트 실행 파일
add_executable(imu_motor_test test_imu_motor_integration.cpp src/imu_motor_integration.cpp)

# 테스트 실행 파일 의존성
target_link_libraries(imu_motor_test 
  websocketpp::websocketpp
  JsonCpp::JsonCpp
  ${MOSQUITTO_LIBRARIES}
  nlohmann_json::nlohmann_json
  ${GPIOD_LIBRARIES}
)

# OpenCV는 ROS2와 독립적으로 처리
target_link_libraries(imu_motor_test ${OpenCV_LIBS})

install(TARGETS imu_motor_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ROS2 패키지 설정 조건부 처리
if(ROS2_AVAILABLE)
    ament_package()
else()
    message(STATUS "ament_package()를 건너뜁니다.")
endif()
