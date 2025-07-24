CXX = g++
CXXFLAGS = -Wall -O2 -std=c++17 -Iinclude `pkg-config --cflags opencv4`
LDFLAGS = -lpthread -lgpiod `pkg-config --libs opencv4`

SRC = \
  main.cpp \
  src/lidar_sensor.cpp \
  src/led_driver.cpp \
  src/motor_driver.cpp \
  src/motor_cmd_interface.cpp \
  src/backend_ws_client.cpp \
  src/backup_manager.cpp \
  src/ros/sensor_data_publisher.cpp \
  src/ros/motor_cmd_subscriber.cpp \
  src/battery/battery_publisher.cpp \
  src/camera/webcam_publisher.cpp \
  src/slam/visual_lidar_slam_node.cpp \
  src/navigation/navigation_client.cpp

OBJ = $(SRC:.cpp=.o)
TARGET = amr_app

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $@ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
