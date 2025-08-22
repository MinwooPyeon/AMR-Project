# Terminal 1
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```
# Terminal 2
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
# Terminal 3
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link
```
# Terminal 4
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
```
# Terminal 5
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link base_footprint
```
# Terminal 6
```bash
ros2 launch nav2_bringup bringup_launch.py map:=/home/ssafy/ros2_ws/map/my_map.yaml
```
# Terminal 7
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```