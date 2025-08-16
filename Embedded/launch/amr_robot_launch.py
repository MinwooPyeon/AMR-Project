from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_project',
            executable='visual_lidar_slam_node',
            name='visual_lidar_slam_node',
            parameters=['config/slam_params.yaml']
        ),



        Node(
            package='amr_project',
            executable='webcam_publisher',
            name='webcam_publisher',
            parameters=['config/camera_config.yaml']
        ),

        Node(
            package='amr_project',
            executable='navigation_client',
            name='navigation_client',
            parameters=['config/nav2_params.yaml']
        ),

        Node(
            package='amr_project',
            executable='sensor_data_publisher_node',
            name='sensor_data_publisher'
        ),

        Node(
            package='amr_project',
            executable='imu_data_publisher_node',
            name='imu_data_publisher',
            parameters=[{
                'frame_id': 'imu_link',
                'publish_rate': 50.0,
                'enable_temperature': True,
                'imu_type': 4,
                'i2c_address': 0x4B
            }]
        ),

        Node(
            package='amr_project',
            executable='motor_cmd_subscriber_node',
            name='motor_cmd_subscriber'
        ),

        Node(
            package='amr_project',
            executable='angle_control_node',
            name='angle_control_node',
            parameters=[{
                'imu_type': 'BNO08X',
                'imu_i2c_address': 0x4B,
                'motor_i2c_address': 0x40,
                'angle_tolerance': 2.0,
                'turn_speed': 50.0,
                'control_frequency': 50.0,
                'pid_kp': 2.0,
                'pid_ki': 0.1,
                'pid_kd': 0.5,
                'enable_smoothing': True,
                'smoothing_factor': 0.8,
                'cmd_vel_topic': 'cmd_vel',
                'target_angle_topic': 'target_angle',
                'enable_topic': 'angle_control_enable',
                'command_topic': 'angle_control_command',
                'imu_topic': 'imu/data',
                'angle_topic': 'current_angle',
                'target_angle_pub_topic': 'target_angle_pub',
                'angle_error_topic': 'angle_error',
                'status_topic': 'angle_control_status'
            }]
        ),

        Node(
            package='amr_project',
            executable='amr_main',
            name='amr_main'
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                "tcp_nodelay": True
            }]
        )
    ])
