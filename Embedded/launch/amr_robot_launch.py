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
            executable='motor_cmd_subscriber_node',
            name='motor_cmd_subscriber'
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
