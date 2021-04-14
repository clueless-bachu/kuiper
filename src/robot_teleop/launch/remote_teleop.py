from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='motor_control',
            node_executable='driver'
        ),

        Node(
            package='robot_teleop',
            node_executable='teleop_internet'
        ),

        Node(
            package='intel_camera',
            node_executable='sensor'
        ),

        Node(
            package='intel_camera',
            node_executable='post_img'
        ),

        ])
