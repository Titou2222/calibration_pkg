from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calibration_pkg',
            executable='save_pointcloud',
            output='screen'),
    ])