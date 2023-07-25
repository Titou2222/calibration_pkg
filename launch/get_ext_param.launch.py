from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calibration_pkg',
            executable='get_ext_param',
            output='screen'),
    ])

