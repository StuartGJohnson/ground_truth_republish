from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration('params_file', default='')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='',
            description='Path to YAML with parameters'),

        Node(
            package='ground_truth_republish',
            executable='ground_truth_republish_node',
            name='ground_truth_republish_node',
            parameters=[params_file],
            output='screen'
        )
    ])
