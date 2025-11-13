from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration('params_file', default='')
    source_topic = LaunchConfiguration('source_topic', default='/ground_truth_pose')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    publish_tf = LaunchConfiguration('publish_tf', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value='',
            description='Path to YAML with parameters'),
        DeclareLaunchArgument('source_topic', default_value='/ground_truth_pose'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('publish_tf', default_value='true'),

        Node(
            package='ground_truth_republish',
            executable='ground_truth_republish_node',
            name='ground_truth_republish_node',
            parameters=[params_file,
                        {'source_topic': source_topic,
                         'odom_topic': odom_topic,
                         'publish_tf': publish_tf}],
            output='screen'
        )
    ])
