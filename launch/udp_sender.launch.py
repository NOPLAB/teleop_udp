from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_host',
            default_value='127.0.0.1',
            description='Target host address'
        ),
        DeclareLaunchArgument(
            'target_port',
            default_value='12345',
            description='Target port number'
        ),
        Node(
            package='teleop_udp',
            executable='udp_sender_node',
            name='udp_sender',
            parameters=[{
                'target_host': LaunchConfiguration('target_host'),
                'target_port': LaunchConfiguration('target_port'),
            }],
            output='screen',
        ),
    ])
