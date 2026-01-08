from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='0.0.0.0',
            description='Bind host address'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='12345',
            description='Listen port number'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='joy',
            description='Frame ID for Joy message'
        ),
        Node(
            package='teleop_udp',
            executable='udp_receiver_node',
            name='udp_receiver',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
            output='screen',
        ),
    ])
