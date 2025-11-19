from launch import LaunchDescription
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='my_motor',
            executable='go',
            name='udp_receiver',
            output='screen',
            arguments=[
                '--topic', '/UDP_RECV_CMD',
            ],
        ),
        Node(
            package='my_motor',
            executable='joystick',
            name='joystick_driver',
            output='screen',
            arguments=[
                '--topic', '/JOYSTICK_CMD',
            ],
        ),
        Node(
            package='my_motor',
            executable='command_mux',
            name='command_mux',
            output='screen',
            arguments=[
                '--udp-topic', '/UDP_RECV_CMD',
                '--joystick-topic', '/JOYSTICK_CMD',
                '--output-topic', '/xycar_motor',
            ],
        ),
    ])
