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
            name='driver',
            output='screen',
            # arguments=[
            #     '--angle-map-in', '5.7',
            #     '--angle-map-out', '50',
            #     '--angle-limit', '50',
            #     '--speed-limit', '100'
            # ],
        ),
    ])
