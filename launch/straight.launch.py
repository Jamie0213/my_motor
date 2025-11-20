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
            executable='straight',
            name='straight_driver',
            output='screen',
            parameters=[
                {'speed': 4}
            ],
        ),
    ])

