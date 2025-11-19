from launch import LaunchDescription
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ultrasonic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('xycar_ultrasonic'),
                'launch/xycar_ultrasonic.launch.py'))
    )

    target_distance = LaunchConfiguration('target_distance_cm')
    tolerance = LaunchConfiguration('tolerance_cm')
    base_speed = LaunchConfiguration('base_speed')
    steering_offset = LaunchConfiguration('steering_offset')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_distance_cm',
            default_value='60.0',
            description='Desired rear distance in cm.'
        ),
        DeclareLaunchArgument(
            'tolerance_cm',
            default_value='5.0',
            description='Allowed error band around the target distance.'
        ),
        DeclareLaunchArgument(
            'base_speed',
            default_value='6.0',
            description='Base forward speed when within tolerance.'
        ),
        DeclareLaunchArgument(
            'steering_offset',
            default_value='-7.0',
            description='Steering offset (degrees) added to PD control output.'
        ),
        ultrasonic_launch,
        Node(
            package='my_motor',
            executable='ultra_republisher',
            name='ultrasonic_republisher',
            parameters=[
                {'input_topic': 'xycar_ultrasonic'},
                {'min_distance_cm': 0.0},
                {'max_distance_cm': 140.0},
            ],
        ),
        Node(
            package='my_motor',
            executable='pd_controller',
            name='pd_controller',
            parameters=[
                {'target_distance_cm': target_distance},
                {'tolerance_cm': tolerance},
                {'base_speed': base_speed},
                {'steering_offset': steering_offset},
            ],
        ),
    ])

