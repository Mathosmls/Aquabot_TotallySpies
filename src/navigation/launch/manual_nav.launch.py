import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter



def generate_launch_description():


    navigation_node = Node(
    package='navigation',
    executable='navigation',
    parameters=[
        {'use_sim_time': True},
    ]
    )


    teleop_node = Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    parameters=[
        {'use_sim_time': True},
    ],
    output='screen',
    prefix='xterm -e',
    )

    return LaunchDescription([
         LaunchDescription([
            SetParameter(name='use_sim_time', value=True),
            # 'use_sim_time' will be set on all nodes following the line above
        ]),

            navigation_node,
            teleop_node

        ])