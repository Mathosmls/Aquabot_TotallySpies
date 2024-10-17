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

    
    tf_sim_GPS= Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "1.3", "0", "0", "0", "aquabot/wamv/base_link", "aquabot/wamv/gps_wamv_link"])

    
    tf_sim_IMU = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["1", "0", "1.62", "0", "0", "0", "aquabot/wamv/base_link", "aquabot/wamv/imu_wamv_link"])
   

    sensors_processing_node = Node(
    package='sensors_processing',
    executable='sensors_processing',
    parameters=[
        {'use_sim_time': True},
    ]
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[
            {'use_sim_time': True},
        ],
        arguments=['-d', 'rviz_config.rviz'])


    return LaunchDescription([
         LaunchDescription([
            SetParameter(name='use_sim_time', value=True),
            # 'use_sim_time' will be set on all nodes following the line above
        ]),
            tf_sim_GPS,
            tf_sim_IMU,
            sensors_processing_node,
            rviz
        ])