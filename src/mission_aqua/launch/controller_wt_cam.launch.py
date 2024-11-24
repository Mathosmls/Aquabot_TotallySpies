from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    node_cam = Node(
        package='control_cam_qr',
        executable='control_cam_qr',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    node_control = Node(
        package='py_control',
        executable='controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    node_wt_pos = Node(
        package='navigation',
        executable='graph_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    node_map_modifier = Node(
        package='map_modifier',
        executable='map_modifier',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Ajouter tous les fichiers de lancement dans la description
    return LaunchDescription([
        LogInfo(msg="Lancement de node_map_modifier"),
        node_map_modifier,
        LogInfo(msg="Lancement de node_wt_pos"),
        node_wt_pos,
        LogInfo(msg="Lancement de node_control"),
        node_control,
        LogInfo(msg="Lancement de node_cam"),
        node_cam,

    ])
