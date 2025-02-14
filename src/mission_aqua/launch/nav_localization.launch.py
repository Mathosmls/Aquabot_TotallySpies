from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Récupération des chemins vers les fichiers de lancement
    package_sensors_processing_launch_file = os.path.join(get_package_share_directory('sensors_processing'), 'launch', 'sensors_processing.launch.py')
    package_rl_launch_file = os.path.join(get_package_share_directory('robot_localization'), 'launch', 'aquabot_ekfs.launch.py')
    package_nav2_launch_file = os.path.join(get_package_share_directory('nav2aqua'), 'launch', 'nav2aqua.launch.py')
    
    # Création des inclusions de fichiers de lancement
    package_sensors_processing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package_sensors_processing_launch_file)
    )
    package_rl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package_rl_launch_file)
    )
    package_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package_nav2_launch_file)
    )
    
    
    # Ajouter tous les fichiers de lancement dans la description
    return LaunchDescription([

        LogInfo(msg="Inclusion du fichier sensors_processing"),
        package_sensors_processing_launch,
        LogInfo(msg="Inclusion du fichier robot_localization"),
        package_rl_launch,
        LogInfo(msg="Inclusion du fichier nav2aqua"),
        package_nav2_launch,

    ])

