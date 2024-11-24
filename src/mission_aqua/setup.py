from setuptools import find_packages, setup

package_name = 'mission_aqua'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav_localization.launch.py']),  # Inclure le fichier de lancement
         ('share/' + package_name + '/launch', ['launch/controller_wt_cam.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mls',
    maintainer_email='mathis.le_scoezec@ensta-bretagne.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_aqua_node = mission_aqua.mission_aqua:main',  # Associer mission_aqua.py à un exécutable ROS 2
        ],
    },
)
