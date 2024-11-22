
from setuptools import setup

package_name = 'control_cam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='Votre Nom',
    maintainer_email='votre.email@example.com',
    description='Package ROS 2 pour la gestion de la caméra et de son orientation avec OpenCV et ROS2.',
    license='Licence du package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_cam_node = control_cam.control_cam_node:main',
        ],
    },
)

