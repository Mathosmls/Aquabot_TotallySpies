from setuptools import setup
from setuptools import find_packages

package_name = 'control_cam_qr'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'opencv-python',
        'numpy',
        'tf-transformations',
        'cv_bridge',
    ],
    zip_safe=True,
    maintainer='Votre Nom',
    maintainer_email='votre_email@example.com',
    description='Nœud ROS 2 combinant le contrôle de caméra et le décodage des QR codes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_cam_qr = control_cam_qr.control_cam_qr_node:main',
        ],
    },
)

