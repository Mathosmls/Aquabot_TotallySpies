from setuptools import setup
import os
from glob import glob

package_name = 'py_qr'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VotreNom',
    maintainer_email='votre.email@example.com',
    description='A minimal image subscriber using ROS 2 and OpenCV',
    license='License Information',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'opencv_decoder_node = py_qr.opencv_decoder_node:main',
        ],
    },
)

