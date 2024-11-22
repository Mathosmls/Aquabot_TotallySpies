from setuptools import setup

package_name = 'map_modifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Votre Nom',
    maintainer_email='votre.email@example.com',
    description='Package ROS 2 pour estimer la vitesse et direction du vent.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_modifier = map_modifier.map_modifier:main',
        ],
    },
)
