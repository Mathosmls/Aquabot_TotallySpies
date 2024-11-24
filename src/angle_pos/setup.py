from setuptools import find_packages, setup

package_name = 'angle_pos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghita',
    maintainer_email='ghita.tazeroualt@ensta-bretagne.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qrcode_angle_estimator = qrcode_angle_estimator.qrcode_angle_estimator:main',
        ],
    },
)

