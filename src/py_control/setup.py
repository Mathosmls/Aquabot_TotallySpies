from setuptools import setup
from setuptools import find_packages

package_name = 'py_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # recherche automatique des sous-packages
    install_requires=[
        'setuptools',
        'numpy',
        'numba',
        'matplotlib',
        'rclpy',  # dépendance ROS 2 Python
    ],
    zip_safe=True,
    author='Votre Nom',
    author_email='votre.email@example.com',
    description='Un contrôleur MPPI pour Aquabot implémenté en Python avec ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_node = py_control.py_control:main'
        ],
    },
)
