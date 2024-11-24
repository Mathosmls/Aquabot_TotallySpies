from setuptools import setup
from setuptools import find_packages
import os
package_name = 'py_control'
compiled_module_path = os.path.join('src', 'py_control', 'mppi_pythran_normal.cpython-310-x86_64-linux-gnu.so')
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # recherche automatique des sous-packages
    package_data={
        'py_control': ['mppi_pythran_normal.cpython-310-x86_64-linux-gnu.so'],  # Inclure le fichier compilé
    },
    install_requires=[
        'setuptools',
        'numpy',
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
