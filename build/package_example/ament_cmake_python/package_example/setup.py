from setuptools import find_packages
from setuptools import setup

setup(
    name='package_example',
    version='0.3.0',
    packages=find_packages(
        include=('package_example', 'package_example.*')),
)
