from setuptools import find_packages
from setuptools import setup

setup(
    name='opencv_example',
    version='0.3.0',
    packages=find_packages(
        include=('opencv_example', 'opencv_example.*')),
)
