from setuptools import find_packages
from setuptools import setup

setup(
    name='aquabot_gz',
    version='0.6.0',
    packages=find_packages(
        include=('aquabot_gz', 'aquabot_gz.*')),
)
