import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='full_gazebo_simulator',
    version='0.3.1',
    packages=find_packages(
        include=('full_gazebo_simulator', 'full_gazebo_simulator.*')),
)
