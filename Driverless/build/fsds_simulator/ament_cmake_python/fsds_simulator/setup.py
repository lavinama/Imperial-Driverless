import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='fsds_simulator',
    version='0.0.0',
    packages=find_packages(
        include=('fsds_simulator', 'fsds_simulator.*')),
)
