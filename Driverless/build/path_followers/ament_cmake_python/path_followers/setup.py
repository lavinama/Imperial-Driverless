import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='path_followers',
    version='3.1.0',
    packages=find_packages(
        include=('path_followers', 'path_followers.*')),
)
