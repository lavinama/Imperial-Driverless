import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='imperial_driverless_utils',
    version='0.13.1',
    packages=find_packages(
        include=('imperial_driverless_utils', 'imperial_driverless_utils.*')),
)
