from setuptools import find_packages
from setuptools import setup

setup(
    name='my_robot_description',
    version='0.0.0',
    packages=find_packages(
        include=('my_robot_description', 'my_robot_description.*')),
)
