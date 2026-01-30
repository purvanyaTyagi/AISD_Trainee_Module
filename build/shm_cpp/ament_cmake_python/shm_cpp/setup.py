from setuptools import find_packages
from setuptools import setup

setup(
    name='shm_cpp',
    version='0.0.0',
    packages=find_packages(
        include=('shm_cpp', 'shm_cpp.*')),
)
