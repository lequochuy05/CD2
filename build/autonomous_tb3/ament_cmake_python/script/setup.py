from setuptools import find_packages
from setuptools import setup

setup(
    name='script',
    version='0.0.0',
    packages=find_packages(
        include=('script', 'script.*')),
)
