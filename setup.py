#!/usr/bin/env python3
from setuptools import setup, find_packages

setup(
    name='PyRoomba',
    version='0.1.0',
    author='Marcell Vazquez-Chanlatte',
    packages=find_packages(),
    url='',
    license='LICENSE',
    description='',
    long_description=open('README.md').read(),
    requires = ['amqp', 'pyserial'],
)
