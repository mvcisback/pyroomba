#!/usr/bin/env python3
from setuptools import setup, find_packages

setup(
    name='PyRoomba',
    version='0.1.0',
    author='Marcell Vazquez-Chanlatte, Damon Kohler',
    author_email='damonkohler@gmail.com',
    packages=find_packages(),
    url='',
    license='LICENSE.txt',
    description='',
    long_description=open('README').read(),
    requires = ['amqp', 'pyserial'],
)
