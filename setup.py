# -*- coding: utf-8 -*-

# Learn more: https://github.com/kennethreitz/setup.py

from setuptools import setup, find_packages


with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='void_py',
    version='0.0.1',
    description='A package for calculating positons of voids',
    long_description=readme,
    author='Akira Shioyoke',
    author_email='s.akira2986@gmail.com',
    url='https://github.com/syoukera/void_py',
    license=license,
    packages=find_packages(exclude=('numpy'))
)

