# -*- coding: utf-8 -*-

# Learn more: https://github.com/kennethreitz/setup.py

from setuptools import setup, find_packages


with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='boid_py',
    version='0.0.1',
    description='A package for calculating positons of boids',
    long_description=readme,
    author='Akira Shioyoke',
    author_email='s.akira2986@gmail.com',
    install_requires=['numpy'],
    url='https://github.com/syoukera/boid_py',
    license=license,
    packages=find_packages()
)

