#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['kermit', 'kermit.controller'],
    scripts=['nodes/stanley_control'],
    package_dir={'': 'src/python'}
)

setup(**d)
