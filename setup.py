#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['dialogue_state_machine', 'dialogue_state_machine_ros'],
 package_dir={'dialogue_state_machine': 'common/src/dialogue_state_machine', 'dialogue_state_machine_ros': 'ros/src/dialogue_state_machine_ros'}
)

setup(**d)
