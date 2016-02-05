## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(
    version='1.0.0',
    scripts=['bin/pixhawk_control.py',
  'bin/INAV_OPTIONS.py',
  'bin/position_algorithm.py'],
    packages=['laserpack'],
    package_dir={'': 'src'}
)