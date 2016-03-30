## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(
    version='1.0.0',
    scripts=['bin/INAV_OPTIONS.py'],
    packages=['laserpack'],
    package_dir={'': 'src'}
)