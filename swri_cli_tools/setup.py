from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['swri_cli_tools'],
        package_dir={'': 'src'},
        scripts=['scripts/swri_cli_tools'],
        requires=['roslib', 'rospkg']
)

setup(**d)
