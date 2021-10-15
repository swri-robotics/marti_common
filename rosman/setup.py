from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['rosman'],
        package_dir={'': 'src'},
        scripts=['scripts/rosman'],
        requires=['roslib', 'rospkg']
)

setup(**d)
