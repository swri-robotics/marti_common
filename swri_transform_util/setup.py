## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['swri_transform_util'],
    package_dir={'': 'src'},
    requires=['diagnostic_msgs', 'geometry_msgs', 'gps_common', 'rospy', 'sensor_msgs', 'tf']
)

setup(**setup_args)
