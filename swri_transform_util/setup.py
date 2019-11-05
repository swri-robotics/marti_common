from setuptools import setup

package_name = 'swri_transform_util'

setup(
    name=package_name,
    version='2.10.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='P. J. Reed',
    author_email='preed@swri.org',
    keywords=['ROS'],
    requires=['diagnostic_msgs', 'geometry_msgs', 'gps_msgs', 'rclpy', 'sensor_msgs']
)
