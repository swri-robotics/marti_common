import os
from setuptools import find_packages, setup

package_name = 'swri_cli_tools'

setup(
    name=package_name,
    version='3.8.3',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['ros2cli', 'setuptools'],
    zip_safe=True,
    author='David Anthony',
    author_email='david.anthony@swri.org',
    maintainer='Southwest Research Institute',
    maintainer_email='swri-robotics@swri.org',
    url='https://github.com/swri-robotics/marti_common',
    keywords=['ROS'],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD 3 Clause',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='SwRI CLI tools provide additional command line tools for introspecting ROS systems.',
    long_description="""\
swri_cli_tools provides command line tools for introspecting and documenting ROS systems""",
    license='BSD 3 Clause',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'swri = swri_cli_tools.command.swri:SwriCommand',
        ],
        'ros2cli.extension_point': [
            'swri_cli_tools.verb = swri_cli_tools.verb:VerbExtension',
        ],
        'swri_cli_tools.verb': [
            'document = swri_cli_tools.verb.document:DocumentVerb',
        ],
    },
)
