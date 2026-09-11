#!/usr/bin/env python3
# *****************************************************************************
#
# Copyright (c) 2026, Southwest Research Institute® (SwRI®)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# *****************************************************************************

import os
import time
import unittest

import pytest

import ament_index_python
import launch
import launch_testing
import rclpy
from gps_msgs.msg import GPSFix
from launch_ros.actions import Node

WARNING = '/local_xy_origin is published as sensor_msgs/msg/NavSatFix'


@pytest.mark.launch_test
def generate_test_description():
    # lat_lon_tf_echo is installed to bin/ rather than lib/<package>, so it is
    # launched by path instead of as a launch_ros Node.
    echo = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                ament_index_python.get_package_prefix('swri_transform_util'),
                'bin', 'lat_lon_tf_echo'),
            'far_field',
            'base_link',
        ],
        name='lat_lon_tf_echo',
        # The node reports with printf, which a pipe would buffer indefinitely.
        emulate_tty=True,
        output='screen',
    )

    target_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '100',
            '--frame-id', 'far_field',
            '--child-frame-id', 'base_link',
        ],
    )

    # Published from its own process, since DDS allows only one type per topic
    # within a process and the test itself publishes a GPSFix later.
    unsupported_origin = launch.actions.ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-r', '2',
             '/local_xy_origin', 'sensor_msgs/msg/NavSatFix', '{}'],
        name='unsupported_origin',
        output='screen',
    )

    return launch.LaunchDescription([
        echo,
        target_tf,
        unsupported_origin,
        launch_testing.actions.ReadyToTest(),
    ]), {'echo': echo}


class LatLonTfEchoUnsupportedOriginTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('lat_lon_tf_echo_unsupported_origin_test')

    def tearDown(self):
        self.node.destroy_node()

    def test_warns_once_then_accepts_supported_origin(self, proc_output, echo):
        self.assertTrue(
            proc_output.waitFor(WARNING, process=echo, timeout=30, stream='stderr'),
            'lat_lon_tf_echo never warned about the unsupported origin type')

        # Discovery polls every 250 ms, so this gives it several chances to
        # repeat the warning.
        time.sleep(2.0)
        stderr = ''.join(output.text.decode()
                         for output in proc_output[echo] if output.from_stderr)
        self.assertEqual(1, stderr.count(WARNING), stderr)

        # Polling continues after the warning, so a supported origin that shows
        # up later is still used.
        publisher = self.node.create_publisher(GPSFix, '/local_xy_origin', 1)
        fix = GPSFix()
        fix.latitude = 29.45
        fix.longitude = -98.61

        deadline = time.monotonic() + 30.0
        reported = False
        while not reported and time.monotonic() < deadline:
            publisher.publish(fix)
            reported = proc_output.waitFor(
                'Latitude:', process=echo, timeout=0.5, stream='stdout')
        self.assertTrue(reported, 'lat_lon_tf_echo never reported a position')
