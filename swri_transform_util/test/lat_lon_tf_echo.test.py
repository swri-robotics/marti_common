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

import math
import os
import re
import time
import unittest

import pytest

import ament_index_python
import launch
import launch_testing
import rclpy
from geometry_msgs.msg import PoseStamped
from gps_msgs.msg import GPSFix
from launch_ros.actions import Node

ORIGIN_LAT = 29.45
ORIGIN_LON = -98.61
# The compass bearing of the local X axis, clockwise from north. 30 degrees
# keeps the correct conversion from a GPSFix track (90 - track), passing track
# through unconverted, and 90 + track all distinguishable, which 0 and 45 do
# not.
TRACK = 30.0
# How far along the local X axis the target frame sits.
DISTANCE = 100.0
# The target frame's yaw in the local frame, counter-clockwise, in degrees.
# It is negative so that dropping its sign is caught, and nonzero so that
# ignoring the reference angle is too: those mistakes print 10 and 110 degrees
# where the correct heading is 50.
TARGET_YAW = -20.0

OUTPUT_RE = re.compile(
    r'Latitude: (-?\d+\.\d+)°, Longitude: (-?\d+\.\d+)°, Heading: (-?\d+\.\d+)°')


def gps_fix_origin():
    fix = GPSFix()
    fix.latitude = ORIGIN_LAT
    fix.longitude = ORIGIN_LON
    fix.altitude = 0.0
    fix.track = TRACK
    return fix


def pose_stamped_origin():
    pose = PoseStamped()
    pose.pose.position.y = ORIGIN_LAT
    pose.pose.position.x = ORIGIN_LON
    pose.pose.position.z = 0.0
    # The yaw is ENU, counter-clockwise from east.
    yaw = math.radians(90.0 - TRACK)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


# Both origins describe the same local frame, so the node's report must be the
# same for each.
@pytest.mark.launch_test
@launch_testing.parametrize('origin', [gps_fix_origin(), pose_stamped_origin()])
def generate_test_description(origin):
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
            '--x', str(DISTANCE),
            '--yaw', str(math.radians(TARGET_YAW)),
            '--frame-id', 'far_field',
            '--child-frame-id', 'base_link',
        ],
    )

    return launch.LaunchDescription([
        echo,
        target_tf,
        launch_testing.actions.ReadyToTest(),
    ]), {'echo': echo, 'origin': origin}


class LatLonTfEchoOriginTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('lat_lon_tf_echo_test')

    def tearDown(self):
        self.node.destroy_node()

    def reported_position(self, proc_output, echo, origin):
        """Publish the origin until the node reports, and return the report."""
        publisher = self.node.create_publisher(type(origin), '/local_xy_origin', 1)

        # The node only subscribes once it has discovered a publisher, so keep
        # publishing until it reports a position.
        deadline = time.monotonic() + 30.0
        reported = False
        while not reported and time.monotonic() < deadline:
            publisher.publish(origin)
            # waitFor() searches stderr unless told otherwise, and the node
            # reports on stdout.
            reported = proc_output.waitFor(
                OUTPUT_RE, process=echo, timeout=0.5, stream='stdout')
        self.assertTrue(reported, 'lat_lon_tf_echo never reported a position')

        text = ''.join(output.text.decode()
                       for output in proc_output[echo] if output.from_stdout)
        match = OUTPUT_RE.search(text)
        self.assertIsNotNone(match, text)
        return tuple(float(group) for group in match.groups())

    def test_position_follows_origin_heading(self, proc_output, echo, origin):
        lat, lon, _ = self.reported_position(proc_output, echo, origin)

        # Metres per degree at the origin's latitude, which is ample precision
        # for the 100 m offset and six decimal places the node prints.
        phi = math.radians(ORIGIN_LAT)
        metres_per_deg_lat = (111132.92 - 559.82 * math.cos(2 * phi)
                              + 1.175 * math.cos(4 * phi))
        metres_per_deg_lon = (111412.84 * math.cos(phi)
                              - 93.5 * math.cos(3 * phi))
        north = (lat - ORIGIN_LAT) * metres_per_deg_lat
        east = (lon - ORIGIN_LON) * metres_per_deg_lon

        # The local X axis points along a compass bearing of TRACK, so the
        # target lies DISTANCE metres away in that direction.
        self.assertAlmostEqual(DISTANCE, math.hypot(north, east), delta=0.5)
        self.assertAlmostEqual(
            TRACK, math.degrees(math.atan2(east, north)), delta=0.5)

    def test_heading_is_compass_bearing(self, proc_output, echo, origin):
        _, _, heading = self.reported_position(proc_output, echo, origin)

        # The local X axis points along a compass bearing of TRACK, and a
        # counter-clockwise yaw from it turns the heading anticlockwise on the
        # compass.
        self.assertAlmostEqual((TRACK - TARGET_YAW) % 360.0, heading, delta=0.01)
