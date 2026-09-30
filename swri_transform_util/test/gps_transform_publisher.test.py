#!/usr/bin/env python3
# *****************************************************************************
#
# Copyright (c) 2026, Southwest Research Institute® (SwRI®)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# *****************************************************************************

"""gps_transform_publisher publishes a TF only for GPSFix messages with a position."""

import math
import time
import unittest

from geometry_msgs.msg import PoseStamped
from gps_msgs.msg import GPSFix
from gps_msgs.msg import GPSStatus
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage

ORIGIN_LAT = 29.45
ORIGIN_LON = -98.61


@pytest.mark.launch_test
def generate_test_description():
    container = ComposableNodeContainer(
        name='gps_transform_publisher_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='swri_transform_util',
                plugin='swri_transform_util::GpsTransformPublisher',
                name='gps_transform_publisher',
                parameters=[{'parent_frame_id': 'map', 'child_frame_id': 'base_link'}]),
        ],
        output='screen',
    )
    # TransformManager only resolves wgs84 to a frame that is already in /tf,
    # as map is in a real system, so put it there.
    map_in_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'gps_test_anchor'],
    )
    return launch.LaunchDescription([
        container,
        map_in_tf,
        launch_testing.actions.ReadyToTest(),
    ]), {'container': container}


def gps_fix(latitude, longitude, status=GPSStatus.STATUS_FIX):
    fix = GPSFix()
    fix.status.status = status
    fix.latitude = latitude
    fix.longitude = longitude
    fix.altitude = 0.0
    fix.track = 0.0
    return fix


class GpsTransformPublisherTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_gps_transform_publisher')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_only_fixes_with_a_position_move_the_frame(self, proc_output, container):
        # The origin makes map a local XY frame at ORIGIN_LAT, ORIGIN_LON.
        origin = PoseStamped()
        origin.header.frame_id = 'map'
        origin.pose.position.y = ORIGIN_LAT
        origin.pose.position.x = ORIGIN_LON
        origin.pose.orientation.w = 1.0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        origin_pub = self.node.create_publisher(PoseStamped, '/local_xy_origin', latched)
        origin_pub.publish(origin)

        transforms = []

        def on_tf(msg):
            transforms.extend(t for t in msg.transforms if t.child_frame_id == 'base_link')

        self.node.create_subscription(TFMessage, '/tf', on_tf, 100)
        gps_pub = self.node.create_publisher(GPSFix, 'gps', 10)

        # Publish a valid fix at the origin until its transform comes back.
        deadline = time.monotonic() + 30.0
        while not transforms and time.monotonic() < deadline:
            gps_pub.publish(gps_fix(ORIGIN_LAT, ORIGIN_LON))
            self.spin_for(0.5)
        self.assertTrue(transforms, 'no transform for a valid GPSFix')

        # Then fixes with no position: no fix at 0N 0E, and a NaN position
        # that claims a fix. Neither may move base_link.
        transforms.clear()
        for _ in range(5):
            gps_pub.publish(gps_fix(0.0, 0.0, GPSStatus.STATUS_NO_FIX))
            gps_pub.publish(gps_fix(math.nan, ORIGIN_LON))
            gps_pub.publish(gps_fix(ORIGIN_LAT, math.nan))
            self.spin_for(0.2)
        self.spin_for(1.0)
        for t in transforms:
            translation = t.transform.translation
            self.assertTrue(
                math.isfinite(translation.x) and math.isfinite(translation.y),
                f'base_link moved to a non-finite position: {translation}')
            self.assertLess(
                math.hypot(translation.x, translation.y), 1.0,
                f'base_link moved away from the origin: {translation}')

        # tf2 rejects NaN transforms with an error; none should have reached it.
        text = ''.join(output.text.decode() for output in proc_output[container])
        self.assertNotIn('TF_NAN_INPUT', text)
