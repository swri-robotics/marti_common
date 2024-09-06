#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Southwest Research Institute (SwRI)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Southwest Research Institute (SwRI) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import math
import sys
import time
import unittest

from geographic_msgs.msg import GeoPose, GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from gps_msgs.msg import GPSFix, GPSStatus
from sensor_msgs.msg import NavSatFix, NavSatStatus
import rclpy
from rclpy.clock import ClockType
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from ros2topic.api import get_msg_class

PKG = 'swri_transform_util'
NAME = 'test_initialize_origin'

ORIGIN_TOPIC = '/local_xy_origin'
ORIGIN_TYPES = [PoseStamped, GPSFix, GeoPose, GeoPoseStamped]

msg_stamp = rclpy.time.Time(seconds=1337, clock_type=ClockType.ROS_TIME).to_msg()
swri = {
    'latitude': 29.45196669,
    'longitude': -98.61370577,
    'altitude': 233.719,
    'heading': 90.0
}

class TestInitializeOrigin(unittest.TestCase):
    def __init__(self):
        super().__init__()
        self.got_origin = False

    def _yaw_from_quaternion(self, w, x, y, z):
        sc = 2 * ((w * z) + (x * y))
        cc = 1 - 2 * ((y * y) + (z * z))
        yaw = math.atan2(sc, cc)
        return yaw

    def subscribeToOrigin(self):
        self.assertIsNotNone(self.node)
        self.origin_class = get_msg_class(self.node, ORIGIN_TOPIC, blocking=True)
        if self.origin_class is None:
            self.node.get_logger().fatal(ORIGIN_TOPIC+" was never advertised")
        self.assertIsNotNone(self.origin_class)
        self.node.get_logger().info("Origin is a " + str(self.origin_class) + " message")
        self.assertIn(self.origin_class, ORIGIN_TYPES)
        self.test_stamp = False  # Enable this for auto origin
        self.got_origin = False
        return self.node.create_subscription(
            self.origin_class, ORIGIN_TOPIC,
            self.originCallback,
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
        )

    def originCallback(self, msg):
        self.assertIsNotNone(self.node)
        self.node.get_logger().info("Callback received a message.")
        self.got_origin = True
        if self.origin_class == PoseStamped:
            latitude = msg.pose.position.y
            longitude = msg.pose.position.x
            altitude = msg.pose.position.z
            yaw = self._yaw_from_quaternion(
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z)
            self.assertAlmostEqual(yaw, 0)
        elif self.origin_class == GPSFix:
            self.node.get_logger().info("Status: %d" % msg.status.status)
            self.assertEqual(msg.status.status, GPSStatus.STATUS_FIX)
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
            self.assertAlmostEqual(msg.track, swri['heading'])
        elif self.origin_class == NavSatFix:
            self.node.get_logger().info("Status: %d" % msg.status.status)
            self.assertEqual(msg.status.status, NavSatStatus.STATUS_FIX)
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
        else:  # self.origin_class == GeoPose:
            latitude = msg.position.latitude
            longitude = msg.position.longitude
            altitude = msg.position.altitude
            yaw = self._yaw_from_quaternion(
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z)
            self.assertAlmostEqual(yaw, 0)
        self.assertEqual(msg.header.frame_id, '/far_field')
        if self.test_stamp:
            self.assertEqual(
                rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds,
                rclpy.time.Time.from_msg(msg_stamp).nanoseconds
            )
        else:
            self.assertEqual(
                rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds,
                0
            )
        self.assertAlmostEqual(longitude, swri['longitude'])
        self.assertAlmostEqual(latitude, swri['latitude'])
        self.assertAlmostEqual(altitude, swri['altitude'])
        self.node.destroy_node()


class TestInvalidOrigin(unittest.TestCase):
    def __init__(self):
        super().__init__()
        self.got_message = False

    def subscribeToOrigin(self):
        self.assertIsNotNone(self.node)
        self.origin_class = get_msg_class(self.node, ORIGIN_TOPIC, blocking=True)
        if self.origin_class is None:
            self.node.get_logger().fatal(ORIGIN_TOPIC+" was never advertised")
        self.assertIsNotNone(self.origin_class)
        self.node.get_logger().info("Origin is a " + str(self.origin_class) + " message")
        self.assertIn(self.origin_class, ORIGIN_TYPES)
        self.test_stamp = False  # Enable this for auto origin
        self.got_origin = False
        return self.node.create_subscription(
            self.origin_class, ORIGIN_TOPIC,
            self.originCallback,
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
        )

    def originCallback(self, msg):
        self.assertIsNotNone(self.node)
        self.got_message = True
        self.node.destroy_node()


class TestAutoOriginFromGPSFix(TestInitializeOrigin):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('test_auto_origin_from_gps_fix')
        gps_pub = self.node.create_publisher(
            GPSFix, 'gps',
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=2
            )
        )
        origin_sub = self.subscribeToOrigin()
        self.test_stamp = True
        gps_msg = GPSFix()
        gps_msg.status.status = GPSStatus.STATUS_FIX
        gps_msg.latitude = swri['latitude']
        gps_msg.longitude = swri['longitude']
        gps_msg.altitude = swri['altitude']
        gps_msg.track = swri['heading']
        gps_msg.header.stamp = msg_stamp
        gps_pub.publish(gps_msg)
        while not self.got_origin:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)


class TestInvalidGPSFix(TestInvalidOrigin):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('test_invalid_gps_fix')
        gps_pub = self.node.create_publisher(GPSFix, 'gps', 2)
        origin_sub = self.subscribeToOrigin()
        self.test_stamp = True
        gps_msg = GPSFix()
        gps_msg.status.status = GPSStatus.STATUS_NO_FIX
        gps_msg.header.stamp = msg_stamp

        # There are two ways in which initialize_origin.py could fail due to getting
        # an invalid fix:  if it publishes an origin despite not getting a valid fix
        # or if it unsubscribes from the gps & fix topics without ever publishing
        # an origin.
        # This will test for those conditions by waiting until the node has
        # subscribed to the topic, then failing if either ROS shuts down, which
        # our subscriber will do if it gets an origin message, or if the number of
        # connections drops to zero, which means initialize_origin.py subscribed
        # but did not publish a message.
        timeout = time.time() + 2  # time out after 2 seconds, which should be plenty
        node_attached = False
        count = 0
        while not self.got_message and time.time() < timeout:
            if not node_attached and gps_pub.get_subscription_count() > 0:
                node_attached = True
            if node_attached and gps_pub.get_subscription_count() == 0:
                break
            count = gps_pub.get_subscription_count()
            gps_pub.publish(gps_msg)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)

        self.assertFalse(self.got_message,
                         "initialize_origin should not have published an origin.")
        self.assertFalse(node_attached and count == 0,
                         "initialize_origin unsubscribed without getting a valid fix.")

        self.node.destroy_node()


class TestAutoOriginFromNavSatFix(TestInitializeOrigin):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('test_auto_origin_from_nav_sat_fix')
        nsf_pub = self.node.create_publisher(
            NavSatFix, 'fix',
            QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=2
            )
        )
        origin_sub = self.subscribeToOrigin()
        self.test_stamp = True
        nsf_msg = NavSatFix()
        nsf_msg.status.status = NavSatStatus.STATUS_FIX
        nsf_msg.latitude = swri['latitude']
        nsf_msg.longitude = swri['longitude']
        nsf_msg.altitude = swri['altitude']
        nsf_msg.header.frame_id = "/far_field"
        nsf_msg.header.stamp = msg_stamp
        nsf_pub.publish(nsf_msg)
        while not self.got_origin:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)


class TestInvalidNavSatFix(TestInvalidOrigin):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('test_invalid_nav_sat_fix')
        nsf_pub = self.node.create_publisher(NavSatFix, 'fix', 2)
        origin_sub = self.subscribeToOrigin()
        self.test_stamp = True
        nsf_msg = NavSatFix()
        nsf_msg.status.status = NavSatStatus.STATUS_NO_FIX
        nsf_msg.header.frame_id = "/far_field"
        nsf_msg.header.stamp = msg_stamp

        # See documentation in testInvalidGPSFix.
        timeout = time.time() + 2  # time out after 2 seconds, which should be plenty
        node_attached = False
        count = 0
        while not self.got_message and time.time() < timeout:
            if not node_attached and nsf_pub.get_subscription_count() > 0:
                node_attached = True
            if node_attached and nsf_pub.get_subscription_count() == 0:
                break
            count = nsf_pub.get_subscription_count()
            nsf_pub.publish(nsf_msg)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)

        self.assertFalse(self.got_message,
                         "initialize_origin should not have published an origin.")
        self.assertFalse(node_attached and count == 0,
                         "initialize_origin unsubscribed without getting a valid fix.")

        self.node.destroy_node()


class TestManualOrigin(TestInitializeOrigin):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('test_manual_origin')
        origin_sub = self.subscribeToOrigin()
        while not self.got_origin:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)


if __name__ == "__main__":
    time.sleep(5)
    rclpy.init()

    if sys.argv[1] == "auto_gps":
        test = TestAutoOriginFromGPSFix()
    elif sys.argv[1] == "auto_navsat":
        test = TestAutoOriginFromNavSatFix()
    elif sys.argv[1] == "invalid_gps":
        test = TestInvalidGPSFix()
    elif sys.argv[1] == "invalid_navsat":
        test = TestInvalidNavSatFix()
    elif sys.argv[1] == "manual":
        test = TestManualOrigin()

    rclpy.shutdown()
