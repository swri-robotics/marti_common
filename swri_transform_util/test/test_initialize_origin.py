#!/usr/bin/env python
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
#
import sys
import time
import unittest

from geographic_msgs.msg import GeoPose, GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from gps_common.msg import GPSFix, GPSStatus
from sensor_msgs.msg import NavSatFix, NavSatStatus
import rospy
import rostest
import rostopic
import tf.transformations

PKG = 'swri_transform_util'
NAME = 'test_initialize_origin'

ORIGIN_TOPIC = '/local_xy_origin'
ORIGIN_TYPES = [PoseStamped, GPSFix, GeoPose, GeoPoseStamped]

msg_stamp = rospy.Time(1337, 0xDEADBEEF)
swri = {
    'latitude': 29.45196669,
    'longitude': -98.61370577,
    'altitude': 233.719,
    'heading': 90
}


class TestInitializeOrigin(unittest.TestCase):
    def subscribeToOrigin(self):
        # This line blocks until initialize_origin is alive and has advertised the origin topic
        origin_class = rostopic.get_topic_class(ORIGIN_TOPIC, blocking=True)[0]
        rospy.loginfo("Origin is a " + origin_class._type + " message")
        self.assertIsNotNone(origin_class, msg=ORIGIN_TOPIC+" was never advertised")
        self.assertIn(origin_class, ORIGIN_TYPES)
        self.test_stamp = False  # Enable this for auto origin
        self.got_origin = False
        return rospy.Subscriber(ORIGIN_TOPIC, origin_class, self.originCallback)

    def originCallback(self, msg):
        rospy.loginfo("Callback received a " + msg._type + " message.")
        self.got_origin = True
        if msg._type == PoseStamped._type:
            latitude = msg.pose.position.y
            longitude = msg.pose.position.x
            altitude = msg.pose.position.z
            quaternion = (msg.pose.orientation.x,
                          msg.pose.orientation.y,
                          msg.pose.orientation.z,
                          msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            self.assertAlmostEqual(yaw, 0)
        elif msg._type == GPSFix._type:
            rospy.loginfo("Status: %d" % msg.status.status)
            self.assertEqual(msg.status.status, GPSStatus.STATUS_FIX)
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
            self.assertAlmostEqual(msg.track, swri['heading'])
        elif msg._type == NavSatFix._type:
            rospy.loginfo("Status: %d" % msg.status.status)
            self.assertEqual(msg.status.status, NavSatStatus.STATUS_FIX)
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
        else:  # msg._type == GeoPose._type:
            latitude = msg.position.latitude
            longitude = msg.position.longitude
            altitude = msg.position.altitude
            quaternion = (msg.pose.orientation.x,
                          msg.pose.orientation.y,
                          msg.pose.orientation.z,
                          msg.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            self.assertAlmostEqual(yaw, 0)
        self.assertEqual(msg.header.frame_id, '/far_field')
        if self.test_stamp:
            self.assertEqual(msg.header.stamp, msg_stamp)
        else:
            self.assertEqual(msg.header.stamp, rospy.Time(0))
        self.assertAlmostEqual(longitude, swri['longitude'])
        self.assertAlmostEqual(latitude, swri['latitude'])
        self.assertAlmostEqual(altitude, swri['altitude'])
        rospy.signal_shutdown("Test complete")


class TestInvalidOrigin(unittest.TestCase):
    def subscribeToOrigin(self):
        # This line blocks until initialize_origin is alive and has advertised the origin topic
        origin_class = rostopic.get_topic_class(ORIGIN_TOPIC, blocking=True)[0]
        rospy.loginfo("Origin is a " + origin_class._type + " message")
        self.assertIsNotNone(origin_class, msg=ORIGIN_TOPIC + " was never advertised")
        self.assertIn(origin_class, ORIGIN_TYPES)
        self.test_stamp = False  # Enable this for auto origin
        self.got_message = False
        return rospy.Subscriber(ORIGIN_TOPIC, origin_class, self.originCallback)

    def originCallback(self, msg):
        rospy.logerr("Callback received a " + msg._type + " message.")
        self.got_message = True
        rospy.signal_shutdown("Test complete")


class TestAutoOriginFromGPSFix(TestInitializeOrigin):
    def testAutoOriginFromGPSFix(self):
        rospy.init_node('test_auto_origin_from_gps_fix')
        gps_pub = rospy.Publisher('gps', GPSFix, queue_size=2, latch=True)
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
        rospy.spin()
        self.assertTrue(self.got_origin)


class TestInvalidGPSFix(TestInvalidOrigin):
    def testInvalidGPSFix(self):
        rospy.init_node('test_invalid_gps_fix')
        gps_pub = rospy.Publisher('gps', GPSFix, queue_size=2)
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
        r = rospy.Rate(100.0)
        timeout = time.time() + 2  # time out after 2 seconds, which should be plenty
        node_attached = False
        while not rospy.is_shutdown() and time.time() < timeout:
            if not node_attached and gps_pub.get_num_connections() > 0:
                node_attached = True
            if node_attached and gps_pub.get_num_connections() == 0:
                break
            gps_pub.publish(gps_msg)
            r.sleep()

        self.assertFalse(self.got_message,
                         "initialize_origin should not have published an origin.")
        self.assertFalse(node_attached and gps_pub.get_num_connections() == 0,
                         "initialize_origin unsubscribed without getting a valid fix.")


class TestAutoOriginFromNavSatFix(TestInitializeOrigin):
    def testAutoOriginFromNavSatFix(self):
        rospy.init_node('test_auto_origin_from_nav_sat_fix')
        nsf_pub = rospy.Publisher('fix', NavSatFix, queue_size=2, latch=True)
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
        rospy.spin()
        self.assertTrue(self.got_origin)


class TestInvalidNavSatFix(TestInvalidOrigin):
    def testInvalidNavSatFix(self):
        rospy.init_node('test_invalid_nav_sat_fix')
        nsf_pub = rospy.Publisher('fix', NavSatFix, queue_size=2)
        origin_sub = self.subscribeToOrigin()
        self.test_stamp = True
        nsf_msg = NavSatFix()
        nsf_msg.status.status = NavSatStatus.STATUS_NO_FIX
        nsf_msg.header.frame_id = "/far_field"
        nsf_msg.header.stamp = msg_stamp

        # See documentation in testInvalidGPSFix.
        r = rospy.Rate(100.0)
        timeout = time.time() + 2  # time out after 2 seconds, which should be plenty
        node_attached = False
        while not rospy.is_shutdown() and time.time() < timeout:
            if not node_attached and nsf_pub.get_num_connections() > 0:
                node_attached = True
            if node_attached and nsf_pub.get_num_connections() == 0:
                break
            nsf_pub.publish(nsf_msg)
            r.sleep()

        self.assertFalse(self.got_message,
                         "initialize_origin should not have published an origin.")
        self.assertFalse(node_attached and nsf_pub.get_num_connections() == 0,
                         "initialize_origin unsubscribed without getting a valid fix.")


class TestAutoOriginFromCustom(TestInitializeOrigin):
    def testAutoOriginFromCustom(self):
        rospy.init_node('test_auto_origin_from_custom')
        custom_pub = rospy.Publisher('pose', GeoPoseStamped, queue_size=2, latch=True)
        origin_sub = self.subscribeToOrigin()
        self.test_stamp = True
        custom_msg = GeoPoseStamped()
        custom_msg.pose.position.latitude = swri['latitude']
        custom_msg.pose.position.longitude = swri['longitude']
        custom_msg.pose.position.altitude = swri['altitude']
        custom_msg.header.frame_id = "/far_field"
        custom_msg.header.stamp = msg_stamp
        custom_pub.publish(custom_msg)
        rospy.spin()
        self.assertTrue(self.got_origin)


class TestManualOrigin(TestInitializeOrigin):
    def testManualOrigin(self):
        rospy.init_node('test_manual_origin')
        origin_sub = self.subscribeToOrigin()
        rospy.spin()
        self.assertTrue(self.got_origin)


if __name__ == "__main__":
    if sys.argv[1] == "auto_custom":
        rostest.rosrun(PKG, NAME, TestAutoOriginFromCustom, sys.argv)
    elif sys.argv[1] == "auto_gps":
        rostest.rosrun(PKG, NAME, TestAutoOriginFromGPSFix, sys.argv)
    elif sys.argv[1] == "auto_navsat":
        rostest.rosrun(PKG, NAME, TestAutoOriginFromNavSatFix, sys.argv)
    elif sys.argv[1] == "invalid_gps":
        rostest.rosrun(PKG, NAME, TestInvalidGPSFix, sys.argv)
    elif sys.argv[1] == "invalid_navsat":
        rostest.rosrun(PKG, NAME, TestInvalidNavSatFix, sys.argv)
    elif sys.argv[1] == "manual":
        rostest.rosrun(PKG, NAME, TestManualOrigin, sys.argv)
