#! /usr/bin/env python
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

from geometry_msgs.msg import PoseStamped
from gps_common.msg import GPSFix
from geographic_msgs.msg import GeoPose
import rospy
import rostest
import rostopic
from sensor_msgs.msg import NavSatFix
import sys
import tf.transformations
import unittest

PKG = 'swri_transform_util'
NAME = 'test_initialize_origin'

ORIGIN_TOPIC = '/local_xy_origin'
ORIGIN_TYPES = [PoseStamped, GPSFix, GeoPose]

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
        return rospy.Subscriber(ORIGIN_TOPIC, origin_class, self.originCallback)        

    def originCallback(self, msg):
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
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
            self.assertAlmostEqual(msg.track, swri['heading'])
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


class TestAutoOriginFromGPSFix(TestInitializeOrigin):
    def testAutoOriginFromGPSFix(self):
        rospy.init_node('test_initialize_origin')
        gps_pub = rospy.Publisher('gps', GPSFix, queue_size=2)
        origin_sub = self.subscribeToOrigin()
        self.test_stamp = True
        gps_msg = GPSFix()
        gps_msg.latitude = swri['latitude']
        gps_msg.longitude = swri['longitude']
        gps_msg.altitude = swri['altitude']
        gps_msg.track = swri['heading']
        gps_msg.header.stamp = msg_stamp
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            gps_pub.publish(gps_msg)
            r.sleep()


class TestAutoOriginFromNavSatFix(TestInitializeOrigin):
    def testAutoOriginFromNavSatFix(self):
        rospy.init_node('test_initialize_origin')
        nsf_pub = rospy.Publisher('fix', NavSatFix, queue_size=2)
        origin_sub = self.subscribeToOrigin()
        self.test_stamp = True
        nsf_msg = NavSatFix()
        nsf_msg.latitude = swri['latitude']
        nsf_msg.longitude = swri['longitude']
        nsf_msg.altitude = swri['altitude']
        nsf_msg.header.frame_id = "/far_field"
        nsf_msg.header.stamp = msg_stamp
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            nsf_pub.publish(nsf_msg)
            r.sleep()


class TestManualOrigin(TestInitializeOrigin):
    def testManualOrigin(self):
        rospy.init_node('test_initialize_origin')
        origin_sub = self.subscribeToOrigin()
        rospy.spin()

if __name__ == "__main__":
    if sys.argv[1] == "auto_gps":
        rostest.rosrun(PKG, NAME, TestAutoOriginFromGPSFix, sys.argv)
    elif sys.argv[1] == "manual":
        rostest.rosrun(PKG, NAME, TestManualOrigin, sys.argv)
    elif sys.argv[1] == "auto_navsat":
        rostest.rosrun(PKG, NAME, TestAutoOriginFromNavSatFix, sys.argv)

