#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) 2013-2017, Southwest Research Institute® (SwRI®)
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import rospy
import tf
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from geometry_msgs.msg import PoseStamped
from gps_common.msg import GPSFix
from sensor_msgs.msg import NavSatFix

# Global variables
_origin = None
_gps_fix = None

def make_origin_msg(frame_id, latitude, longitude, altitude):
    origin = PoseStamped()
    origin.header.frame_id = frame_id
    origin.pose.position.y = latitude
    origin.pose.position.x = longitude
    origin.pose.position.z = altitude
    # Heading is always 0
    origin.pose.orientation.x = 0.0
    origin.pose.orientation.y = 0.0
    origin.pose.orientation.z = 0.0
    origin.pose.orientation.w = 1.0
    return origin

def parse_origin(local_xy_origin):
    global _gps_fix

    local_xy_origins = rospy.get_param('~local_xy_origins', [])

    for origin in local_xy_origins:
        if origin["name"] == local_xy_origin:

            _gps_fix = GPSFix()
            _gps_fix.header.frame_id = _local_xy_frame
            _gps_fix.status.header.frame_id = local_xy_origin
            _gps_fix.latitude = origin["latitude"]
            _gps_fix.longitude = origin["longitude"]
            _gps_fix.altitude = origin["altitude"]
            _gps_fix.track = 90

            _origin_pub.publish(_gps_fix)

def make_origin_from_list(frame_id, origin_name, origin_list):
    if len(origin_list) == 0:
        rospy.logwarn("No origins specified--defaulting to auto")
        return None
    if len(origin_list) == 1:
        origin = origin_list[0]
        return make_origin_msg(frame_id,
                               origin["latitude"],
                               origin["longitude"],
                               origin["altitude"])
    for origin in origin_list:
        if origin["name"] == origin_name:
            return make_origin_msg(frame_id,
                                   origin["latitude"],
                                   origin["longitude"],
                                   origin["altitude"])
    rospy.logerror('Origin "{}" is not in the origin list. Available origins are {}.'.format(origin_name, ",".join(['"{}"'.format(x.name) for x in origin_list])))
    rospy.logerror("No origin found--defaulting to auto")
    return None

def navsatfix_callback(data, (frame_id, pub, sub)):
    if data.status.status == -1:
        # This fix is invalid, ignore it and wait until we get a valid one
        rospy.logwarn("Got invalid fix.  Waiting for a valid one...")
        return
    global _origin
    rospy.loginfo("Got NavSat message. Setting origin and unsubscribing from NavSat.")
    sub.unregister()
    if _origin is None:
        _origin = make_origin_msg(frame_id,
                                  data.latitude,
                                  data.longitude,
                                  data.altitude)
        pub.publish(_origin)

def gps_callback(data):
    if data.status.status == -1:
        # This fix is invalid, ignore it and wait until we get a valid one
        rospy.logwarn("Got invalid fix.  Waiting for a valid one...")
        return
    global _gps_fix

    if _gps_fix == None:
        global _sub
        _sub.unregister()
        _sub = None

        _gps_fix = data
        _gps_fix.header.frame_id = _local_xy_frame
        _gps_fix.track = 90

        _origin_pub.publish(_gps_fix)

def make_diagnostic(origin, static_origin):
    diagnostic = DiagnosticArray()
    diagnostic.header.stamp = rospy.Time.now()
    status = DiagnosticStatus()
    status.name = "LocalXY Origin"
    status.hardware_id = "origin_publisher"
    if origin == None:
        status.level = DiagnosticStatus.ERROR
        status.message = "No Origin"
    else:
        if not static_origin:
            status.level = DiagnosticStatus.OK
            status.message = "Has Origin (auto)"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "Origin is static (non-auto)"
        
        frame_id = origin.header.frame_id
        status.values.append(KeyValue(key="Origin Frame ID", value=frame_id))
        
        latitude = "%f" % origin.pose.position.y
        status.values.append(KeyValue(key="Latitude", value=latitude))
        
        longitude = "%f" % origin.pose.position.x
        status.values.append(KeyValue(key="Longitude", value=longitude))
        
        altitude = "%f" % origin.pose.position.z
        status.values.append(KeyValue(key="Altitude", value=altitude))
    
    diagnostic.status.append(status)
    return diagnostic

def initialize_origin():
    global _origin
    rospy.init_node('initialize_origin', anonymous=True)

    ros_distro = os.environ.get('ROS_DISTRO')

    if not ros_distro:
        rospy.logerror('ROS_DISTRO environment variable was not set.')
        exit(1)

    if ros_distro == 'indigo':
        # ROS Indigo uses the GPSFix message 
        global _origin_pub
        global _local_xy_frame
        _origin_pub = rospy.Publisher('/local_xy_origin', GPSFix, latch=True, queue_size=2)
    
        diagnostic_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=2)
    
        local_xy_origin = rospy.get_param('~local_xy_origin', 'auto')
        _local_xy_frame = rospy.get_param('~local_xy_frame', 'map')
        _local_xy_frame_identity = rospy.get_param('~local_xy_frame_identity', _local_xy_frame + "__identity")
    
        if local_xy_origin == "auto":
            global _sub
            _sub = rospy.Subscriber("gps", GPSFix, gps_callback)
        else:
            parse_origin(local_xy_origin)
    
        if len(_local_xy_frame):
            tf_broadcaster = tf.TransformBroadcaster()
        else:
            tf_broadcaster = None
    
        hw_id = rospy.get_param('~hw_id', 'none')
    
        while not rospy.is_shutdown():
            if tf_broadcaster:
                # Publish transform involving map (to an anonymous unused
                # frame) so that TransformManager can support /tf<->/wgs84
                # conversions without requiring additional nodes.
                tf_broadcaster.sendTransform(
                    (0, 0, 0),
                    (0, 0, 0, 1),
                    rospy.Time.now(),
                    _local_xy_frame_identity, _local_xy_frame)
    
            if _gps_fix == None:
                diagnostic = DiagnosticArray()
                diagnostic.header.stamp = rospy.Time.now()
    
                status = DiagnosticStatus()
    
                status.name = "LocalXY Origin"
                status.hardware_id = hw_id
    
                status.level = DiagnosticStatus.ERROR
                status.message = "No Origin"
    
                diagnostic.status.append(status)
    
                diagnostic_pub.publish(diagnostic)
            else:
                _origin_pub.publish(_gps_fix) # Publish this at 1Hz for bag convenience
                diagnostic = DiagnosticArray()
                diagnostic.header.stamp = rospy.Time.now()
    
                status = DiagnosticStatus()
    
                status.name = "LocalXY Origin"
                status.hardware_id = hw_id
    
                if local_xy_origin == 'auto':
                    status.level = DiagnosticStatus.OK
                    status.message = "Has Origin (auto)"
                else:
                    status.level = DiagnosticStatus.WARN
                    status.message = "Origin is static (non-auto)"
    
                value0 = KeyValue()
                value0.key = "Origin"
                value0.value = _gps_fix.status.header.frame_id
                status.values.append(value0)
    
                value1 = KeyValue()
                value1.key = "Latitude"
                value1.value = "%f" % _gps_fix.latitude
                status.values.append(value1)
    
                value2 = KeyValue()
                value2.key = "Longitude"
                value2.value = "%f" % _gps_fix.longitude
                status.values.append(value2)
    
                value3 = KeyValue()
                value3.key = "Altitude"
                value3.value = "%f" % _gps_fix.altitude
                status.values.append(value3)
    
                diagnostic.status.append(status)
    
                diagnostic_pub.publish(diagnostic)
            rospy.sleep(1.0)
    else:
        # ROS distros later than Indigo use NavSatFix
        origin_pub = rospy.Publisher('/local_xy_origin', PoseStamped, latch=True, queue_size=2)
        diagnostic_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=2)
        origin_name = rospy.get_param('~local_xy_origin', 'auto')
        rospy.loginfo('Local XY origin is "' + origin_name + '"')
        origin_frame_id = rospy.get_param(rospy.search_param('local_xy_frame'), 'map')
        origin_frame_identity = rospy.get_param('~local_xy_frame_identity', origin_frame_id + "__identity")
        rospy.loginfo('Local XY frame ID is "' + origin_frame_id + '"')
        
        if len(origin_frame_id):
            tf_broadcaster = tf.TransformBroadcaster()
        else:
            tf_broadcaster = None
    
        if origin_name != "auto":
            origin_list = rospy.get_param('~local_xy_origins', [])
            _origin = make_origin_from_list(origin_frame_id, origin_name, origin_list)
            if _origin is not None:
                origin_pub.publish(_origin)
            else:
                origin_name = "auto"
        if origin_name == "auto":
            sub = rospy.Subscriber("gps", NavSatFix)
            sub.impl.add_callback(navsatfix_callback, (origin_frame_id, origin_pub, sub))
            rospy.loginfo('Subscribed to NavSat on ' + sub.resolved_name)
        while not rospy.is_shutdown():
            if tf_broadcaster:
                # Publish transform involving map (to an anonymous unused
                # frame) so that TransformManager can support /tf<->/wgs84
                # conversions without requiring additional nodes.
                tf_broadcaster.sendTransform(
                    (0, 0, 0),
                    (0, 0, 0, 1),
                    rospy.Time.now(),
                    origin_frame_identity, origin_frame_id)
    
            diagnostic_pub.publish(make_diagnostic(_origin, origin_name != "auto"))
            rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        initialize_origin()
    except rospy.ROSInterruptException: pass
