#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
#

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Global variables
_origin = None

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

def gps_callback(data, (frame_id, pub, sub)):
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
        if static_origin:
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
        sub.impl.add_callback(gps_callback, (origin_frame_id, origin_pub, sub))
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

        diagnostic_pub.publish(make_diagnostic(_origin, (origin_name != "auto")))
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        initialize_origin()
    except rospy.ROSInterruptException: pass
