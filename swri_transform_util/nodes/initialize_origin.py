#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
#

import rospy
from gps_common.msg import GPSFix
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Global variables
_origin = None

def make_origin_msg(frame_id, latitude, longitude, altitude):
    gps_fix = GPSFix()
    gps_fix.header.frame_id = frame_id
    gps_fix.latitude = latitude
    gps_fix.longitude = longitude
    gps_fix.altitude = altitude
    return gps_fix

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
    global _origin
    rospy.loginfo("Got GPS message. Setting origin and unsubscribing from GPS.")
    sub.unregister()
    if _origin is None:
        _origin = make_origin_msg(frame_id,
                                  data.latitude,
                                  data.longitude,
                                  data.altitude)
        pub.publish(_origin)

def make_diagnostic(gps_fix, static_origin):
    diagnostic = DiagnosticArray()
    diagnostic.header.stamp = rospy.Time.now()
    status = DiagnosticStatus()
    status.name = "LocalXY Origin"
    status.hardware_id = "origin_publisher"
    if gps_fix == None:
        status.level = DiagnosticStatus.ERROR
        status.message = "No Origin"
    else:
        if static_origin:
            status.level = DiagnosticStatus.OK
            status.message = "Has Origin (auto)"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "Origin is static (non-auto)"
        
        frame_id = gps_fix.header.frame_id
        status.values.append(KeyValue(key="Origin Frame ID", value=frame_id))
        
        latitude = "%f" % gps_fix.latitude
        status.values.append(KeyValue(key="Latitude", value=latitude))
        
        longitude = "%f" % gps_fix.longitude
        status.values.append(KeyValue(key="Longitude", value=longitude))
        
        altitude = "%f" % gps_fix.altitude
        status.values.append(KeyValue(key="Altitude", value=altitude))
    
    diagnostic.status.append(status)
    return diagnostic

def initialize_origin():
    global _origin
    rospy.init_node('initialize_origin', anonymous=True)
    origin_pub = rospy.Publisher('/local_xy_origin', GPSFix, latch=True)
    diagnostic_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
    origin_name = rospy.get_param('~local_xy_origin', 'auto')
    rospy.loginfo('Local XY origin is "' + origin_name + '"')
    origin_frame_id = rospy.get_param('local_xy_frame', 'map')
    rospy.loginfo('Local XY frame ID is "' + origin_frame_id + '"')
    
    if origin_name != "auto":
        origin_list = rospy.get_param('~local_xy_origins', [])
        _origin = make_origin_from_list(origin_frame_id, origin_name, origin_list)
        if _origin is not None:
            origin_pub.publish(_origin)
        else:
            origin_name = "auto"
    if origin_name == "auto":
        sub = rospy.Subscriber("gps", GPSFix)
        sub.impl.add_callback(gps_callback, (origin_frame_id, origin_pub, sub))
        rospy.loginfo('Subscribed to GPS on ' + sub.resolved_name)
    while not rospy.is_shutdown():
        diagnostic_pub.publish(make_diagnostic(_origin, (origin_name != "auto")))
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        initialize_origin()
    except rospy.ROSInterruptException: pass
