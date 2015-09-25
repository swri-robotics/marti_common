#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
#

import subprocess
import roslib; roslib.load_manifest('swri_transform_util')
import rospy
from gps_common.msg import GPSFix
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Global variables
_gps_fix = None
_local_xy_frame = None

_sub = None
_origin_pub = None

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
            
            _origin_pub.publish(_gps_fix)

    return

def gps_callback(data):
    global _gps_fix
    
    if _gps_fix == None:
        global _sub
        _sub.unregister()
        _sub = None
       
        _gps_fix = data
        _gps_fix.status.header.frame_id = "auto"
        
        _origin_pub.publish(_gps_fix)

def initialize_origin():
    rospy.init_node('initialize_origin', anonymous=True)
   
    global _origin_pub
    global _local_xy_frame
    _origin_pub = rospy.Publisher('/local_xy_origin', GPSFix, latch=True)
    
    diagnostic_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
   
    local_xy_origin = rospy.get_param('~local_xy_origin', 'auto')
    _local_xy_frame = rospy.get_param('~local_xy_frame', 'map')
   
    if local_xy_origin == "auto":
        global _sub
        _sub = rospy.Subscriber("gps", GPSFix, gps_callback)
    else:
        parse_origin(local_xy_origin)
   
    hw_id = rospy.get_param('~hw_id', 'none') 
    
    while not rospy.is_shutdown():
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

            if _gps_fix.status.header.frame_id == 'auto':
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
if __name__ == '__main__':
    try:
        initialize_origin()
    except rospy.ROSInterruptException: pass
