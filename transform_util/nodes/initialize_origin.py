#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
#

import subprocess
import roslib; roslib.load_manifest('transform_util')
import rospy
from gps_common.msg import GPSFix
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Global variables
_has_origin = False
_latitude = 0
_longitude = 0
_altitude = 0

_sub = None

def gps_callback(data):   
    global _latitude
    global _longitude
    global _altitude
    global _has_origin
    
    if not _has_origin:
        _latitude = data.latitude
        _longitude = data.longitude
        _altitude = data.altitude
        _has_origin = True
        
        rospy.set_param('/local_xy_auto_latitude', _latitude)
        rospy.set_param('/local_xy_auto_longitude', _longitude)
        rospy.set_param('/local_xy_auto_altitude', _altitude)
        rospy.set_param('/local_xy_auto_set', _has_origin)
        
        global _sub
        _sub.unregister()
        _sub = None

def initialize_origin():
    rospy.init_node('initialize_origin', anonymous=True)
   
    global _sub
    _sub = rospy.Subscriber("gps", GPSFix, gps_callback)
    pub = rospy.Publisher('/diagnostics', DiagnosticArray)
   
    hw_id = rospy.get_param('~hw_id', 'none') 
    
    while not rospy.is_shutdown():
        if not _has_origin:
            diagnostic = DiagnosticArray()
            diagnostic.header.stamp = rospy.Time.now()
        
            status = DiagnosticStatus()
        
            status.name = "LocalXY Origin"
            status.hardware_id = hw_id
        
            status.level = DiagnosticStatus.ERROR
            status.message = "No Origin"
                
            diagnostic.status.append(status)
        
            pub.publish(diagnostic)
        else:
            diagnostic = DiagnosticArray()
            diagnostic.header.stamp = rospy.Time.now()
        
            status = DiagnosticStatus()
        
            status.name = "LocalXY Origin"
            status.hardware_id = hw_id
        
            status.level = DiagnosticStatus.OK
            status.message = "Has Origin"
                        
            value1 = KeyValue()
            value1.key = "Latitude"
            value1.value = "%f" % _latitude
            status.values.append(value1)
        
            value2 = KeyValue()
            value2.key = "Longitude"
            value2.value = "%f" % _longitude
            status.values.append(value2)
        
            value3 = KeyValue()
            value3.key = "Altitude"
            value3.value = "%f" % _altitude
            status.values.append(value3)
        
            diagnostic.status.append(status)
        
            pub.publish(diagnostic)
        
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        initialize_origin()
    except rospy.ROSInterruptException: pass
