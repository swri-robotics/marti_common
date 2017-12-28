#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) 2017, Southwest Research Institute® (SwRI®)
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

from gps_common.msg import GPSFix
import rospy
from sensor_msgs.msg import NavSatFix
from swri_transform_util.origin_manager import OriginManager, InvalidFixException


def navsat_callback(msg, (manager, navsat_sub, gps_sub)):
        try:
            manager.set_origin_from_navsat(msg)
        except InvalidFixException as e:
            rospy.logwarn(e)
            return
        finally:
            rospy.loginfo('Got NavSat message. Setting origin and unsubscribing from NavSat.')
            navsat_sub.unregister()
            gps_sub.unregister()


def gps_callback(msg, (manager, gps_sub, navsat_sub)):
        try:
            manager.set_origin_from_gps(msg)
        except InvalidFixException as e:
            rospy.logwarn(e)
            return
        finally:
            rospy.loginfo('Got GPSFix message. Setting origin and unsubscribing from GPSFix.')
            gps_sub.unregister()
            navsat_sub.unregister()


rospy.init_node('initialize_origin', anonymous=True)
local_xy_frame = rospy.get_param('~local_xy_frame', 'map')
local_xy_origin = rospy.get_param('~local_xy_origin', 'auto')
manager = OriginManager(local_xy_frame)
if local_xy_origin == 'auto':
    gps_sub = rospy.Subscriber('gps', GPSFix, queue_size=2)
    navsat_sub = rospy.Subscriber('fix', NavSatFix, queue_size=2)
    gps_sub.impl.add_callback(gps_callback,
                              (manager, gps_sub, navsat_sub))  # Extra arguments to callback
    navsat_sub.impl.add_callback(navsat_callback,
                                 (manager, navsat_sub, gps_sub))  # Extra arguments to callback
else:
    try:
        origin_list = rospy.get_param('~local_xy_origins')
    except KeyError:
        message = 'local_xy_frame is "{}", but local_xy_origins is not specified'
        rospy.logfatal(message.format(local_xy_frame))
        exit(1)
    try:
        manager.set_origin_from_list(local_xy_origin, origin_list)
    except (TypeError, KeyError) as e:
        message = 'local_xy_origins is malformed or does not contain the local_xy_frame "{}"'
        rospy.logfatal(message.format(local_xy_frame))
        rospy.logfatal(e)
        exit(1)
manager.start()
rospy.spin()
