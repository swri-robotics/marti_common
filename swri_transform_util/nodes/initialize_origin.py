#!/usr/bin/env python3
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

from importlib import import_module

from gps_msgs.msg import GPSFix
import rclpy
import rclpy.exceptions
import rclpy.node
from sensor_msgs.msg import NavSatFix
from swri_transform_util.origin_manager import OriginManager, InvalidFixException
import sys


class OriginInitializer(rclpy.node.Node):
    def __init__(self):
        super().__init__('initialize_origin')

        self.declare_parameter('local_xy_frame')
        self.declare_parameter('local_xy_origin')

        for name, param in self._parameters.items():
            print("%s: Value [%s]" % (name, param.value))

        local_xy_frame = self.get_parameter_or('local_xy_frame', 'map').value
        origin_param = self.get_parameter_or('local_xy_origin', 'auto')
        local_xy_origin = origin_param.value

        self.get_logger().info("Origin: %s" % local_xy_origin)

        manager = OriginManager(self, local_xy_frame)
        if local_xy_origin == 'auto':
            self.declare_parameter('local_xy_gpsfix_topic')
            self.declare_parameter('local_xy_navsatfix_topic')
            self.declare_parameter('local_xy_custom_topic')

            local_xy_gpsfix_topic = self.get_parameter_or('local_xy_gpsfix_topic', 'gps')
            gps_sub = self.create_subscription(GPSFix,
                                               local_xy_gpsfix_topic.value,
                                               self.gps_callback, 2)

            local_xy_navsatfix_topic = self.get_parameter_or('local_xy_navsatfix_topic', 'fix')
            navsat_sub = self.create_subscription(NavSatFix,
                                                  local_xy_navsatfix_topic.value,
                                                  self.navsat_callback, 2)
            self.subscribers = [gps_sub, navsat_sub]
            # try:
            #     local_xy_custom_topic = self.get_parameter('local_xy_custom_topic')
            #     custom_sub = self.create_subscription(rclpy.local_xy_custom_topic, rospy.AnyMsg, queue_size=2)
            #     subscribers.append(custom_sub)
            # except rclpy.exceptions.ParameterException:
            #     pass

            # Add extra arguments to callback
        else:
            self.declare_parameter('local_xy_origins')
            try:
                origin_list = self.get_parameter('local_xy_origins').value
            except rclpy.exceptions.ParameterException:
                message = 'local_xy_origin is "{}", but local_xy_origins is not specified'
                self.get_logger().fatal(message.format(local_xy_origin))
                exit(1)
            try:
                manager.set_origin_from_list(local_xy_origin, origin_list)
            except (TypeError, KeyError) as e:
                message = 'local_xy_origins is malformed or does not contain the local_xy_origin "{}"'
                self.get_logger().fatal(message.format(local_xy_origin))
                self.get_logger().fatal(e)
                exit(1)
        manager.start()

    def navsat_callback(self, msg, params):
        (manager, subscribers) = params
        try:
            while self.subscribers:
                self.subscribers.pop()
            self.get_logger().info('Got NavSat message. Setting origin and unsubscribing.')
            manager.set_origin_from_navsat(msg)
        except InvalidFixException as e:
            self.get_logger().warn(e)
            return

    def gps_callback(self, msg, params):
        (manager, subscribers) = params
        try:
            while self.subscribers:
                self.subscribers.pop()
            self.get_logger().info('Got GPSFix message. Setting origin and unsubscribing.')
            manager.set_origin_from_gps(msg)
        except InvalidFixException as e:
            self.get_logger().warn(e)
            return

    def custom_callback(self, params):
        (manager, subscribers) = params
        connection_header = self._connection_header['type'].split('/')
        ros_pkg = connection_header[0] + '.msg'
        msg_type = connection_header[1]
        msg_class = getattr(import_module(ros_pkg), msg_type)
        msg = msg_class().deserialize(self._buff)
        stamp = None
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            stamp = msg.header.stamp
        if hasattr(msg, 'pose'):  # Messages like GeoPoseStamped
            msg = msg.pose
        if hasattr(msg, 'position'):  # Messages like GeoPose
            msg = msg.position
        pos = None
        if hasattr(msg, 'latitude') and hasattr(msg, 'longitude') and hasattr(msg, 'altitude'):
            pos = (msg.latitude, msg.longitude, msg.altitude)
        elif hasattr(msg, 'lat') and hasattr(msg, 'lon') and hasattr(msg, 'height'):
            pos = (msg.lat, msg.lon, msg.height)

        if pos:
            while self.subscribers:
                self.subscribers.pop()
            self.get_logger().info('Got {} message from topic "{}". Setting origin and unsubscribing.'
                          .format(self._connection_header['type'], self._connection_header['topic']))
            manager.set_origin_from_custom(pos, stamp)


def main():
    rclpy.init(args=sys.argv)
    node = OriginInitializer()
    # rospy.init_node('initialize_origin', anonymous=True)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
