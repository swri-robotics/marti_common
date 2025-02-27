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

from gps_msgs.msg import GPSFix
from math import nan
import rclpy
import rclpy.exceptions
import rclpy.node
from sensor_msgs.msg import NavSatFix
from swri_transform_util.origin_manager import OriginManager, InvalidFixException
import sys
import yaml


class OriginInitializer(rclpy.node.Node):
    def __init__(self):
        super().__init__('initialize_origin')

        self.local_xy_frame_param = self.declare_parameter('local_xy_frame',
                                                           'map',
                                                           descriptor=rclpy.node.ParameterDescriptor(
                                                               name='local_xy_frame',
                                                               type=rclpy.parameter.ParameterType.PARAMETER_STRING
                                                           ))
        self.local_xy_origin_param = self.declare_parameter('local_xy_origin',
                                                            'auto',
                                                            descriptor=rclpy.node.ParameterDescriptor(
                                                                name='local_xy_frame',
                                                                type=rclpy.parameter.ParameterType.PARAMETER_STRING
                                                            ))

        self.gpsfix_topic_param = self.declare_parameter('local_xy_gpsfix_topic',
                                                         'gps',
                                                         descriptor=rclpy.node.ParameterDescriptor(
                                                             name='local_xy_gpsfix_topic',
                                                             type=rclpy.parameter.ParameterType.PARAMETER_STRING
                                                         ))
        self.navsatfix_topic_param = self.declare_parameter('local_xy_navsatfix_topic',
                                                            'fix',
                                                            descriptor=rclpy.node.ParameterDescriptor(
                                                                name='local_xy_navsatfix_topic',
                                                                type=rclpy.parameter.ParameterType.PARAMETER_STRING
                                                            ))
        self.local_xy_origins_param = self.declare_parameter('local_xy_origins',
                                                             [nan, nan, nan, nan],
                                                             descriptor=rclpy.node.ParameterDescriptor(
                                                                 name='local_xy_origins',
                                                                 dynamic_typing=True
                                                             ))

        self.get_logger().info("Origin: %s" % self.local_xy_origin_param.value)
        self.get_logger().info("Frame: %s" % self.local_xy_frame_param.value)

        self.manager = OriginManager(self, self.local_xy_frame_param.value)
        if self.local_xy_origin_param.value == 'auto':

            local_xy_gpsfix_topic = self.gpsfix_topic_param.value
            gps_sub = self.create_subscription(GPSFix,
                                               local_xy_gpsfix_topic,
                                               self.gps_callback, 2)

            local_xy_navsatfix_topic = self.navsatfix_topic_param.value
            navsat_sub = self.create_subscription(NavSatFix,
                                                  local_xy_navsatfix_topic,
                                                  self.navsat_callback, 2)
            self.subscribers = [gps_sub, navsat_sub]
        elif type(self.local_xy_origins_param.value) == list:
            if (len(self.local_xy_origins_param.value) != 4):
               self.get_logger().fatal(f'{self.local_xy_origins_param.name} should have len 4 [lat, lon, alt, heading]')
               exit(1)
            self.manager.set_origin("manual", *self.local_xy_origins_param.value[0:3])
        elif type(self.local_xy_origins_param.value) == str:
            try:
                origins_list = yaml.safe_load(self.local_xy_origins_param.value)
                self.manager.set_origin_from_list(self.local_xy_origin_param.value, origins_list)
            except (TypeError, KeyError) as e:
                message = '{} is malformed or does not contain the {} "{}"'
                self.get_logger().fatal(message.format(self.local_xy_origins_param.name,
                                                       self.local_xy_origin_param.name,
                                                       self.local_xy_origin_param.value))
                self.get_logger().fatal(str(e))
                exit(1)
            except (yaml.parser.ParserError) as e:
                self.get_logger().fatal(f"'{self.local_xy_origins_param.name}' string value is malformed")
                self.get_logger().fatal(f"yaml error: {str(e)}")
                exit(1)
        else:
            self.get_logger().fatal(f"Parameter '{self.local_xy_origins_param.name}' has incorrect type. Expected: string or double array")
            exit(1)
        self.manager.start()

    def navsat_callback(self, msg):
        try:
            self.get_logger().info('Got NavSat message.')
            self.manager.set_origin_from_navsat(msg)
            self.get_logger().info('Successfully set origin; unsubscribing.')
            while self.subscribers:
                sub = self.subscribers.pop()
                self.destroy_subscription(sub)
        except InvalidFixException as e:
            self.get_logger().warn("%s" % str(e))
            return

    def gps_callback(self, msg):
        try:
            self.get_logger().info('Got GPSFix message.')
            self.manager.set_origin_from_gps(msg)
            self.get_logger().info('Successfully set origin; unsubscribing.')
            while self.subscribers:
                sub = self.subscribers.pop()
                self.destroy_subscription(sub)
        except InvalidFixException as e:
            self.get_logger().warn("%s" % str(e))
            return


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = OriginInitializer()
    rclpy.spin(node)
