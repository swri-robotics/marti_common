# -*- coding: utf-8 -*-

"""A module containing a simple class for transforming from WGS84 coordinate
frame to a local_xy frame and vice versa."""

# Copyright (C) 2020, Southwest Research Institute® (SwRI®)
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

import math
import numpy as np

EARTH_ECCENTRICITY = 0.08181919084261
EARTH_EQUATOR_RADIUS = 6378137.0


# TODO: Use library implementation similar to euler_from_quaternion when available in ROS2
def yaw_from_quaternion(quaternion):
    """
    Converts a quartenion into a yaw value
    :param quaternion: list of quaternion values in (x,y,z,w) order
    :return: The yaw angle (in radians) of the quaternion
    """
    x, y, z, w = quaternion
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return yaw


class Wgs84Transformer(object):
    """
    A simple class for transforming from the WGS84 frame to the local_xy frame
    and vice versa.

    Transforms to and from WGS84 and the local_xy frame using a local origin
    as a reference.
    """

    def __init__(self, local_origin):
        """
        Constructor for the Wgs84Transformer class
        :param geometry_msgs.Pose local_origin: An initialized local origin
        """
        self._reference_heading = yaw_from_quaternion(
            quaternion=(local_origin.pose.orientation.x,
                        local_origin.pose.orientation.y,
                        local_origin.pose.orientation.z,
                        local_origin.pose.orientation.w))  # get yaw from quaternion
        self._cos_heading = math.cos(self._reference_heading)
        self._sin_heading = math.sin(self._reference_heading)
        self._reference_latitude = local_origin.pose.position.y * math.pi / 180
        self._reference_longitude = local_origin.pose.position.x * math.pi / 180
        self._depth = -local_origin.pose.position.z

        p = EARTH_ECCENTRICITY * math.sin(self._reference_latitude)
        p = 1.0 - p**2

        rho_e_num = EARTH_EQUATOR_RADIUS * (1.0 - EARTH_ECCENTRICITY**2)
        rho_e_den = math.sqrt(p) * p
        rho_e = rho_e_num / rho_e_den
        rho_n = EARTH_EQUATOR_RADIUS / math.sqrt(p)

        self._rho_lat = rho_e - self._depth
        self._rho_lon = (rho_n - self._depth) * math.cos(self._reference_latitude)

    def wgs84_to_local_xy(self, wgs84_points):
        """
        Transforms point(s) in the WGS84 coordinate frame to the local_xy frame.
        :param list wgs84_points: list of (latitude, longitude) coordinates
        :return: The transformed list of (x, y) coordinates in the local_xy frame
        """
        wgs84_points = np.array(wgs84_points)

        r = wgs84_points * math.pi / 180.0

        d = (r - [self._reference_latitude, self._reference_longitude]) * [self._rho_lat,
                                                                           self._rho_lon]

        points = d.dot([[self._sin_heading, self._cos_heading],
                        [self._cos_heading, -self._sin_heading]])

        return points

    def local_xy_to_wgs84(self, local_points):
        """
        Transforms point(s) in the local_xy frame to the WGS84 coordinate frame
        :param list local_points: list of (x, y) coordinates
        :return: The transformed list of (latitude, longitude) coordinates in the WGS84 frame
        """
        points = np.array(local_points)

        d = points.dot([[self._sin_heading, self._cos_heading],
                        [self._cos_heading, -self._sin_heading]])

        r = d / [self._rho_lat, self._rho_lon] + [self._reference_latitude,
                                                  self._reference_longitude]

        wgs84_points = r * 180.0 / math.pi

        return wgs84_points
