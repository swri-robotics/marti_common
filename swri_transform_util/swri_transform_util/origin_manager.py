# -*- coding: utf-8 -*-

"""A module containing a simple class for publishing the Local XY origin as a PoseStamped."""

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

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from gps_msgs.msg import GPSStatus
import rclpy.node
import rclpy.qos
from sensor_msgs.msg import NavSatStatus
from tf2_ros import TransformBroadcaster


class InvalidFixException(Exception):
    """Exception to throw when a GPSFix or NavSatFix message does not contain a valid fix."""

    pass


class OriginManager(object):
    """
    A simple class for publishing the Local XY origin as a PoseStamped.

    Creates publishers to `/local_xy_origin` (geometry_msgs/PoseStamped)
    and `/diagnostics` (diagnostic_msgs/DiagnosticArray). Publishes an
    identity TF from the origin to support /tf<->/wgs84 conversions.

    Usage (manual origin):
        manager = OriginManager("/map")
        manager.set_origin(29.45196669, -98.61370577, 233.719)
        manager.start()

    Usage (manual origin from parameter):
        local_xy_origin = rospy.get_param('~local_xy_origin')
        manager = OriginManager("map")
        origin_list = rospy.get_param('~local_xy_origins')
        manager.set_origin_from_list(local_xy_origin, origin_list)
        manager.start()

    Usage (auto origin from NavSatFix):
        def navsat_callback(msg, params):
            (manager, navsat_sub) = params
            try:
                manager.set_origin_from_navsat(msg)
            except InvalidFixException as e:
                rospy.logwarn(e)
                return
            finally:
                navsat_sub.unregister()
        manager = OriginManager("/map")
        navsat_sub = rospy.Subscriber("fix", NavSatFix, queue_size=2)
        navsat_sub.impl.add_callback(navsat_callback, (manager, navsat_sub))
        manager.start()

    Usage (auto origin from GPSFix):
        def gps_callback(msg, params):
            (manager, gps_sub) = params
            try:
                manager.set_origin_from_gps(msg)
            except InvalidFixException as e:
                rospy.logwarn(e)
                return
            finally:
                gps_sub.unregister()
        manager = OriginManager("/map")
        gps_sub = rospy.Subscriber("gps", GPSFix, queue_size=2)
        gps_sub.impl.add_callback(gps_callback, (manager, gps_sub))
        manager.start()

    Usage (auto origin from custom topic):
        def navsat_callback(msg, params):
            (manager, custom_sub) = params
            pos = (msg.latitude, msg.longitude, mgs.altitude)
            manager.set_origin_from_custom(pos, msg.header.stamp)
            custom_sub.unregister()
        manager = OriginManager("/map")
        custom_sub = rospy.Subscriber(custom_topic, CustomMsg, queue_size=2)
        custom_sub.impl.add_callback(custom_callback, (manager, custom_sub))
        manager.start()
    """

    def __init__(self, node, local_xy_frame, local_xy_frame_identity=None):
        """
        Construct an OriginManager, create publishers and TF Broadcaster.

        Args:
            node (rclpy.node.Node): ROS node for subscribing
            local_xy_frame (str): TF frame ID of the local XY origin
            local_xy_frame_identity (str): TF frame ID of the identity frame published
                to enable tf<->/wgs_84 conversions in systems with no other frames. If
                this argument is `None`, `local_xy_frame` + "__identity" is used.
                (default None).
        """
        self.node = node
        self.timer = None
        self.origin = None
        self.origin_source = None
        self.local_xy_frame = local_xy_frame
        if local_xy_frame_identity is None:
            local_xy_frame_identity = local_xy_frame + "__identity"
        self.local_xy_frame_identity = local_xy_frame_identity
        qos = rclpy.node.QoSProfile(depth=1,
                                    durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.origin_pub = node.create_publisher(PoseStamped, '/local_xy_origin', qos_profile=qos)
        self.diagnostic_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 2)
        self.tf_broadcaster = TransformBroadcaster(self.node)

    def set_origin(self, source, latitude, longitude, altitude, stamp=None):
        """
        Set the origin to the position described by the arguments and publish it.

        All other set_ methods wrap this method.

        Args:
            source (string): The source of the origin.
            latitude (float): The latitude of the origin in degrees.
            longitude (float): The longitude of the origin in degrees.
            altitude (float): The altitude of the origin in meters.
                Positive values correspond to altitude above the geoid.
            stamp (rospy.Time): The time to use for the origin's header.stamp field.
                If the argument is `None`, the stamp is not set and defaults to
                `rospy.Time(0)`. (default None).
        """
        if self.origin is not None:
            return
        origin = PoseStamped()
        origin.header.frame_id = self.local_xy_frame
        if stamp is not None:
            origin.header.stamp = stamp
        origin.pose.position.y = latitude
        origin.pose.position.x = longitude
        origin.pose.position.z = altitude
        # Heading is always 0
        origin.pose.orientation.x = 0.0
        origin.pose.orientation.y = 0.0
        origin.pose.orientation.z = 0.0
        origin.pose.orientation.w = 1.0
        self.origin_source = source
        self.origin = origin
        self.node.get_logger().info("Origin from '{}' source set to {}, {}, {}".format(
            source, latitude, longitude, altitude))
        self._publish_origin()

    def _publish_origin(self):
        if self.origin is not None:
            self.origin_pub.publish(self.origin)

    def set_origin_from_dict(self, origin_dict):
        """
        Set the local origin from a dictionary.

        Args:
            origin_dict (dict): A dictionary containing the keys "latitude", "longitude", and
                "altitude" with appropriate float values

        Raises:
            KeyError: If `origin_dict` does not contain all of the required keys.
        """
        self.set_origin("manual", origin_dict["latitude"],
                        origin_dict["longitude"],
                        origin_dict["altitude"])

    def set_origin_from_list(self, origin_name, origin_list):
        """
        Set the local origin from a list of named origins.

        Args:
            origin_name (str): The name of the origin in origin_list to use
            origin_list (list): A list of dicts containing a name key and all keys required
                by set_origin_from_dict()

        Raises:
            KeyError: If any origin in `origins_list` lacks a "name" key or if there exists
                no origin in `origins_list` whose "name" is `origin_name`
        """
        origin = next((x for x in origin_list if x['name'] == origin_name), None)
        if origin:
            self.set_origin_from_dict(origin)
        else:
            origins_list_str = ', '.join(['"{}"'.format(x['name']) for x in origin_list])
            message = 'Origin "{}" is not in the origin list. Available origins are {}'.format(
                origin_name, origins_list_str)
            raise KeyError(message)

    def set_origin_from_gps(self, msg):
        """
        Set the local origin from a gps_common.msg.GPSFix object.

        Args:
            msg (gps_common.msg.GPSFix): A GPSFix message with the local origin

        Raises:
            InvalidFixException: If `msg.status.status` is STATUS_NO_FIX
        """
        if msg.status.status == GPSStatus.STATUS_NO_FIX:
            message = 'Cannot set origin from invalid GPSFix. Waiting for a valid one...'
            raise InvalidFixException(message)
        self.set_origin("gpsfix", msg.latitude, msg.longitude, msg.altitude, msg.header.stamp)

    def set_origin_from_navsat(self, msg):
        """
        Set the local origin from a sensor_msgs.msg.NavSatFix object.

        Args:
            msg (sensor_msgs.msg.NavSatFix): A NavSatFix message with the local origin

        Raises:
            InvalidFixException: If `msg.status.status` is STATUS_NO_FIX
        """
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            message = 'Cannot set origin from invalid NavSatFix. Waiting for a valid one...'
            raise InvalidFixException(message)
        self.set_origin("navsat", msg.latitude, msg.longitude, msg.altitude, msg.header.stamp)

    def set_origin_from_custom(self, pos, stamp):
        """
        Set the local origin from a custom object.

        Args:
            pos: Tuple with the local origin
            stamp: Msg header stamp if available
        """
        self.set_origin("custom", pos[0], pos[1], pos[2], stamp)

    def _publish_diagnostic(self):
        """Publish diagnostics."""
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.node.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "LocalXY Origin"
        status.hardware_id = "origin_publisher"
        if self.origin is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "No Origin"
        else:
            if self.origin_source == "gpsfix":
                status.level = DiagnosticStatus.OK
                status.message = "Has Origin (GPSFix)"
            elif self.origin_source == "navsat":
                status.level = DiagnosticStatus.OK
                status.message = "Has Origin (NavSatFix)"
            elif self.origin_source == "custom":
                status.level = DiagnosticStatus.OK
                status.message = "Has Origin (Custom Topic)"
            else:
                status.level = DiagnosticStatus.WARN
                status.message = "Origin Was Set Manually"

            frame_id = self.origin.header.frame_id
            status.values.append(KeyValue(key="Origin Frame ID", value=frame_id))

            latitude = "%f" % self.origin.pose.position.y
            status.values.append(KeyValue(key="Latitude", value=latitude))

            longitude = "%f" % self.origin.pose.position.x
            status.values.append(KeyValue(key="Longitude", value=longitude))

            altitude = "%f" % self.origin.pose.position.z
            status.values.append(KeyValue(key="Altitude", value=altitude))

            diagnostic.status.append(status)
            self.diagnostic_pub.publish(diagnostic)

    def _publish_identity_tf(self):
        """
        Publish a transform from the map frame to an anonymous unused frame.

        This allows the TransformManager to support /tf<->/wgs84 conversions
        without requiring additional nodes.
        """
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = self.local_xy_frame_identity
        t.child_frame_id = self.local_xy_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def publish_messages(self):
        """
        Publish identity transform and diagnostics.

        This method can be called as a rospy timer callback.
        """
        self._publish_diagnostic()
        self._publish_identity_tf()
        self._publish_origin()

    def start(self):
        """
        Creates a timer that periodically publishes our messages.
        """
        self.timer = self.node.create_timer(1.0, self.publish_messages)
