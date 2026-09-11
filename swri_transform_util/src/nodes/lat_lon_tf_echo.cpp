// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <cmath>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geographic_msgs/msg/geo_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <swri_math_util/constants.h>
#include <swri_transform_util/local_xy_util.h>

/**
 * @file
 *
 * This is a lattitude/longitude analog to the tf_echo node in the tf
 * package. When run in a console, it periodically outputs the latitude,
 * longitude, and heading of the desired TF.
 *
 * <b>Usage:</b>
 * lat_lon_tf_echo fixed_frame_id target_frame_id
 *
 * fixed_frame_id is the id of the frame fixed at the /local_xy_origin, usually
 *    /far_field
 * target_frame_id is the id of the frame to find the coordinates of
 *
 * All outputs are in degrees, and heading follows the compass heading
 * convention (0° is North, clockwise angles are positive)
 *
 * <b>Subscribed Topics</b>
 * - \e /tf [geometry_msgs::Transform] - The transform from fixed_frame_id to
 *        target_frame_id must be published
 * - \e /local_xy_origin [gps_msgs::GPSFix, geographic_msgs::GeoPose, or
 *        geometry_msgs::PoseStamped] - This topic is used to initialize the
 *        WGS84 transformer. The node subscribes with whichever of these types
 *        the first publisher it discovers uses, and disconnects once it is
 *        initialized.
 *        The fields of a PoseStamped should be filled as follows:
 *        - pose.position.x - longitude in degrees east of the prime meridian
 *        - pose.position.y - lattitude in degrees north of the equator.
 *        - pose.position.z - altitude in meters above the WGS84 ellipsoid
 *        - pose.orientation - its yaw is the direction of the local X axis,
 *          counter-clockwise from east
 *        All other fields in the message are ignored.
 */

class LatLonTFEchoNode : public rclcpp::Node
{
public:
  LatLonTFEchoNode(
      std::string frame_id,
      std::string fixed_frame) :
      rclcpp::Node("lat_lon_tf_echo"),
      buffer_(this->get_clock()),
      tf_listener_(buffer_),
      frame_id_(frame_id),
      fixed_frame_(fixed_frame)
  {
    // /local_xy_origin may carry any of several message types, but DDS allows
    // only one type per topic within a process, so wait for a publisher to
    // show which type it uses before subscribing.
    origin_discovery_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(250),
        std::bind(&LatLonTFEchoNode::SubscribeToOrigin, this));

    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&LatLonTFEchoNode::TimerCallback, this));
  }

private:
  static constexpr const char* kOriginTopic = "/local_xy_origin";

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr origin_discovery_timer_;
  std::set<std::string> unsupported_origin_types_;  // Already warned about.
  std::shared_ptr<swri_transform_util::LocalXyWgs84Util> xy_wgs84_util_;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPose>::SharedPtr geopose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posestamped_sub_;
  std::string frame_id_;
  std::string fixed_frame_;

  void SubscribeToOrigin()
  {
    for (const auto& info : this->get_publishers_info_by_topic(kOriginTopic))
    {
      // Match the publisher's reliability and durability so that the
      // subscription is compatible with it, and so that a latched origin
      // published before this node started is still delivered.
      const rmw_qos_profile_t& publisher_qos = info.qos_profile().get_rmw_qos_profile();
      rclcpp::QoS qos(1);
      qos.reliability(publisher_qos.reliability);
      qos.durability(publisher_qos.durability);

      const std::string& type = info.topic_type();
      if (type == "gps_msgs/msg/GPSFix")
      {
        gps_sub_ = this->create_subscription<gps_msgs::msg::GPSFix>(
            kOriginTopic, qos,
            std::bind(&LatLonTFEchoNode::HandleGpsFix, this, std::placeholders::_1));
      }
      else if (type == "geographic_msgs/msg/GeoPose")
      {
        geopose_sub_ = this->create_subscription<geographic_msgs::msg::GeoPose>(
            kOriginTopic, qos,
            std::bind(&LatLonTFEchoNode::HandleGeoPose, this, std::placeholders::_1));
      }
      else if (type == "geometry_msgs/msg/PoseStamped")
      {
        posestamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            kOriginTopic, qos,
            std::bind(&LatLonTFEchoNode::HandlePoseStamped, this, std::placeholders::_1));
      }
      else
      {
        // Keep polling in case a supported publisher appears, but say why the
        // node is still waiting. Once per type, since this runs every poll.
        if (unsupported_origin_types_.insert(type).second)
        {
          RCLCPP_WARN(this->get_logger(),
              "%s is published as %s; expected gps_msgs/msg/GPSFix, "
              "geographic_msgs/msg/GeoPose or geometry_msgs/msg/PoseStamped",
              kOriginTopic, type.c_str());
        }
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Subscribing to %s as %s", kOriginTopic, type.c_str());
      origin_discovery_timer_->cancel();
      return;
    }
  }

  void HandleGpsFix(const gps_msgs::msg::GPSFix::UniquePtr msg)
  {
    xy_wgs84_util_.reset(
        new swri_transform_util::LocalXyWgs84Util(
            msg->latitude,
            msg->longitude,
            // The constructor takes degrees ENU (counter-clockwise from
            // east); track is a compass heading (clockwise from north).
            90.0 - msg->track,
            msg->altitude));
    Unsubscribe();
  }

  void HandleGeoPose(const geographic_msgs::msg::GeoPose::UniquePtr msg)
  {
    xy_wgs84_util_.reset(
        new swri_transform_util::LocalXyWgs84Util(
            msg->position.latitude,
            msg->position.longitude,
            // The constructor takes degrees; getYaw() returns radians.
            tf2::getYaw(msg->orientation) * swri_math_util::_rad_2_deg,
            msg->position.altitude));
    Unsubscribe();
  }

  void HandlePoseStamped(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
  {
    xy_wgs84_util_.reset(
        new swri_transform_util::LocalXyWgs84Util(
            msg->pose.position.y,    // Latitude
            msg->pose.position.x,    // Longitude
            // The orientation's yaw is the heading in radians ENU; the
            // constructor takes degrees.
            tf2::getYaw(msg->pose.orientation) * swri_math_util::_rad_2_deg,
            msg->pose.position.z));  // Altitude
    Unsubscribe();
  }

  void Unsubscribe()
  {
    gps_sub_.reset();
    geopose_sub_.reset();
    posestamped_sub_.reset();
  }

  void TimerCallback()
  {
    if (!xy_wgs84_util_ || !xy_wgs84_util_->Initialized())
    {
      printf("Still waiting for /local_xy_origin\n");
      return;
    }
    geometry_msgs::msg::TransformStamped transform_msg;
    try
    {
      transform_msg =
          buffer_.lookupTransform(fixed_frame_,
                                  frame_id_,
                                  tf2::TimePointZero,
                                  std::chrono::seconds(1));
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }
    double lat, lon;
    tf2::Stamped<tf2::Transform> transform;
    tf2::fromMsg(transform_msg, transform);
    xy_wgs84_util_->ToWgs84(
        transform.getOrigin().x(), transform.getOrigin().y(),
        lat, lon);
    // The yaw is counter-clockwise from the fixed frame's X axis, which lies
    // ReferenceAngle() degrees counter-clockwise from east, while a compass
    // heading is clockwise from north.
    double yaw = tf2::getYaw(transform.getRotation()) * swri_math_util::_rad_2_deg;
    double heading = std::fmod(90.0 - (yaw + xy_wgs84_util_->ReferenceAngle()), 360.0);
    if (heading < 0)
    {
      heading += 360;
    }
    printf("Latitude: %f°, Longitude: %f°, Heading: %f°\n", lat, lon, heading);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  if (argc < 3)
  {
    printf("Usage: lat_lon_tf_echo <fixed_frame_id> <target_frame_id>\n");
    return 1;
  }
  std::string fixed_frame(argv[1]);
  std::string frame_id(argv[2]);
  std::shared_ptr<LatLonTFEchoNode> node = std::make_shared<LatLonTFEchoNode>(
      frame_id, fixed_frame);
  rclcpp::spin(node);

  return (0);
}
