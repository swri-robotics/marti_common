// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
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

#include <boost/smart_ptr.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>

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
 * - \e /local_xy_origin [geometry_msgs::PoseStamped] - This topic is used to 
 *        initialize the WGS84 transformer. Once it is initialized, the subscriber
 *        disconnects.
 *        The fields of this message should be filled as follows:
 *        - pose.position.y - longitude in degrees east of the prime meridian
 *        - pose.position.y - lattitude in degrees north of the equator.
 *        - pose.position.z - altitude in meters above the WGS84 ellipsoid
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
    auto gps_callback = [this](const gps_msgs::msg::GPSFix::UniquePtr msg) -> void
    {
      xy_wgs84_util_.reset(
          new swri_transform_util::LocalXyWgs84Util(
              msg->latitude,
              msg->longitude,
              msg->track,
              msg->altitude));
      Unsubscribe();
    };
    gps_sub_ = this->create_subscription<gps_msgs::msg::GPSFix>(
        "/local_xy_origin",
        1,
        gps_callback);

    auto geopose_callback = [this](const geographic_msgs::msg::GeoPose::UniquePtr msg) -> void
    {
      xy_wgs84_util_.reset(
          new swri_transform_util::LocalXyWgs84Util(
              msg->position.latitude,
              msg->position.longitude,
              tf2::getYaw(msg->orientation),
              msg->position.altitude));
      Unsubscribe();
    };
    geopose_sub_ = this->create_subscription<geographic_msgs::msg::GeoPose>(
        "/local_xy_origin",
        1,
        geopose_callback);

    auto posestamped_callback = [this](const geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void
    {
      xy_wgs84_util_.reset(
          new swri_transform_util::LocalXyWgs84Util(
              msg->pose.position.y,    // Latitude
              msg->pose.position.x,    // Longitude
              0.0,                        // Heading
              msg->pose.position.z));  // Altitude
      Unsubscribe();
    };
    posestamped_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/local_xy_origin",
        1,
        posestamped_callback
    );

    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&LatLonTFEchoNode::TimerCallback, this));
  }

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<swri_transform_util::LocalXyWgs84Util> xy_wgs84_util_;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPose>::SharedPtr geopose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posestamped_sub_;
  std::string frame_id_;
  std::string fixed_frame_;

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
    tf2::Quaternion q = transform.getRotation();
    q.setY(0);
    q.setX(0);
    q.normalize();
    double heading = -q.getAngle() * swri_math_util::_rad_2_deg + 90;
    while (heading < 0)
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
