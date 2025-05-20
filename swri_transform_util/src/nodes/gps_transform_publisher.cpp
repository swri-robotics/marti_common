// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
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

#include <functional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform_manager.h>
#include <tf2/transform_datatypes.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace swri_transform_util
{
  class GpsTransformPublisher : public rclcpp::Node
  {
  public:
    explicit GpsTransformPublisher(const rclcpp::NodeOptions& options);

    void HandleGps(const gps_msgs::msg::GPSFix::UniquePtr gps_fix);

  private:
    void InitTransformManager();
    void InitTransformBroadcaster();

    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buf_;

    std::shared_ptr<swri_transform_util::TransformManager> tf_manager_;
  };

  GpsTransformPublisher::GpsTransformPublisher(const rclcpp::NodeOptions& options) :
    rclcpp::Node("gps_transform_publisher", options)
  {
    this->declare_parameter("child_frame_id", "base_link");
    this->declare_parameter("parent_frame_id", "map");

    gps_sub_ = this->create_subscription<gps_msgs::msg::GPSFix>(
        "gps",
        100,
        std::bind(&GpsTransformPublisher::HandleGps, this, std::placeholders::_1));
  }

  void GpsTransformPublisher::InitTransformManager()
  {
    tf_manager_ = std::make_shared<swri_transform_util::TransformManager>(shared_from_this());
    tf_buf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_buf_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_, shared_from_this(), false);
    tf_manager_->Initialize(tf_buf_);
  }

  void GpsTransformPublisher::InitTransformBroadcaster()
  {
    tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  }

  void GpsTransformPublisher::HandleGps(const gps_msgs::msg::GPSFix::UniquePtr gps_fix)
  {
    tf2::Transform transform;

    // Get the orientation from the GPS track.
    // NOTE: This will be unreliable when the vehicle is stopped or moving at low
    //       speed.
    double yaw = (90.0 - gps_fix->track) * swri_math_util::_deg_2_rad;
    yaw = swri_math_util::WrapRadians(yaw, swri_math_util::_pi);
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, yaw);
    transform.setRotation(orientation);

    if (!tf_manager_)
    {
      InitTransformManager();
    }

    // Get the position by converting lat/lon to LocalXY.
    swri_transform_util::Transform to_local_xy;
    std::string global_frame = this->get_parameter("parent_frame_id").as_string();
    if (tf_manager_->GetTransform(global_frame, swri_transform_util::_wgs84_frame, tf2::TimePointZero, to_local_xy))
    {
      tf2::Vector3 position(gps_fix->longitude, gps_fix->latitude, gps_fix->altitude);
      position = to_local_xy * position;
      transform.setOrigin(position);

      geometry_msgs::msg::TransformStamped tf_stamped;
      tf_stamped.transform = tf2::toMsg(transform);
      tf_stamped.child_frame_id = this->get_parameter("child_frame_id").as_string();
      tf_stamped.header.frame_id = global_frame;
      tf_stamped.header.stamp = gps_fix->header.stamp;

      if (!tf_)
      {
        InitTransformBroadcaster();
      }

      tf_->sendTransform(tf_stamped);
    }
  }
}  // namespace swri_transform_util

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_transform_util::GpsTransformPublisher)
