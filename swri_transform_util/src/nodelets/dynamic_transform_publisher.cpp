// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#include <algorithm>
#include <functional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
//#include <swri_roscpp/dynamic_parameters.h>
#include <swri_roscpp/parameters.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace swri_transform_util
{

  class DynamicTransformPublisher : public rclcpp::Node
  {
  public:
    explicit DynamicTransformPublisher(const rclcpp::NodeOptions& options) :
        rclcpp::Node("dynamic_transform_publisher", options),
        tf_broadcaster_(*this),
        rate_(50),
        stamp_offset_(1.0)
    {
      swri::param(*this, "rate", rate_, rate_);
      swri::param(*this, "stamp_offset", stamp_offset_, stamp_offset_);
      swri::param(*this, "child_frame", child_frame_, "child");
      swri::param(*this, "parent_frame", parent_frame_, "parent");

      rcl_interfaces::msg::ParameterDescriptor y_desc;
      rcl_interfaces::msg::FloatingPointRange coord_range;
      coord_range.from_value = -10000.0;
      coord_range.to_value = 10000.0;
      y_desc.floating_point_range.push_back(coord_range);
      x_ = declare_parameter("x", 0.0, y_desc);
      y_ = declare_parameter("y", 0.0, y_desc);
      y_ = declare_parameter("z", 0.0, y_desc);

      rcl_interfaces::msg::ParameterDescriptor rotation_desc;
      rcl_interfaces::msg::FloatingPointRange rotation_range;
      coord_range.from_value = -3.1415;
      coord_range.to_value = 3.1415;
      rotation_desc.floating_point_range.push_back(coord_range);
      roll_ = declare_parameter("roll", 0.0, rotation_desc);
      pitch_ = declare_parameter("pitch", 0.0, rotation_desc);
      yaw_ = declare_parameter("yaw", 0.0, rotation_desc);

      //params_.initialize(priv);

      /*params_.get("x", x_, 0.0, "X offset (m)", -10000.0, 10000.0);
      params_.get("y", y_, 0.0, "Y offset (m)", -10000.0, 10000.0);
      params_.get("z", z_, 0.0, "Z offset (m)", -10000.0, 10000.0);
      params_.get("roll", roll_, 0.0, "Roll offset (rad)", -3.1415, 3.1415);
      params_.get("pitch", pitch_, 0.0, "Pitch offset (rad)", -3.1415, 3.1415);
      params_.get("yaw", yaw_, 0.0, "Yaw offset (rad)", -3.1415, 3.1415);

      params_.finalize();*/

      rate_ = std::max(1.0, rate_);
      pub_timer_ = this->create_wall_timer(
          std::chrono::duration<float>(1.0 / rate_), std::bind(&DynamicTransformPublisher::Publish, this));
    }

  private:
    void Publish()
    {
      //params_.mutex().lock();
      tf2::Vector3 origin(x_, y_, z_);
      tf2::Quaternion rotation;
      rotation.setRPY(roll_, pitch_, yaw_);
      //params_.mutex().unlock();

      tf2::Transform transform(rotation, origin);

      geometry_msgs::msg::TransformStamped stamped_transform;
      stamped_transform.transform = tf2::toMsg(transform);
      stamped_transform.header.stamp = rclcpp::Clock().now() + rclcpp::Duration(stamp_offset_);
      stamped_transform.child_frame_id = child_frame_;
      stamped_transform.header.frame_id = parent_frame_;

      tf_broadcaster_.sendTransform(stamped_transform);
    }

    double x_, y_, z_;
    double roll_, pitch_, yaw_;
    //swri::DoubleParam x_, y_, z_;

    // swri::DynamicParameters params_;

    rclcpp::TimerBase::SharedPtr pub_timer_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    double rate_;
    double stamp_offset_;
    std::string child_frame_;
    std::string parent_frame_;
  };

}  // namespace swri_transform_util

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_transform_util::DynamicTransformPublisher)
