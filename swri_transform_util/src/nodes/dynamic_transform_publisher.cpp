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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace swri_transform_util
{

  class DynamicTransformPublisher : public rclcpp::Node
  {
  public:
    explicit DynamicTransformPublisher(const rclcpp::NodeOptions& options) :
        rclcpp::Node("dynamic_transform_publisher", options),
        tf_broadcaster_(*this)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "stamp_offset";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      this->declare_parameter("stamp_offset", 1.0, desc);
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      desc.name = "child_frame";
      this->declare_parameter("child_frame", std::string(""), desc);
      desc.name = "parent_frame";
      this->declare_parameter("parent_frame", std::string(""), desc);
      desc.name = "rate";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      desc.read_only = true;
      this->declare_parameter("rate", 50.0, desc);


      rcl_interfaces::msg::ParameterDescriptor coord_desc;
      rcl_interfaces::msg::FloatingPointRange coord_range;
      coord_range.from_value = -10000.0;
      coord_range.to_value = 10000.0;
      coord_desc.floating_point_range.push_back(coord_range);
      coord_desc.name = "x";
      coord_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      declare_parameter("x", 0.0, coord_desc);
      coord_desc.name = "y";
      declare_parameter("y", 0.0, coord_desc);
      coord_desc.name = "z";
      declare_parameter("z", 0.0, coord_desc);

      rcl_interfaces::msg::ParameterDescriptor rotation_desc;
      rcl_interfaces::msg::FloatingPointRange rotation_range;
      rotation_range.from_value = -3.1415;
      rotation_range.to_value = 3.1415;
      rotation_desc.floating_point_range.push_back(rotation_range);
      rotation_desc.name = "roll";
      rotation_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      declare_parameter("roll", 0.0, rotation_desc);
      rotation_desc.name = "pitch";
      declare_parameter("pitch", 0.0, rotation_desc);
      rotation_desc.name = "yaw";
      declare_parameter("yaw", 0.0, rotation_desc);

      pub_timer_ = this->create_wall_timer(
          std::chrono::duration<float>(1.0 / this->get_parameter("rate").as_double()),
              std::bind(&DynamicTransformPublisher::Publish, this));
    }

  private:
    void Publish()
    {
      std::vector<rclcpp::Parameter> params =
          get_parameters(std::vector<std::string>{"x", "y", "z", "roll", "pitch", "yaw"});
      tf2::Vector3 origin(params.at(0).as_double(), params.at(1).as_double(), params.at(2).as_double());
      tf2::Quaternion rotation;
      rotation.setRPY(params.at(3).as_double(), params.at(4).as_double(), params.at(5).as_double());

      tf2::Transform transform(rotation, origin);

      geometry_msgs::msg::TransformStamped stamped_transform;
      stamped_transform.transform = tf2::toMsg(transform);
      stamped_transform.header.stamp = rclcpp::Clock().now() +
          rclcpp::Duration::from_seconds(this->get_parameter("stamp_offset").as_double());
      stamped_transform.child_frame_id = this->get_parameter("child_frame").as_string();
      stamped_transform.header.frame_id = this->get_parameter("parent_frame").as_string();

      tf_broadcaster_.sendTransform(stamped_transform);
    }

    rclcpp::TimerBase::SharedPtr pub_timer_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
  };

}  // namespace swri_transform_util

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_transform_util::DynamicTransformPublisher)
