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

#include <string>

// ROS Libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

namespace swri_image_util
{
  class DummyImagePublisherNode : public rclcpp::Node
  {
  public:
    DummyImagePublisherNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("dummy_image_publisher", options)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      desc.name = "encoding";
      this->declare_parameter("encoding", "mono8", desc);

      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.name = "width";
      this->declare_parameter("width", 640, desc);
      desc.name = "height";
      this->declare_parameter("height", 480, desc);

      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      desc.read_only = true;
      this->declare_parameter("rate", 10.0, desc);

      auto publisher = [this]() -> void
      {
        int64_t width = this->get_parameter("width").as_int();
        int64_t height = this->get_parameter("height").as_int();

        sensor_msgs::msg::Image::UniquePtr image = std::make_unique<sensor_msgs::msg::Image>();
        image->header.stamp = rclcpp::Clock().now();
        image->encoding = this->get_parameter("encoding").as_string();
        image->width = width;
        image->height = height;
        image->step = width;
        image->data.resize(height * width);

        image_pub_.publish(std::move(image));
      };

      rmw_qos_profile_t qos;
      qos.depth = 100;
      image_pub_ = image_transport::create_publisher(this, "image", qos);

      timer_ = this->create_wall_timer(std::chrono::duration<float>(1.0 / this->get_parameter("rate").as_double()), publisher);
    }

  private:
    image_transport::Publisher image_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::DummyImagePublisherNode)
