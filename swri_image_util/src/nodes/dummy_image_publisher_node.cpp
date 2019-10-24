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

namespace swri_image_util
{
  class DummyImagePublisherNode : public rclcpp::Node
  {
  public:
    DummyImagePublisherNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("dummy_image_publisher", options)
    {
      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 100);

      this->get_parameter_or("rate", rate_, 10.0);
      this->get_parameter_or("width", width_, 640);
      this->get_parameter_or("height", height_, 480);
      this->get_parameter_or("encoding", encoding_, std::string("mono8"));

      auto publisher = [this]() -> void
      {
        sensor_msgs::msg::Image::UniquePtr image = std::make_unique<sensor_msgs::msg::Image>();
        image->header.stamp = rclcpp::Clock().now();
        image->encoding = encoding_;
        image->width = width_;
        image->height = height_;
        image->step = width_;
        image->data.resize(height_ * width_);

        image_pub_->publish(std::move(image));
      };

      timer_ = this->create_wall_timer(std::chrono::duration<float>(1.0 / rate_), publisher);
    }

  private:
    double rate_;
    int32_t width_;
    int32_t height_;
    std::string encoding_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::DummyImagePublisherNode)
