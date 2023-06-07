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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#ifdef USE_CVBRIDGE_H_FILES
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <swri_math_util/math_util.h>

namespace swri_image_util
{
  class DrawTextNode : public rclcpp::Node
  {
  public:
    explicit DrawTextNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("draw_text", options)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.name = "text";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      this->declare_parameter("text", "label", desc);

      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.name = "font_thickness";
      this->declare_parameter("font_thickness", 1, desc);

      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      desc.name = "offset_x";
      this->declare_parameter("offset_x", 0.0, desc);
      desc.name = "offset_y";
      this->declare_parameter("offset_y", 0.0, desc);
      desc.name = "font_scale";
      this->declare_parameter("font_scale", 1.0, desc);

      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) -> void
      {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);

        cv::putText(
            cv_image->image,
            this->get_parameter("text").as_string(),
            cv::Point(this->get_parameter("offset_x").as_double(),
                this->get_parameter("offset_y").as_double()),
            cv::FONT_HERSHEY_SIMPLEX,
            this->get_parameter("font_scale").as_double(),
            cv::Scalar(255, 255, 255),
            this->get_parameter("font_thickness").as_int());

        image_pub_.publish(cv_image->toImageMsg());
      };

      image_pub_ = image_transport::create_publisher(this, "stamped_image");
      image_sub_ = image_transport::create_subscription(this, "image", callback, "raw");
    }

  private:

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::DrawTextNode)
