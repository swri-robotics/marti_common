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
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <swri_math_util/math_util.h>

namespace swri_image_util
{
class DrawTextNodelet : public rclcpp::Node
  {
  public:
      explicit DrawTextNodelet(const rclcpp::NodeOptions& options) :
        rclcpp::Node("draw_text", options),
        text_("label"),
        offset_x_(0),
        offset_y_(0),
        font_scale_(1.0),
        font_thickness_(1)
    {
      this->get_parameter_or("text", text_, text_);
      this->get_parameter_or("offset_x", offset_x_, offset_x_);
      this->get_parameter_or("offset_y", offset_y_, offset_y_);
      this->get_parameter_or("font_scale", font_scale_, font_scale_);
      this->get_parameter_or("font_thickness", font_thickness_, font_thickness_);

      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) -> void {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);

        cv::putText(
            cv_image->image,
            text_,
            cv::Point(offset_x_, offset_y_),
            cv::FONT_HERSHEY_SIMPLEX,
            font_scale_,
            cv::Scalar(255, 255, 255),
            font_thickness_);

        image_pub_.publish(cv_image->toImageMsg());
      };

      image_transport::ImageTransport it(shared_from_this());
      image_pub_ = it.advertise("stamped_image", 1);
      image_sub_ = it.subscribe("image", 1, callback);
    }

  private:
    std::string text_;
    double offset_x_;
    double offset_y_;
    double font_scale_;
    int font_thickness_;
    
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::DrawTextNodelet)
