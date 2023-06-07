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
  class RotateImageNode : public rclcpp::Node
  {
  public:
    explicit RotateImageNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("rotate_image", options),
        operations_(0),
        flip_axis_(false)
    {
      this->declare_parameter("angle", 0.0);

      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) -> void
      {
        int32_t angle_90 = static_cast<int32_t>(
            swri_math_util::ToNearest(this->get_parameter("angle").as_double(), 90));
        flip_axis_ = angle_90 > 0;
        operations_ = std::abs(angle_90 / 90);

        if (operations_ == 0)
        {
          image_pub_.publish(image);
          return;
        }

        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);

        for (int32_t i = 0; i < operations_; i++)
        {
          cv::transpose(cv_image->image, cv_image->image);
          cv::flip(cv_image->image, cv_image->image, flip_axis_);
        }

        image_pub_.publish(cv_image->toImageMsg());
      };

      image_pub_ = image_transport::create_publisher(this, "rotated_image");
      image_sub_ = image_transport::create_subscription(this, "image", callback, "raw");
    }

  private:
    int32_t operations_;
    bool flip_axis_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::RotateImageNode)
