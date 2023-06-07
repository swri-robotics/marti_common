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
  class ScaleImageNode : public rclcpp::Node
  {
  public:
    explicit ScaleImageNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("scale_image", options)
    {
      this->declare_parameter("scale", 1.0);

      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) -> void
      {
        double scale = this->get_parameter("scale").as_double();
        if (std::fabs(scale - 1.0) < 0.001)
        {
          image_pub_.publish(image);
          return;
        }

        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);

        cv::Size size(
            swri_math_util::Round(image->width * scale),
            swri_math_util::Round(image->height * scale));
        cv::Mat scaled;
        cv::resize(cv_image->image, scaled, size);

        cv_bridge::CvImagePtr cv_scaled = std::make_shared<cv_bridge::CvImage>();
        cv_scaled->image = scaled;
        cv_scaled->encoding = cv_image->encoding;
        cv_scaled->header = cv_image->header;

        image_pub_.publish(cv_scaled->toImageMsg());
      };

      image_pub_ = image_transport::create_publisher(this, "scaled_image");
      image_sub_ = image_transport::create_subscription(this, "image", callback, "raw");
    }

  private:
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::ScaleImageNode)
