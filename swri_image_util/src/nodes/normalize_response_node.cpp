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
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <swri_image_util/image_normalization.h>

#include <swri_math_util/math_util.h>

namespace swri_image_util
{
class NormalizeResponseNodelet : public rclcpp::Node
  {
  public:
    explicit NormalizeResponseNodelet(const rclcpp::NodeOptions& options) :
      rclcpp::Node("normalize_response", options)
    {
      this->declare_parameter("filter_size", 9);
      this->declare_parameter("filter_cap", 31);

      buffer_.create(1, 10000000, CV_8U);

      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) -> void {
        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);

        if (image->encoding == sensor_msgs::image_encodings::MONO8)
        {
          swri_image_util::NormalizeResponse(
              cv_image->image,
              normalized_,
              this->get_parameter("filter_size").as_int(),
              this->get_parameter("filter_cap").as_int(),
              buffer_.ptr());
          cv_bridge::CvImage normalized_image;
          normalized_image.header = image->header;
          normalized_image.encoding = image->encoding;
          normalized_image.image = normalized_;
          image_pub_.publish(normalized_image.toImageMsg());
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Unsupported image encoding: %s", image->encoding.c_str());
        }
      };

      image_pub_ = image_transport::create_publisher(this, "normalized_image");
      image_sub_ = image_transport::create_subscription(this, "image", callback, "raw");
    }

  private:
    cv::Mat normalized_;
    cv::Mat buffer_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::NormalizeResponseNodelet)
