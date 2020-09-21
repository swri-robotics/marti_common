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

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

namespace swri_image_util
{
  class CrosshairsNode : public rclcpp::Node
  {
  public:
    explicit CrosshairsNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("crosshairs", options)
    {
      image_pub_ = image_transport::create_publisher(this, "crosshairs_image");

      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) -> void
      {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);
        // Get image dimensions
        const int h = cv_image->image.rows;
        const int w = cv_image->image.cols;
        // Define line properties
        const cv::Scalar black(0, 0, 0);
        const int thickness = 3;
        const int line_type = 8;  // 8-connected line
        // Draw vertical line
        cv::line(cv_image->image, cv::Point(0, w / 2), cv::Point(h - 1, w / 2), black, thickness, line_type);
        // Draw horizontal line
        cv::line(cv_image->image, cv::Point(h / 2, 0), cv::Point(h / 2, w - 1), black, thickness, line_type);
        // Publish image
        image_pub_.publish(cv_image->toImageMsg());
      };

      image_sub_ = image_transport::create_subscription(this, "image", callback, "raw");
    }

  private:
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::CrosshairsNode)
