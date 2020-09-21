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
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <swri_image_util/image_normalization.h>

#include <swri_math_util/math_util.h>

namespace swri_image_util
{
  class ContrastStretchNode : public rclcpp::Node
  {
  public:
    explicit ContrastStretchNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("contrast_stretch", options)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.name = "bins";
      this->declare_parameter("bins", 8, desc);

      desc.name = "over_exposure_dilation";
      this->declare_parameter("over_exposure_dilation", 3, desc);

      desc.name = "max_min";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      this->declare_parameter("max_min", 0.0, desc);

      desc.name = "min_max";
      this->declare_parameter("min_max", 0.0, desc);

      desc.name = "over_exposure_threshold";
      this->declare_parameter("over_exposure_threshold", 255.0, desc);

      desc.name = "mask";
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      desc.read_only = true;
      this->declare_parameter("mask", std::string(""), desc);

      std::string mask = this->get_parameter("mask").as_string();
      if (!mask.empty())
      {
        mask_ = cv::imread(mask, 0);
      }

      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) -> void
      {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);

        if (mask_.empty())
        {
          mask_ = cv::Mat::ones(cv_image->image.size(), CV_8U);
        }
        else if (mask_.rows != cv_image->image.rows || mask_.cols != cv_image->image.cols)
        {
          cv::resize(mask_, mask_, cv_image->image.size(), 1.0, 1.0, cv::INTER_NEAREST);
        }

        cv::Mat mask;

        double over_exposure_threshold = this->get_parameter("over_exposure_threshold").as_double();

        if (over_exposure_threshold < 255 && over_exposure_threshold > 0)
        {
          int32_t over_exposure_dilation = this->get_parameter("over_exposure_dilation").as_int();
          cv::Mat over_exposed = cv_image->image > over_exposure_threshold;
          cv::Mat element = cv::getStructuringElement(
              cv::MORPH_ELLIPSE,
              cv::Size(2 * over_exposure_dilation + 1, 2 * over_exposure_dilation + 1),
              cv::Point(over_exposure_dilation, over_exposure_dilation));
          cv::dilate(over_exposed, over_exposed, element);

          mask = mask_.clone();
          mask.setTo(0, over_exposed);
        }
        else
        {
          mask = mask_;
        }

        swri_image_util::ContrastStretch(this->get_parameter("bins").as_int(),
            cv_image->image,
            cv_image->image,
            mask,
            this->get_parameter("max_min").as_double(),
            this->get_parameter("min_max").as_double());

        image_pub_.publish(cv_image->toImageMsg());
      };

      image_pub_ = image_transport::create_publisher(this, "normalized_image");
      image_sub_ = image_transport::create_subscription(this, "image", callback, "raw");
    }

  private:
    cv::Mat mask_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::ContrastStretchNode)
