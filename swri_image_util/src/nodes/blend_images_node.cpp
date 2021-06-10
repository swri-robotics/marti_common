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

#include <opencv2/core/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <swri_opencv_util/blend.h>

namespace swri_image_util
{
  const double DEFAULT_ALPHA_LEVEL = 0.5;
  const cv::Scalar NO_MASK = cv::Scalar(-1, -1, -1);

  // ROS nodelet for blending the images together
  class BlendImagesNode : public rclcpp::Node
  {
  public:
    explicit BlendImagesNode(const rclcpp::NodeOptions& options);

  private:
    // Use the message_filters to listen for the base and top image
    // using an approximate time synchronization policy. These typedefs
    // make creating the filter cleaner looking
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> ApproximateTimePolicy;
    typedef message_filters::Synchronizer<ApproximateTimePolicy>
        ApproximateTimeSync;

    // Callback for the two images we are blending together
    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& base_image,
        const sensor_msgs::msg::Image::ConstSharedPtr& top_image);

    // Publishes the blended image
    image_transport::Publisher image_pub_;
    // The subscribers for the base and top image
    message_filters::Subscriber<sensor_msgs::msg::Image> base_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> top_image_sub_;
    // Synchronization object for the two images we need for the blending
    // process
    std::shared_ptr<ApproximateTimeSync> image_sync_;
  };



  BlendImagesNode::BlendImagesNode(const rclcpp::NodeOptions& options) :
      Node("blend_images", options)
  {
    rcl_interfaces::msg::ParameterDescriptor alphaDesc;
    alphaDesc.name = "alpha";
    alphaDesc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    alphaDesc.floating_point_range.resize(1);
    auto& alphaRange = alphaDesc.floating_point_range.at(0);
    alphaRange.from_value = 0.0;
    alphaRange.to_value = 1.0;
    alphaRange.step = 0.01;
    alphaDesc.description = "Alpha value for blending images; 1.0 is full base image, 0.0 is full top image.";
    this->declare_parameter("alpha", DEFAULT_ALPHA_LEVEL, alphaDesc);

    // Values should be in the range [0,255]
    rcl_interfaces::msg::ParameterDescriptor rgbDesc;
    rgbDesc.name = "mask_r";
    rgbDesc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    rgbDesc.description = "Red value of color to mask; 0.0 to 255.0, or -1.0 to to mask a color.";
    rgbDesc.floating_point_range.resize(1);
    auto& rgbRange = rgbDesc.floating_point_range.at(0);
    rgbRange.from_value = -1.0;
    rgbRange.to_value = 255.0;
    rgbRange.step = 1.0;

    this->declare_parameter("mask_r", -1.0, rgbDesc);
    rgbDesc.name = "mask_g";
    rgbDesc.description = "Green value of color to mask; 0.0 to 255.0, or -1.0 to to mask a color.";
    this->declare_parameter("mask_g", -1.0, rgbDesc);
    rgbDesc.name = "mask_b";
    rgbDesc.description = "Blue value of color to mask; 0.0 to 255.0, or -1.0 to to mask a color.";
    this->declare_parameter("mask_b", -1.0, rgbDesc);

    image_pub_ = image_transport::create_publisher(this, "blended_image");
    base_image_sub_.subscribe(this, "base_image");
    top_image_sub_.subscribe(this, "top_image");
    image_sync_.reset(new ApproximateTimeSync(
        ApproximateTimePolicy(10),
        base_image_sub_,
        top_image_sub_));
    // Start listening for the images
    image_sync_->registerCallback(std::bind(
        &BlendImagesNode::imageCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

  void BlendImagesNode::imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& base_image,
      const sensor_msgs::msg::Image::ConstSharedPtr& top_image)
  {
    // Convert the ROS image types to OpenCV types. Use toCvShare() here because
    // the base and top image will not be modified, and so we do not need our
    // own copy of the image
    cv_bridge::CvImageConstPtr cv_base_image = cv_bridge::toCvShare(base_image);
    // Use the base image encoding during the conversion so different types of
    // images can be blended together
    cv_bridge::CvImageConstPtr cv_top_image = cv_bridge::toCvShare(
        top_image,
        base_image->encoding);

    // Initialize the output to the same size and type as the base image
    cv::Mat blended = cv::Mat::zeros(
        cv_base_image->image.rows,
        cv_base_image->image.cols,
        cv_base_image->image.type());

    std::vector<rclcpp::Parameter> mask_colors =
        this->get_parameters(std::vector<std::string>{"mask_r", "mask_g", "mask_b"});

    cv::Scalar mask_color;
    if (mask_colors[0].as_double() >= 0.0 &&
        mask_colors[1].as_double() >= 0.0 &&
        mask_colors[2].as_double() >= 0.0)
    {
      mask_color = cv::Scalar(mask_colors[0].as_double(),
          mask_colors[1].as_double(),
          mask_colors[2].as_double());
    }
    else
    {
      mask_color = NO_MASK;
    }

    // Blend the images together
    if (mask_color != NO_MASK)
    {
      cv::Mat mask;
      cv::inRange(cv_top_image->image, mask_color, mask_color, mask);

      blended = swri_opencv_util::overlayColor(
          cv_base_image->image,
          mask,
          mask_color,
          this->get_parameter("alpha").as_double());
    }
    else
    {
      blended = swri_opencv_util::blend(
          cv_top_image->image,
          cv_base_image->image,
          this->get_parameter("alpha").as_double());
    }

    // Convert the blended image to a ROS type and publish the result
    cv_bridge::CvImagePtr cv_blended = std::make_shared<cv_bridge::CvImage>();
    cv_blended->image = blended;
    cv_blended->encoding = cv_base_image->encoding;
    cv_blended->header = cv_base_image->header;

    image_pub_.publish(cv_blended->toImageMsg());
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::BlendImagesNode)
