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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <swri_opencv_util/blend.h>

namespace swri_image_util
{
  const double DEFAULT_ALPHA_LEVEL = 0.5;
  const cv::Scalar NO_MASK = cv::Scalar(-1, -1, -1);

  // ROS nodelet for blending the images together
  class BlendImagesNodelet : public nodelet::Nodelet
  {
  public:
      BlendImagesNodelet();
      ~BlendImagesNodelet();
  private:
      // Use the message_filters to listen for the base and top image
      // using an approximate time synchronization policy. These typedefs
      // make creating the filter cleaner looking
      typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image> ApproximateTimePolicy;
      typedef message_filters::Synchronizer<ApproximateTimePolicy>
        ApproximateTimeSync;
      // Called when the nodelet starts up. This does all of the initialization
      virtual void onInit();
      // Callback for the two images we are blending together
      void imageCallback(
          const sensor_msgs::ImageConstPtr &base_image,
          const sensor_msgs::ImageConstPtr &top_image);
      // Alpha blending level of the two images
      double alpha_;
      // Color to mask, if necessary
      cv::Scalar mask_color_;
      // Publishes the blended image
      image_transport::Publisher image_pub_;
      // The subscribers for the base and top image
      message_filters::Subscriber<sensor_msgs::Image> base_image_sub_;
      message_filters::Subscriber<sensor_msgs::Image> top_image_sub_;
      // Synchronization object for the two images we need for the blending
      // process
      boost::shared_ptr<ApproximateTimeSync> image_sync_;
  };

  BlendImagesNodelet::BlendImagesNodelet() :
    alpha_(DEFAULT_ALPHA_LEVEL),
    mask_color_(NO_MASK)
  {
  }

  BlendImagesNodelet::~BlendImagesNodelet()
  {
  }

  void BlendImagesNodelet::onInit()
  {
    // Node handles for interacting with the wider ROS system
    ros::NodeHandle& node = getNodeHandle();
    ros::NodeHandle& priv = getPrivateNodeHandle();

    // User setting for the alpha value. The constructor should have
    // already set this to the default value
    priv.param("alpha", alpha_, alpha_);

    // Values should be in the range [0,255]
    double mask_r;
    double mask_g;
    double mask_b;
    priv.param("mask_r", mask_r, -1.0);
    priv.param("mask_g", mask_g, -1.0);
    priv.param("mask_b", mask_b, -1.0);

    // Only create the mask if all components are valid
    if ((mask_r >= 0) && (mask_g >= 0) && (mask_b >= 0) &&
        (mask_r <= 255) && (mask_g <= 255) && (mask_b <= 255))
    {
      mask_color_ = cv::Scalar(mask_r, mask_g, mask_b);
    }
    else
    {
      ROS_ERROR("Mask color components must be in range [0,255]");
      ROS_ERROR("  Components were (%f, %f, %f)", mask_r, mask_g, mask_b);
    }

    // Set up our publisher of the blended data and listen to the two input
    // images
    image_transport::ImageTransport it(node);
    image_pub_ = it.advertise("blended_image", 1);
    base_image_sub_.subscribe(node, "base_image", 1);
    top_image_sub_.subscribe(node, "top_image", 1);
    image_sync_.reset(new ApproximateTimeSync(
                        ApproximateTimePolicy(10),
                        base_image_sub_,
                        top_image_sub_));
    // Start listening for the images
    image_sync_->registerCallback(boost::bind(
                                    &BlendImagesNodelet::imageCallback,
                                    this,
                                    _1,
                                    _2));
  }

  void BlendImagesNodelet::imageCallback(
      const sensor_msgs::ImageConstPtr& base_image,
      const sensor_msgs::ImageConstPtr& top_image)
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

    // Blend the images together
    if (mask_color_ != NO_MASK)
    {
      cv::Mat mask;
      cv::inRange(cv_top_image->image, mask_color_, mask_color_, mask);

      blended = swri_opencv_util::overlayColor(
            cv_base_image->image,
            mask,
            mask_color_,
            alpha_);
    }
    else
    {
      blended = swri_opencv_util::blend(
            cv_top_image->image,
            cv_base_image->image,
            alpha_);
    }

    // Convert the blended image to a ROS type and publish the result
    cv_bridge::CvImagePtr cv_blended = boost::make_shared<cv_bridge::CvImage>();
    cv_blended->image = blended;
    cv_blended->encoding = cv_base_image->encoding;
    cv_blended->header = cv_base_image->header;

    image_pub_.publish(cv_blended->toImageMsg());
  }
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(swri_image_util, BlendImagesNodelet)
