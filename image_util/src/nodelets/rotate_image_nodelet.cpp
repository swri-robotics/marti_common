// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <string>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <math_util/math_util.h>

namespace image_util
{
  class RotateImageNodelet : public nodelet::Nodelet
  {
  public:
    RotateImageNodelet() :
      angle_(0),
      operations_(0),
      flip_axis_(false)
    {
    }

    ~RotateImageNodelet()
    {
    }

    void onInit()
    {
      ros::NodeHandle &node = getNodeHandle();
      ros::NodeHandle &priv = getPrivateNodeHandle();

      priv.param("angle", angle_, angle_);

      int32_t angle_90 = static_cast<int32_t>(math_util::ToNearest(angle_, 90));
      flip_axis_ = angle_90 > 0 ? 1 : 0;
      operations_ = std::abs(angle_90 / 90);

      image_transport::ImageTransport it(node);
      image_sub_ = it.subscribe("image", 1, &RotateImageNodelet::ImageCallback, this);
      image_pub_ = it.advertise("rotated_image", 1);
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr& image)
    {
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
    }

  private:
    double angle_;
    int32_t operations_;
    bool flip_axis_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  };
}

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    image_util,
    rotate_image,
    image_util::RotateImageNodelet,
    nodelet::Nodelet)
