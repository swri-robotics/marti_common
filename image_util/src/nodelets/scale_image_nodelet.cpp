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
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <math_util/math_util.h>

namespace image_util
{
  class ScaleImageNodelet : public nodelet::Nodelet
  {
  public:
      ScaleImageNodelet() :
      scale_(1.0)
    {
    }

    ~ScaleImageNodelet()
    {
    }

    void onInit()
    {
      ros::NodeHandle &node = getNodeHandle();
      ros::NodeHandle &priv = getPrivateNodeHandle();

      priv.param("scale", scale_, scale_);

      image_transport::ImageTransport it(node);
      image_pub_ = it.advertise("scaled_image", 1);
      image_sub_ = it.subscribe("image", 1, &ScaleImageNodelet::ImageCallback, this);
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr& image)
    {
      if (scale_ == 1.0)
      {
        image_pub_.publish(image);
        return;
      }

      cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);

      cv::Size size(
          math_util::Round(image->width * scale_),
          math_util::Round(image->height * scale_));
      cv::Mat scaled;
      cv::resize(cv_image->image, scaled, size);

      cv_bridge::CvImagePtr cv_scaled = boost::make_shared<cv_bridge::CvImage>();
      cv_scaled->image = scaled;
      cv_scaled->encoding = cv_image->encoding;
      cv_scaled->header = cv_image->header;

      image_pub_.publish(cv_scaled->toImageMsg());
    }

  private:
    double scale_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    image_util,
    scale_image,
    image_util::ScaleImageNodelet,
    nodelet::Nodelet)
