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
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_util/image_normalization.h>

#include <math_util/math_util.h>

namespace image_util
{
  class ContrastStretchNodelet : public nodelet::Nodelet
  {
  public:
    ContrastStretchNodelet() :
      bins_(8)
    {
    }

    ~ContrastStretchNodelet()
    {
    }

    void onInit()
    {
      ros::NodeHandle &node = getNodeHandle();
      ros::NodeHandle &priv = getPrivateNodeHandle();

      priv.param("bins", bins_, bins_);
      
      std::string mask;
      priv.param("mask", mask, std::string(""));
      if (!mask.empty())
      {
        mask_ = cv::imread(mask, 0);
      }

      image_transport::ImageTransport it(node);
      image_pub_ = it.advertise("normalized_image", 1);
      image_sub_ = it.subscribe("image", 1, &ContrastStretchNodelet::ImageCallback, this);
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr& image)
    {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);

      image_util::ContrastStretch(bins_, cv_image->image, cv_image->image, mask_);

      image_pub_.publish(cv_image->toImageMsg());
    }

  private:
    int32_t bins_;
    
    cv::Mat mask_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

  };
}

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    image_util,
    contrast_stretch,
    image_util::ContrastStretchNodelet,
    nodelet::Nodelet)
