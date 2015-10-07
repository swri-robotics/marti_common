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
      bins_(8),
      max_min_(0.0),
      min_max_(0.0)
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
      priv.param("max_min", max_min_, max_min_);
      priv.param("min_max", min_max_, min_max_);
      
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

      image_util::ContrastStretch(bins_, cv_image->image, cv_image->image, mask_, max_min_, min_max_);

      image_pub_.publish(cv_image->toImageMsg());
    }

  private:
    int32_t bins_;
    double max_min_;
    double min_max_;
    
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
