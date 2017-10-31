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
#include <swri_image_util/image_normalization.h>

#include <swri_math_util/math_util.h>

namespace swri_image_util
{
  class NormalizeResponseNodelet : public nodelet::Nodelet
  {
  public:
    NormalizeResponseNodelet() :
      filter_size_(9),
      filter_cap_(31)
    {
    }

    ~NormalizeResponseNodelet()
    {
    }

    void onInit()
    {
      ros::NodeHandle &node = getNodeHandle();
      ros::NodeHandle &priv = getPrivateNodeHandle();

      priv.param("filter_size", filter_size_, filter_size_);
      priv.param("filter_cap", filter_cap_, filter_cap_);

      buffer_.create(1, 10000000, CV_8U);

      image_transport::ImageTransport it(node);
      image_pub_ = it.advertise("normalized_image", 1);
      image_sub_ = it.subscribe("image", 1, &NormalizeResponseNodelet::ImageCallback, this);
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr& image)
    {
      cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);

      if (image->encoding == sensor_msgs::image_encodings::MONO8)
      {
        swri_image_util::NormalizeResponse(cv_image->image, normalized_, filter_size_, filter_cap_, buffer_.ptr());
        cv_bridge::CvImage normalized_image;
        normalized_image.header = image->header;
        normalized_image.encoding = image->encoding;
        normalized_image.image = normalized_;
        image_pub_.publish(normalized_image.toImageMsg());
      }
      else
      {
        ROS_WARN("Unsupported image encoding: %s", image->encoding.c_str());
      }
    }

  private:
    int32_t filter_size_;
    int32_t filter_cap_;
    
    cv::Mat normalized_;
    cv::Mat buffer_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(swri_image_util, NormalizeResponseNodelet)
