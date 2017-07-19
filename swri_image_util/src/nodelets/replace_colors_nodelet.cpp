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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <swri_image_util/replace_colors.h>

namespace swri_image_util
{
  class ReplaceColorsNodelet : public nodelet::Nodelet
  {
  public:
    ReplaceColorsNodelet();
    ~ReplaceColorsNodelet();
  
  private:
    virtual void onInit();
    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

    cv::Mat color_lut_;
    image_transport::Publisher image_pub_;
  };

  ReplaceColorsNodelet::ReplaceColorsNodelet()
  {
  }

  ReplaceColorsNodelet::~ReplaceColorsNodelet()
  {
  }

  void ReplaceColorsNodelet::onInit()
  {

  }

  void ReplaceColorsNodelet::imageCallback(
    const sensor_msgs::ImageConstPtr& image_msg)
  {

  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
  swri_image_util,
  replace_colors,
  swri_image_util::ReplaceColorsNodelet,
  nodelet::Nodelet)