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
#include <sensor_msgs/image_encodings.h>
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
    bool readLut(const XmlRpc::XmlRpcValue &param);

    cv::Mat color_lut_;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
  };

  ReplaceColorsNodelet::ReplaceColorsNodelet()
  {
  }

  ReplaceColorsNodelet::~ReplaceColorsNodelet()
  {
  }

  void ReplaceColorsNodelet::onInit()
  {
    // Node handles for interacting with ROS
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& priv_nh  = getPrivateNodeHandle();

    color_lut_ = cv::Mat::zeros(1, 256, CV_8UC3);

    XmlRpc::XmlRpcValue color_param;
    if (priv_nh.getParam("colors", color_param))
    {
      readLut(color_param);
    }
    else
    {
      ROS_ERROR("LUT for color transformation must be specified");
      ROS_BREAK();
    }

    image_transport::ImageTransport it(nh);
    image_pub_ = it.advertise("modified_image", 1);
    image_sub_ = it.subscribe(
      "image",
      1,
      &ReplaceColorsNodelet::imageCallback,
      this);
  }

  void ReplaceColorsNodelet::imageCallback(
    const sensor_msgs::ImageConstPtr& image_msg)
  {
    if (image_msg->encoding != sensor_msgs::image_encodings::MONO8)
    {
      ROS_ERROR("Changing image colors is only supported for MONO8 images");
      return;
    }

    // Convert image data from ROS to OpenCV type
    cv_bridge::CvImageConstPtr original_image = cv_bridge::toCvShare(image_msg);

    // Allocate space for the new image
    cv::Mat modified_image = cv::Mat::zeros(
      original_image->image.rows,
      original_image->image.cols,
      CV_8UC3);

    swri_image_util::replaceColors(
      original_image->image,
      color_lut_,
      modified_image);

    cv_bridge::CvImagePtr output = boost::make_shared<cv_bridge::CvImage>();
    output->image = modified_image;
    output->encoding = sensor_msgs::image_encodings::RGB8;
    output->header = image_msg->header;

    image_pub_.publish(output->toImageMsg());
  }

  bool ReplaceColorsNodelet::readLut(const XmlRpc::XmlRpcValue& param)
  {
    if (param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("LUT must be an array");
      return false;
    }

    for (uint32_t lut_idx = 0; lut_idx < param.size(); lut_idx++)
    {
      XmlRpc::XmlRpcValue lut_row = param[lut_idx];
      if ((lut_row.getType() != XmlRpc::XmlRpcValue::TypeArray) ||
        (lut_row.size() != 2))
      {
        ROS_ERROR("LUT entries must be two entry arrays");
        return false;
      }
      XmlRpc::XmlRpcValue map_key = lut_row[0];
      XmlRpc::XmlRpcValue rgb_values = lut_row[1];
      if ((rgb_values.getType() != XmlRpc::XmlRpcValue::TypeArray) || 
        (rgb_values.size() != 3) ||
        (rgb_values[0].getType() != XmlRpc::XmlRpcValue::TypeInt) ||
        (rgb_values[1].getType() != XmlRpc::XmlRpcValue::TypeInt) ||
        (rgb_values[2].getType() != XmlRpc::XmlRpcValue::TypeInt))
      {
        ROS_ERROR("RGB entries must be three entry arrays of integers");
        return false;
      }

      cv::Vec3b rgb_entry(
        static_cast<uint8_t>(static_cast<int32_t>(rgb_values[0])),
        static_cast<uint8_t>(static_cast<int32_t>(rgb_values[1])),
        static_cast<uint8_t>(static_cast<int32_t>(rgb_values[2])));
      color_lut_.at<cv::Vec3b>(0, static_cast<uint8_t>(
        static_cast<int32_t>(map_key))) = rgb_entry;
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
  swri_image_util,
  replace_colors,
  swri_image_util::ReplaceColorsNodelet,
  nodelet::Nodelet)