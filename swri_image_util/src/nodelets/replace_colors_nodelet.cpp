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
  const int32_t MAX_GRAY_VALUE = 255;
  const int32_t MAX_RGB_VALUE = 255;

  // ROS nodelet for replacing colors in an image
  class ReplaceColorsNodelet : public nodelet::Nodelet
  {
  public:
    ReplaceColorsNodelet();
    ~ReplaceColorsNodelet();
  
  private:
    // Does the actual initialization since this is a nodelet
    virtual void onInit();
    // Callback for the input image
    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);
    // Initialize the lookup table
    void initLut();
    // Helper function for getting color mapping frC++ static cast to unsigned charom parameter server
    bool readLut(const XmlRpc::XmlRpcValue &param);

    // Lookup table defining color replacement strategy. The row indices 
    // correspond to the gray scale values, and the values in the rows are RGB
    // values to replace the gray scale values with
    cv::Mat color_lut_;
    // Publishes the modified image
    image_transport::Publisher image_pub_;
    // Subscribes to the original image
    image_transport::Subscriber image_sub_;
  };

  ReplaceColorsNodelet::ReplaceColorsNodelet()
  {
    // All initialization is deferred to the onInit() call
  }

  ReplaceColorsNodelet::~ReplaceColorsNodelet()
  {
    // Nothing to clean up on destruction
  }

  void ReplaceColorsNodelet::onInit()
  {
    // Node handles for interacting with ROS
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& priv_nh  = getPrivateNodeHandle();

    // Lookup table to replace colors with. By default will just convert the
    // gray scale values to their RGB equivalents. If this node is ever extended
    // to more than gray scale, this will have to be changed
    color_lut_ = cv::Mat::zeros(1, 256, CV_8UC3);
    initLut();

    // Get the array representing the color mapping from the parameter server.
    // This will be in the format [[u0, [r0, g0, b0]], [u1, [r1, g1, b1]]]
    // where u is the gray scale value to replace, and r, g, b are the RGB
    // components to replace it with.
    XmlRpc::XmlRpcValue color_param;
    if (priv_nh.getParam("colors", color_param))
    {
      // Try to read in the lookup values from the parameter server. Reset the
      // values if this operation fails
      if (!readLut(color_param))
      {
        initLut();
      }
    }
    else
    {
      ROS_ERROR("LUT for color transformation was not specified. Images will ");
      ROS_ERROR("only be converted to their gray scale equivalents");
    }

    // Set up the ROS interface
    image_transport::ImageTransport it(nh);
    image_pub_ = it.advertise("modified_image", 1);
    image_sub_ = it.subscribe(
      "image",
      1,
      &ReplaceColorsNodelet::imageCallback,
      this);
  }

  // Callback for getting the input image to change the colors on
  void ReplaceColorsNodelet::imageCallback(
    const sensor_msgs::ImageConstPtr& image_msg)
  {
    // Only do the color conversion if someone is subscribing to the data
    if (image_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    // This node currently only support changing gray scale images
    if (image_msg->encoding != sensor_msgs::image_encodings::MONO8)
    {
      ROS_ERROR("Changing image colors is only supported for MONO8 images");
      return;
    }

    // Convert image data from ROS to OpenCV type
    cv_bridge::CvImageConstPtr original_image = cv_bridge::toCvShare(image_msg);

    // Allocate space for the modified image
    cv::Mat modified_image = cv::Mat::zeros(
      original_image->image.rows,
      original_image->image.cols,
      CV_8UC3);

    // Do the actual color replacement
    swri_image_util::replaceColors(
      original_image->image,
      color_lut_,
      modified_image);

    // Copy results to output message and set up the header and encoding values
    cv_bridge::CvImagePtr output = boost::make_shared<cv_bridge::CvImage>();
    output->image = modified_image;
    output->encoding = sensor_msgs::image_encodings::RGB8;
    output->header = image_msg->header;

    // Publish the modified image to the rest of the system
    image_pub_.publish(output->toImageMsg());
  }

  void ReplaceColorsNodelet::initLut()
  {
    for (uint32_t idx; idx < 256; idx++)
    {
      color_lut_.at<cv::Vec3b>(0, idx) = cv::Vec3b(idx, idx, idx);
    }
  }

  bool ReplaceColorsNodelet::readLut(const XmlRpc::XmlRpcValue& param)
  {
    // Assume the parameter parsing works by default
    bool success = true;
    if (param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("LUT must be an array");
      success = false;
    }

    // Loop over all values that will be replaced. The casting in here is 
    // complicated because XmlRpc cannot go directly from the parameters to 
    // uint8_t values, so they must first be cast to int32_t, and then to the
    // smaller value
    for (uint32_t lut_idx = 0; success && (lut_idx < param.size()); lut_idx++)
    {
      // Each row will be of the form [key, [r, g, b]]
      XmlRpc::XmlRpcValue lut_row = param[lut_idx];
      if ((lut_row.getType() != XmlRpc::XmlRpcValue::TypeArray) ||
        (lut_row.size() != 2))
      {
        ROS_ERROR("LUT entries must be two entry arrays");
        success = false;
        break;
      }

      // The first element in the gray scale value to replace. Make sure it
      // is of the appropriate type
      XmlRpc::XmlRpcValue map_key = lut_row[0];
      if (map_key.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        ROS_ERROR("Color to replace must be an integer");
        success = false;
        break;
      }

      // Make sure the gray scale index is in the proper range
      int32_t gray_index = static_cast<int32_t>(map_key);
      if (gray_index > MAX_GRAY_VALUE)
      {
        ROS_ERROR("Gray scale index must be less than %d", MAX_GRAY_VALUE);
        success = false;
        break;
      }

      // Convert to the value used to index into the LUT
      uint8_t color_index = static_cast<uint8_t>(gray_index);

      // Now read the RGB values. There must be three of them, they must be
      // integers, and less than 256
      XmlRpc::XmlRpcValue rgb_values = lut_row[1];
      if ((rgb_values.getType() != XmlRpc::XmlRpcValue::TypeArray) || 
        (rgb_values.size() != 3) ||
        (rgb_values[0].getType() != XmlRpc::XmlRpcValue::TypeInt) ||
        (rgb_values[1].getType() != XmlRpc::XmlRpcValue::TypeInt) ||
        (rgb_values[2].getType() != XmlRpc::XmlRpcValue::TypeInt))
      {
        ROS_ERROR("RGB entries must be three entry arrays of integers");
        success = false;
        break;
      }

      // Make sure the RGB values are less than 256
      int32_t original_red = static_cast<int32_t>(rgb_values[0]);
      if (original_red > MAX_RGB_VALUE)
      {
        ROS_ERROR("Red values must be less than %d", MAX_RGB_VALUE);
        success = false;
        break;
      }
      int32_t original_green = static_cast<int32_t>(rgb_values[1]);
      if (original_green > MAX_RGB_VALUE)
      {
        ROS_ERROR("Green values must be less than %d", MAX_RGB_VALUE);
        success = false;
        break;
      }
      int32_t original_blue = static_cast<int32_t>(rgb_values[2]);
      if (original_blue > MAX_RGB_VALUE)
      {
        ROS_ERROR("Blue values must be less than %d", MAX_RGB_VALUE);
        success = false;
        break;
      }

      // Get the RGB values to the correct type
      uint8_t red = static_cast<uint8_t>(original_red);
      uint8_t green = static_cast<uint8_t>(original_green);
      uint8_t blue = static_cast<uint8_t>(original_blue);

      // Convert the RGB values to OpenCV types and put them in the LUT
      cv::Vec3b rgb_entry(red, green, blue);
      color_lut_.at<cv::Vec3b>(0, color_index) = rgb_entry;
    }
    
    return success;
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
  swri_image_util,
  replace_colors,
  swri_image_util::ReplaceColorsNodelet,
  nodelet::Nodelet)