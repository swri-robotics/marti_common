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
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#ifdef USE_CVBRIDGE_H_FILES
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <swri_image_util/replace_colors.h>

namespace swri_image_util
{
  // This constant defines how large our lookup and transform tables are.
  // Currently assumes 8 bit mono encoded images, so there are 256 gray colors
  // to potentially replace with a different color
  const int32_t NUM_GRAY_VALUES = 256;
  // The output is an RGB8 image. This constant checks that the user passes
  // in a valid RGB value to replace a gray level with
  const int32_t MAX_RGB_VALUE = 255;

  // ROS node for replacing colors in an image
  class ReplaceColorsNode : public rclcpp::Node
  {
  public:
    explicit ReplaceColorsNode(const rclcpp::NodeOptions& options);
  
  private:
    // Callback for the input image
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    // Initialize the lookup table
    void initLut();
    // Read in user requested colormap
    void readColormap(const std::string& colormap_name, int64_t num_entries);
    // Helper function for getting color mapping from parameter server
    void readUserLut(const std::vector<int64_t>& colors);

    // Lookup table defining color replacement strategy. The row indices 
    // correspond to the grayscale values, and the values in the rows are RGB
    // values to replace the grayscale values with
    cv::Mat color_lut_;
    // Publishes the modified image
    image_transport::Publisher image_pub_;
    // Subscribes to the original image
    image_transport::Subscriber image_sub_;
  #ifndef USE_LEGACY_IMAGE_TRANSPORT_API
    image_transport::ImageTransport image_transport_;
  #endif
    // Mapping from a colormap name to the OpenCV integer representation
    std::map<std::string, int32_t> colormap_names_;
  };

  ReplaceColorsNode::ReplaceColorsNode(const rclcpp::NodeOptions& options) :
    rclcpp::Node("replace_colors", options)
#ifndef USE_LEGACY_IMAGE_TRANSPORT_API
    , image_transport_(image_transport::RequiredInterfaces{*this})
#endif
  {
    // Initialize the colormap name mapping. Every OpenCV colormap should have
    // a string identifiying it. This allows the node to easily take a user
    // parameter and convert it to a representation the algorithm can use.
    // OpenCV 2.x does not have the Parula colormap, so only include this
    // when OpenCV 3.x is available
    colormap_names_["autumn"] = cv::COLORMAP_AUTUMN;
    colormap_names_["bone"] = cv::COLORMAP_BONE;
    colormap_names_["jet"] = cv::COLORMAP_JET;
    colormap_names_["winter"] = cv::COLORMAP_WINTER;
    colormap_names_["rainbow"] = cv::COLORMAP_RAINBOW;
    colormap_names_["ocean"] = cv::COLORMAP_OCEAN;
    colormap_names_["summer"] = cv::COLORMAP_SUMMER;
    colormap_names_["spring"] = cv::COLORMAP_SPRING;
    colormap_names_["cool"] = cv::COLORMAP_COOL;
    colormap_names_["hsv"] = cv::COLORMAP_HSV;
    colormap_names_["pink"] = cv::COLORMAP_PINK;
    colormap_names_["hot"] = cv::COLORMAP_HOT;
    colormap_names_["parula"] = cv::COLORMAP_PARULA;
    
    // Lookup table to replace colors with. By default will just convert the
    // grayscale values to their RGB equivalents. If this node is ever extended
    // to more than grayscale, this will have to be changed
    color_lut_ = cv::Mat::zeros(1, NUM_GRAY_VALUES, CV_8UC3);
    initLut();

    // This node has two different methods of changing grayscale values to
    // color imagery. The first maps the grayscale values to OpenCV colormaps.
    this->declare_parameter<std::string>("colormap", "");
    this->declare_parameter<int64_t>("num_colors", NUM_GRAY_VALUES);
    const std::string colormap = this->get_parameter("colormap").as_string();
    const bool colormap_specified = !colormap.empty();
    if (colormap_specified)
    {
      readColormap(colormap, this->get_parameter("num_colors").as_int());
    }

    // The other option for modifying the grayscale images is to define a
    // flat lookup table of [gray, red, green, blue] entries.
    // This can be used in conjunction with the colormap option to replace
    // values in the OpenCV colormap with user values.
    this->declare_parameter<std::vector<int64_t>>("colors", std::vector<int64_t>{});
    const auto colors = this->get_parameter("colors").as_integer_array();
    if (!colors.empty())
    {
      readUserLut(colors);
    }
    else if (!colormap_specified)
    {
      RCLCPP_ERROR(this->get_logger(),
        "Color transformation was not specified. Images will only be converted to their grayscale equivalents");
    }

    // Set up the ROS interface
#ifdef USE_LEGACY_IMAGE_TRANSPORT_API
    image_pub_ = image_transport::create_publisher(this, "modified_image");
    image_sub_ = image_transport::create_subscription(
      this, "image", std::bind(&ReplaceColorsNode::imageCallback, this, std::placeholders::_1), "raw");
#else
    image_pub_ = image_transport_.advertise("modified_image", 1);
    image_sub_ = image_transport_.subscribe(
      "image",
      1,
      &ReplaceColorsNode::imageCallback,
      this);
#endif
  }

  // Callback for getting the input image to change the colors on
  void ReplaceColorsNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
  {
    // Only do the color conversion if someone is subscribing to the data
    if (image_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    // This node currently only supports changing grayscale images. Display an
    // error message if this is not the case, but limit the error reporting rate
    // to once every minute avoid spamming the user with redundant warnings
    if (image_msg->encoding != sensor_msgs::image_encodings::MONO8)
    {
      RCLCPP_ERROR_ONCE(this->get_logger(),
        "Changing image colors is only supported for MONO8 images");
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
    cv_bridge::CvImagePtr output = std::make_shared<cv_bridge::CvImage>();
    output->image = modified_image;
    output->encoding = sensor_msgs::image_encodings::RGB8;
    output->header = image_msg->header;

    // Publish the modified image to the rest of the system
    image_pub_.publish(output->toImageMsg());
  }

  // Initialize grayscale lookup table
  void ReplaceColorsNode::initLut()
  {
    // Sets every row in the lookup table to a triple <x, x, x>,
    // where x is the grayscale value. This will directly map
    // the gray values to their equivalent RGB representation
    for (int32_t idx = 0; idx < NUM_GRAY_VALUES; idx++)
    {
      color_lut_.at<cv::Vec3b>(0, idx) = cv::Vec3b(idx, idx, idx);
    }
  }

  // Read in the colormap and parameters the parameter server has for this
  // node
  void ReplaceColorsNode::readColormap(
    const std::string& colormap_name,
    int64_t num_entries)
  {
    // Sanity check on the number of classes the user specified
    if (num_entries <= 1)
    {
      RCLCPP_ERROR(this->get_logger(), "Must use at least two colors from the colormap");
      return;
    }

    // Make sure the number of values from the colormap is at most the number
    // of grayscale values in our transformation
    if (num_entries > NUM_GRAY_VALUES)
    {
      RCLCPP_ERROR(this->get_logger(), "Number of colormap entries was greater than %d",
        NUM_GRAY_VALUES);
      return;
    }

    // Get the OpenCV representation of the requested colormap
    const auto iter = colormap_names_.find(colormap_name);
    if (iter == colormap_names_.end())
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown colormap: %s requested", colormap_name.c_str());
      return;
    }

    // Now get the specified number of colors from the requested colormap
    cv::Mat original_colors = color_lut_.clone();
    cv::applyColorMap(original_colors, color_lut_, iter->second);

    // Now modify the original colormap to only have the number
    // of distinct entries specified by the user
    original_colors = color_lut_.clone();

    const int32_t lut_size = color_lut_.cols;

      // Frequently the input image may have some small subset of values,
      // like 0-5. In this case, just mapping to a colormap will make the 
      // resulting image look like one color, because the first 6 colors from
      // the colormap will be used, which for most colormaps are almost the
      // same value. This will more intelligently remap this values, so that
      // the 0, 50, 100, 150, 200, 250 color indices are used from the
      // colormap. This "pushes" the color values apart to make them more
      // visually apparent.
    for (int32_t replace_idx = 0; replace_idx < lut_size; replace_idx++)
    {
      int32_t lookup_idx = replace_idx % num_entries;
      lookup_idx = lookup_idx * lut_size / num_entries;
      color_lut_.at<cv::Vec3b>(0, replace_idx) =
        original_colors.at<cv::Vec3b>(0, lookup_idx);
    }
  }

  // Read the user lookup table from the parameter server
  void ReplaceColorsNode::readUserLut(const std::vector<int64_t>& colors)
  {
    if (colors.size() % 4 != 0)
    {
      RCLCPP_ERROR(this->get_logger(),
        "Colors must contain [gray, red, green, blue] entries");
      return;
    }

    // Make a copy of the current LUT. The copy will be modified, and only
    // if the complete parameter reading works will the real LUT be modified.
    cv::Mat temp_lut = color_lut_.clone();

    for (size_t lut_idx = 0; lut_idx < colors.size(); lut_idx += 4)
    {
      const int64_t gray_index = colors[lut_idx];
      if ((gray_index >= NUM_GRAY_VALUES) || (gray_index < 0))
      {
        RCLCPP_ERROR(this->get_logger(),
          "Grayscale value for LUT entry %zu was %ld, but must be between 0 and %d",
          lut_idx / 4, gray_index, NUM_GRAY_VALUES - 1);
        return;
      }

      for (size_t channel = 1; channel < 4; channel++)
      {
        if (colors[lut_idx + channel] < 0 || colors[lut_idx + channel] > MAX_RGB_VALUE)
        {
          RCLCPP_ERROR(this->get_logger(),
            "RGB value on LUT entry %zu was %ld, and must be between 0 and %d",
            lut_idx / 4, colors[lut_idx + channel], MAX_RGB_VALUE);
          return;
        }
      }

      temp_lut.at<cv::Vec3b>(0, static_cast<int32_t>(gray_index)) = cv::Vec3b(
        static_cast<uint8_t>(colors[lut_idx + 1]),
        static_cast<uint8_t>(colors[lut_idx + 2]),
        static_cast<uint8_t>(colors[lut_idx + 3]));
    }

    color_lut_ = temp_lut;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::ReplaceColorsNode)
