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
#include <opencv2/core/version.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
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

  // ROS nodelet for replacing colors in an image
class ReplaceColorsNodelet : public rclcpp::Node
  {
  public:
    explicit ReplaceColorsNodelet(const rclcpp::NodeOptions& options);
  
  private:
    // Callback for the input image
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg);
    // Initialize the lookup table
    void initLut();
    // Read in user requested colormap
    void readColormap(const XmlRpc::XmlRpcValue& param);
    // Helper function for getting color mapping from parameter server
    void readUserLut(const XmlRpc::XmlRpcValue& param);
    // Helper function for displaying the type of parameter that was read
    std::string getValueTypeString(const XmlRpc::XmlRpcValue& value);

    // Lookup table defining color replacement strategy. The row indices 
    // correspond to the gray scale values, and the values in the rows are RGB
    // values to replace the gray scale values with
    cv::Mat color_lut_;
    // Publishes the modified image
    image_transport::Publisher image_pub_;
    // Subscribes to the original image
    image_transport::Subscriber image_sub_;
    // Mapping from a colormap name to the OpenCV integer representation
    std::map<std::string, int32_t> colormap_names_;
  };

  ReplaceColorsNodelet::ReplaceColorsNodelet(const rclcpp::NodeOptions& options) :
    rclcpp::Node("replace_colors", options)
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
    // gray scale values to their RGB equivalents. If this node is ever extended
    // to more than gray scale, this will have to be changed
    color_lut_ = cv::Mat::zeros(1, NUM_GRAY_VALUES, CV_8UC3);
    initLut();

    // Get the array representing the color mapping from the parameter server.
    // This will be in the format [[u0, [r0, g0, b0]], [u1, [r1, g1, b1]]]
    // where u is the gray scale value to replace, and r, g, b are the RGB
    // components to replace it with.
    XmlRpc::XmlRpcValue color_param;

    // This node has two different methods of changing gray scale values to
    // color imagery. The first maps the gray scale values to OpenCV colormaps
    // This call checks for that option
    bool colormap_specified = this->get_parameter("colormap", color_param);
    if (colormap_specified)
    {
      //  Use the colormap the user has requested
      readColormap(color_param);
    }

    // The other option for modifying the grayscale images is to define a
    // lookup table that maps grayscale levels to user defined RGB values.
    // This can be used in conjunction with the colormap option to replace
    // values in the OpenCV colormap with user values. This call checks
    // if the user is attempting this operation, and reads the lookup table
    // from the parameter server if necessary 
    if (this->get_parameter("colors", color_param))
    {
      // Try to read in the lookup values from the parameter server.
      readUserLut(color_param);
    }
    else if (!colormap_specified)
    {
      RCLCPP_ERROR(this->get_logger(), "Color transformation was not specified. Images will ");
      RCLCPP_ERROR(this->get_logger(), "only be converted to their gray scale equivalents");
    }

    // Set up the ROS interface
    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = it.advertise("modified_image", 1);
    image_sub_ = it.subscribe(
      "image",
      1,
      &ReplaceColorsNodelet::imageCallback,
      this);
  }

  // Callback for getting the input image to change the colors on
  void ReplaceColorsNodelet::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
  {
    // Only do the color conversion if someone is subscribing to the data
    if (image_pub_.getNumSubscribers() == 0)
    {
      return;
    }

    // This node currently only supports changing gray scale images. Display an
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
  void ReplaceColorsNodelet::initLut()
  {
    // Sets every row in the lookup table to a triple <x, x, x>,
    // where x is the grayscale value. This will directly map
    // the gray values to their equivalent RGB representation
    for (uint32_t idx; idx < NUM_GRAY_VALUES; idx++)
    {
      color_lut_.at<cv::Vec3b>(0, idx) = cv::Vec3b(idx, idx, idx);
    }
  }

  // Read in the colormap and parameters the parameter server has for this
  // node
  void ReplaceColorsNodelet::readColormap(const XmlRpc::XmlRpcValue& param)
  {
    // This is a multistep process, and if any step goes wrong then the rest
    // of the process should be aborted.
    bool success = true;

    // The colormap parameters should be formatted like:
    // ["colormap_name", num_colors]
    // This checks to make sure the parameter is a two element array
    if ((param.getType() != XmlRpc::XmlRpcValue::TypeArray) ||
      (param.size() != 2))
    {
      RCLCPP_ERROR(this->get_logger(), "Colormap specification must be a two entry array with the");
      RCLCPP_ERROR(this->get_logger(), "Following format:");
      RCLCPP_ERROR(this->get_logger(), "\t[\"colormap_name\", num_colors]");
      success = false;
    }

    // These are the parameters to read from the parameter server
    std::string colormap_name;
    int32_t num_entries;
    int32_t colormap_idx;

    if (success)
    {
      // Get the XmpRpc representation of the colormap name the number of
      // colors to get from that colormap
      XmlRpc::XmlRpcValue colormap_param = param[0];
      XmlRpc::XmlRpcValue num_entries_param = param[1];

      // Make sure the colormap name is a string
      if (colormap_param.getType() != XmlRpc::XmlRpcValue::TypeString)
      {
        RCLCPP_ERROR(this->get_logger(), "First colormap parameter must be a string");
        success = false;
      }

      // Make sure the number of colors to get from the colormap is
      // an integer
      if (num_entries_param.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        RCLCPP_ERROR(this->get_logger(), "Second colormap parameter must be an integer");
        success = false;
      }

      // Parameters had the correct types. Now XmlRpc datatypes to C++ types
      if (success)
      {
        colormap_name = static_cast<std::string>(colormap_param);
        num_entries = static_cast<int32_t>(num_entries_param);
      }
    }

    // Sanity check on the number of classes the user specified
    if (success && (num_entries <= 1))
    {
      RCLCPP_ERROR(this->get_logger(), "Must use at least two colors from the colormap");
      success = false;
    }

    // Make sure the number of values from the colormap is at most the number
    // of grayscale values in our transformation
    if (success)
    {
      if (num_entries > NUM_GRAY_VALUES)
      {
        RCLCPP_ERROR(this->get_logger(), "Number of colormap entries was greater");
        RCLCPP_ERROR(this->get_logger(), " than %d", NUM_GRAY_VALUES);
        success = false;
      }
    }

    // Get the OpenCV representation of the requested colormap
    if (success)
    {
      std::map<std::string, int32_t>::iterator iter = 
        colormap_names_.find(colormap_name);
      if (iter != colormap_names_.end())
      {
        colormap_idx = colormap_names_[colormap_name]; 
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown colormap: %s requested", colormap_name.c_str());
        success = false;
      }
    }

    // Now get the specified number of colors from the requested colormap
    if (success)
    {
      // Make a copy of the grayscale LUT as a working variable
      cv::Mat original_colors = color_lut_.clone();
      // color_lut_ will have the transformation from the grayscale values
      // to the RGB values after this call for every grayscale value
      cv::applyColorMap(original_colors, color_lut_, colormap_idx);

      // Now modify the original colormap to only have the number
      // of distinct entries specified by the user
      original_colors = color_lut_.clone();

      int32_t lut_size = color_lut_.cols;

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
        color_lut_.at<cv::Vec3b>(0, static_cast<uint8_t>(replace_idx)) = 
          original_colors.at<cv::Vec3b>(0, static_cast<uint8_t>(lookup_idx));
      }
    }
  }

  // Read the user lookup table from the parameter server
  void ReplaceColorsNodelet::readUserLut(const XmlRpc::XmlRpcValue& param)
  {
    // Assume the parameter parsing works by default
    bool success = true;
    if (param.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      RCLCPP_ERROR(this->get_logger(), "LUT must be an array, but type was: %s",
        getValueTypeString(param).c_str());
      success = false;
    }

    // Make a copy of the current LUT. The copy will be modified, and only
    // if the complete parameter reading works will the real LUT be modified.
    cv::Mat temp_lut = color_lut_.clone();

    // Loop over all values that will be replaced. The casting in here is 
    // complicated because XmlRpc cannot go directly from the parameters to 
    // uint8_t values, so they must first be cast to int32_t, and then to the
    // smaller value
    for (uint32_t lut_idx = 0; success && (lut_idx < param.size()); lut_idx++)
    {
      // Each row will be of the form [key, [r, g, b]]. The next two checks make
      // sure the right type and number of entries was passed in
      XmlRpc::XmlRpcValue lut_row = param[lut_idx];
      if (lut_row.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        RCLCPP_ERROR(this->get_logger(), "LUT entries must be two entry arrays with the following");
        RCLCPP_ERROR(this->get_logger(), "format:");
        RCLCPP_ERROR(this->get_logger(), "\t[gray_value, [r, g, b]]");
        RCLCPP_ERROR(this->get_logger(), "but for entry %d a %s was given", 
          lut_idx, getValueTypeString(lut_row).c_str());
        success = false;
        break;
      }

      if (lut_row.size() != 2)
      {
        RCLCPP_ERROR(this->get_logger(), "LUT entries must be two entry arrays with the following");
        RCLCPP_ERROR(this->get_logger(), "format:");
        RCLCPP_ERROR(this->get_logger(), "\t[gray_value, [r, g, b]]");
        RCLCPP_ERROR(this->get_logger(), "but entry %d had an array with only %d values", 
          lut_idx, lut_row.size());
        success = false;
        break;
      }

      // The first element in the gray scale value to replace. Make sure it
      // is of the appropriate type
      XmlRpc::XmlRpcValue map_key = lut_row[0];
      if (map_key.getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        RCLCPP_ERROR(this->get_logger(), "Color to replace must be an integer, but a %s was given for",
          getValueTypeString(map_key).c_str());
        RCLCPP_ERROR(this->get_logger(), "entry %d", lut_idx);
        success = false;
        break;
      }

      // Make sure the gray scale index is in the proper range
      int32_t gray_index = static_cast<int32_t>(map_key);
      if ((gray_index >= NUM_GRAY_VALUES) || (gray_index < 0))
      {
        RCLCPP_ERROR(this->get_logger(), "Gray scale value for LUT entry %d was %d, but must be", 
          lut_idx, gray_index);
        RCLCPP_ERROR(this->get_logger(), "between 0 and %d", NUM_GRAY_VALUES - 1);
        success = false;
        break;
      }

      // Convert to the value used to index into the LUT
      uint8_t color_index = static_cast<uint8_t>(gray_index);

      // Now read the RGB values. There must be three of them, they must be
      // integers, and less than MAX_RGB_VALUE 
      XmlRpc::XmlRpcValue rgb_values = lut_row[1];
      if (rgb_values.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        RCLCPP_ERROR(this->get_logger(), "RGB entries must be three entry arrays of integers, but");
        RCLCPP_ERROR(this->get_logger(), "LUT row %d was a %s", lut_idx,
          getValueTypeString(rgb_values).c_str());
        success = false;
        break;
      }

      if (rgb_values.size() != 3)
      {
        RCLCPP_ERROR(this->get_logger(), "RGB entries must be three entry arrays of integers, but");
        RCLCPP_ERROR(this->get_logger(), "LUT row %d had %d entries", lut_idx, rgb_values.size());
        success = false;
        break;
      }

      if (rgb_values[0].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        RCLCPP_ERROR(this->get_logger(), "RGB entries must be three entry arrays of integers, but");
        RCLCPP_ERROR(this->get_logger(), "LUT row %d had a red value with type %s", lut_idx,
          getValueTypeString(rgb_values[0]).c_str());
        success = false;
        break;
      }

      if (rgb_values[1].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        RCLCPP_ERROR(this->get_logger(), "RGB entries must be three entry arrays of integers, but");
        RCLCPP_ERROR(this->get_logger(), "LUT row %d had a green value with type %s", lut_idx,
          getValueTypeString(rgb_values[1]).c_str());
        success = false;
        break;
      }

      if (rgb_values[2].getType() != XmlRpc::XmlRpcValue::TypeInt)
      {
        RCLCPP_ERROR(this->get_logger(), "RGB entries must be three entry arrays of integers, but");
        RCLCPP_ERROR(this->get_logger(), "LUT row %d had a blue value with type %s", lut_idx,
          getValueTypeString(rgb_values[2]).c_str());
        success = false;
        break;
      }

      // Make sure the RGB values are between 0 and MAX_RGB_VALUE 
      int32_t original_red = static_cast<int32_t>(rgb_values[0]);
      if ((original_red > MAX_RGB_VALUE) || (original_red < 0))
      {
        RCLCPP_ERROR(this->get_logger(), "Red value on row %d was %d, and must be between 0 and %d",
          lut_idx, original_red, MAX_RGB_VALUE);
        success = false;
        break;
      }
      int32_t original_green = static_cast<int32_t>(rgb_values[1]);
      if ((original_green > MAX_RGB_VALUE) || (original_green < 0))
      {
        RCLCPP_ERROR(this->get_logger(), "Green value on row %d was %d, and must be between 0 and %d",
          lut_idx, original_green, MAX_RGB_VALUE);
        success = false;
        break;
      }
      int32_t original_blue = static_cast<int32_t>(rgb_values[2]);
      if ((original_blue > MAX_RGB_VALUE) || (original_blue < 0))
      {
        RCLCPP_ERROR(this->get_logger(), "Blue value on row %d was %d, and must be between 0 and %d",
          lut_idx, original_blue, MAX_RGB_VALUE);
        success = false;
        break;
      }

      // make the RGB values the correct type
      uint8_t red = static_cast<uint8_t>(original_red);
      uint8_t green = static_cast<uint8_t>(original_green);
      uint8_t blue = static_cast<uint8_t>(original_blue);

      // Convert the RGB values to OpenCV types and put them in the LUT
      cv::Vec3b rgb_entry(red, green, blue);
      temp_lut.at<cv::Vec3b>(0, color_index) = rgb_entry;
    }

    // If the parametre were successfully read the modified LUT will be
    // copied back to the persistent LUT.
    if (success)
    {
      color_lut_ = temp_lut.clone();
    }
  }

  std::string ReplaceColorsNodelet::getValueTypeString(
    const XmlRpc::XmlRpcValue& value)
  {
    std::string type_str = "unknown";
    switch(value.getType())
    {
      case XmlRpc::XmlRpcValue::TypeInvalid:
        type_str = "invalid";
        break;

      case XmlRpc::XmlRpcValue::TypeBoolean:
        type_str = "boolean";
        break;
      
      case XmlRpc::XmlRpcValue::TypeInt:
        type_str = "int";
        break;

      case XmlRpc::XmlRpcValue::TypeDouble:
        type_str = "double";
        break;

      case XmlRpc::XmlRpcValue::TypeString:
        type_str = "string";
        break;
        
      case XmlRpc::XmlRpcValue::TypeDateTime:
        type_str = "date time";
        break;
        
      case XmlRpc::XmlRpcValue::TypeBase64:
        type_str = "base 64";
        break;
        
      case XmlRpc::XmlRpcValue::TypeArray:
        type_str = "array";
        break;
        
      case XmlRpc::XmlRpcValue::TypeStruct:
        type_str = "struct";
        break;

      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown XML RPC value type encountered");
        type_str = "unknown";
        break;
    }

    return type_str;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::ReplaceColorsNodelet)
