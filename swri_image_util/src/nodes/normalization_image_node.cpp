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
#include <vector>

// ROS Libraries
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <swri_image_util/image_normalization.h>
#include <image_transport/image_transport.hpp>

namespace swri_image_util
{
  class NormalizationImageNode : public rclcpp::Node
  {
  public:
    explicit NormalizationImageNode(const rclcpp::NodeOptions&) :
        rclcpp::Node("image_normalization_node"),
        raw_count_(0),
        image_count_(0),
        image_written_(false)
    {
      this->declare_parameter("num_to_skip", 20);
      this->declare_parameter("filename", ament_index_cpp::get_package_prefix("ranger_common") +
                                          "/normalization_image.png");
      this->declare_parameter("max_num_to_average", 100);

      subscribe_to_topics();
    }

    void shut_down()
    {
      if (!image_written_ && image_array_.size() > 25)
      {
        generate_and_write_image();
        RCLCPP_ERROR(this->get_logger(), "\nNode killed before enough frames received to generate "
                        "normalized image, so a normalized image was generated with "
                        "available frames (%d vs. %ld)\n",
                image_count_,
                this->get_parameter("max_num_to_average").as_int());
      }
      else if (!image_written_)
      {
        RCLCPP_ERROR(this->get_logger(), "\nNode killed before enough frames received to generate "
                        "normalized image: Too few frames in the buffer to generate a "
                        "normalization image\n");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "\nExiting normally\n");
      }
    }

    bool get_image_written()
    {
      return image_written_;
    }
  private:
    void subscribe_to_topics()
    {
      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) -> void
      {
        int64_t max_num_to_average = this->get_parameter("max_num_to_average").as_int();
        if (image_count_ >= max_num_to_average)
        {
          // ::sleep(1);
          return;
        }

        if (raw_count_++ % this->get_parameter("num_to_skip").as_int() == 0)
        {
          image_count_++;
          RCLCPP_ERROR(this->get_logger(), "Got image %d of %ld",
                       image_count_,
                       max_num_to_average);

          cv_bridge::CvImagePtr im_ptr = cv_bridge::toCvCopy(*msg);
          cv::Mat image(im_ptr->image);
          image_array_.push_back(image);
          if (image_count_ >= max_num_to_average)
          {
            generate_and_write_image();
          }
        }
      };
      rmw_qos_profile_t qos = rmw_qos_profile_default;
      qos.depth = 2;
      image_sub_ = image_transport::create_subscription(
          this,
          "image",
          callback,
          "raw",
          qos);
    }

    void generate_and_write_image()
    {
      cv::Mat norm_im = swri_image_util::generate_normalization_image(image_array_);
      if (!norm_im.empty())
      {
        std::string filename = this->get_parameter("filename").as_string();
        try
        {
          cv::imwrite(filename, norm_im);
        }
        catch (const std::exception& e)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to save the normalization image: %s",
                       e.what());
          return;
        }
        RCLCPP_ERROR(this->get_logger(), "Successfully wrote normalization image to: %s",
                     filename.c_str());

        image_written_ = true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate a normalization image");
      }
    }

    image_transport::Subscriber image_sub_;

    int32_t raw_count_;
    int32_t image_count_;

    bool image_written_;

    std::vector<cv::Mat> image_array_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::NormalizationImageNode)
