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
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// RANGER Libraries
#include <swri_image_util/image_normalization.h>

class NormalizationImageNode
{
 private:
  ros::NodeHandle nh_;

  ros::Subscriber image_sub_;

  int32_t num_to_skip_;
  int32_t max_num_to_average_;
  std::string filename_;

  int32_t raw_count_;
  int32_t image_count_;

  bool image_written_;

  std::vector<cv::Mat> image_array_;


  void get_parameters()
  {
    nh_.param(ros::this_node::getName() + "/num_to_skip",
              num_to_skip_,
              200);

    nh_.param(ros::this_node::getName() + "/max_num_to_average",
              max_num_to_average_,
              100);

    std::string temp_filename;
    temp_filename = ros::package::getPath("ranger_common") +
                "/normalization_image.png";


    nh_.param(ros::this_node::getName() + "/filename",
              filename_,
              temp_filename);

    ROS_ERROR("Planning to write normalization image to: %s",
              filename_.c_str());
  }


  void subscribe_to_topics()
  {
    image_sub_ = nh_.subscribe("image",
                               2,
                               &NormalizationImageNode::image_cb,
                               this);
  }

  void image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (image_count_ >= max_num_to_average_)
    {
      ::sleep(1);
      return;
    }

    if (raw_count_++ % num_to_skip_ == 0)
    {
      image_count_++;
      ROS_ERROR("Got image %d of %d",
                image_count_,
                max_num_to_average_);

      cv_bridge::CvImagePtr im_ptr = cv_bridge::toCvCopy(msg);
      cv::Mat image(im_ptr->image);
      image_array_.push_back(image);
      if (image_count_ >= max_num_to_average_)
      {
        generate_and_write_image();
      }
    }
  }


  void generate_and_write_image()
  {
    cv::Mat norm_im = swri_image_util::generate_normalization_image(image_array_);
    if (!norm_im.empty())
    {
      try
      {
        cv::imwrite(filename_, norm_im);
      }
      catch (const std::exception& e)
      {
        ROS_ERROR("Failed to save the normalization image: %s",
                  e.what());
        return;
      }
      ROS_ERROR("Successfully wrote normalization image to: %s",
                filename_.c_str());

      image_written_ = true;
    }
    else
    {
      ROS_ERROR("Failed to generate a normalization image");
    }
  }

 public:
  explicit NormalizationImageNode(const ros::NodeHandle& nh):
    nh_(nh),
    num_to_skip_(20),
    max_num_to_average_(100),
    raw_count_(0),
    image_count_(0),
    image_written_(false)
  {
    filename_ = ros::package::getPath("ranger_common") +
                "/normalization_image.png";

    get_parameters();

    subscribe_to_topics();
  }

  void shut_down()
  {
    if (!image_written_ && image_array_.size() > 25)
    {
      generate_and_write_image();
      fprintf(stderr, "\nNode killed before enough frames received to generate "
                "normalized image, so a normalized image was generated with "
                "available frames (%d vs. %d)\n",
                image_count_,
                max_num_to_average_);
    }
    else if (!image_written_)
    {
      fprintf(stderr, "\nNode killed before enough frames received to generate "
                "normalized image: Too few frames in the buffer to generate a "
                "normalization image\n");
    }
    else
    {
      fprintf(stderr, "\nExiting normally\n");
    }
  }

  bool get_image_written()
  {
    return image_written_;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_normalization_node");

  ros::NodeHandle n;

  NormalizationImageNode node(n);

  ros::spin();

  node.shut_down();

  return 0;
}
