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

#include <algorithm>

#include <boost/make_shared.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <swri_roscpp/parameters.h>

namespace swri_image_util
{
  class ImagePubNodelet : public nodelet::Nodelet
  {
    public:
      virtual void onInit()
      {
        ros::NodeHandle node = getNodeHandle();

        init_timer_ = node.createTimer(
            ros::Duration(1.0), &ImagePubNodelet::initialize, this, true);
      }

      void initialize(const ros::TimerEvent& unused)
      {
        ros::NodeHandle &node = getNodeHandle();
        ros::NodeHandle &priv = getPrivateNodeHandle();

        std::string image_file;
        swri::param(priv, "image_file", image_file, image_file);

        std::string mode;
        swri::param(priv, "mode", mode, sensor_msgs::image_encodings::BGR8);

        double rate = 1;
        swri::param(priv, "rate", rate, rate);
        rate = std::max(0.1, rate);

        cv_image.header.stamp = ros::Time::now();
        if (mode == sensor_msgs::image_encodings::BGR8)
        {
          cv_image.image = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
          cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        }
        else
        {
          cv_image.image = cv::imread(image_file, CV_LOAD_IMAGE_GRAYSCALE);
          cv_image.encoding = sensor_msgs::image_encodings::MONO8;
        }

        if (!cv_image.image.empty())
        {
          image_transport::ImageTransport it(node);
          image_pub_ = it.advertise("image", 2, true);
          pub_timer_ = node.createTimer(
              ros::Duration(1.0 / rate), &ImagePubNodelet::publish, this);
        }
        else
        {
          ROS_FATAL("Failed to load image.");
          ros::requestShutdown();
        }
      }

      void publish(const ros::TimerEvent& e)
      {
        cv_image.header.stamp = e.current_real;
        image_pub_.publish(cv_image.toImageMsg());
      }

    private:
      ros::Timer init_timer_;
      ros::Timer pub_timer_;
      image_transport::Publisher image_pub_;

      cv_bridge::CvImage cv_image;
  };
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(swri_image_util, ImagePubNodelet)
