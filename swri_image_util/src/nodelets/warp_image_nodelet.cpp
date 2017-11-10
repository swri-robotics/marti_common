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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <opencv2/core/core.hpp>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <swri_roscpp/parameters.h>

namespace swri_image_util
{
  class WarpImageNodelet : public nodelet::Nodelet
  {
    public:
      WarpImageNodelet(): use_input_size_(false)
      {
      }

      ~WarpImageNodelet()
      {
      }

      void onInit()
      {
        ros::NodeHandle &node = getNodeHandle();
        ros::NodeHandle &priv = getPrivateNodeHandle();

        std::vector<double> transform;
        if (priv.hasParam("width") && priv.hasParam("height"))
        {
          use_input_size_ = false;
          swri::getParam(priv, "width", output_size_.width);
          swri::getParam(priv, "height", output_size_.height);
        }
        else
        {
          use_input_size_ = true;
          NODELET_INFO("No ~width and ~height parameters given. Output images will be same size as input.");
        }
        priv.param("transform", transform, transform);
        if (transform.size() != 9)
        {
          NODELET_FATAL("~transform must be a 9-element list of doubles (3x3 matrix, row major)");
          // Return without setting up callbacks
          // Don't shut down, because that would bring down all other nodelets as well
          return;
        }
        m_ = cv::Mat(transform, true).reshape(0, 3);
        NODELET_INFO_STREAM("Transformation matrix:" << std::endl << m_);

        image_transport::ImageTransport it(node);
        image_pub_ = it.advertise("warped_image", 1);
        image_sub_ = it.subscribe("image", 1, &WarpImageNodelet::handleImage, this);
      }

      void handleImage(sensor_msgs::ImageConstPtr const& image)
      {
        cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);

        cv_bridge::CvImagePtr cv_warped = boost::make_shared<cv_bridge::CvImage>();
        if (use_input_size_)
        {
          output_size_ = cv_image->image.size();
        }
        cv::warpPerspective(cv_image->image, cv_warped->image, m_, output_size_, CV_INTER_LANCZOS4);

        cv_warped->encoding = cv_image->encoding;
        cv_warped->header = cv_image->header;

        image_pub_.publish(cv_warped->toImageMsg());
      }

    private:
      image_transport::Subscriber image_sub_;
      image_transport::Publisher image_pub_;
      cv::Mat m_;
      bool use_input_size_;
      cv::Size output_size_;
  };
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(swri_image_util, WarpImageNodelet)
