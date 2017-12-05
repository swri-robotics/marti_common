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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <XmlRpcException.h>

namespace swri_image_util
{
  class DrawPolygonNodelet : public nodelet::Nodelet
  {
  public:
      DrawPolygonNodelet() :
        thickness_(-1),
        r_(0),
        g_(0),
        b_(0)
    {
    }

    void onInit()
    {
      ros::NodeHandle &node = getNodeHandle();
      ros::NodeHandle &priv = getPrivateNodeHandle();

      priv.param("thickness", thickness_, thickness_);
      priv.param("r", r_, r_);
      priv.param("g", g_, g_);
      priv.param("b", b_, b_);

      try
      {
        XmlRpc::XmlRpcValue polygon;
        priv.getParam("polygon", polygon);
        for (size_t i = 0; i < polygon.size(); i++)
        {
          const XmlRpc::XmlRpcValue& point = polygon[i];
          XmlRpc::XmlRpcValue x_param = point[0];
          XmlRpc::XmlRpcValue y_param = point[1];
          polygon_.push_back(cv::Point(
              static_cast<int>(x_param),
              static_cast<int>(y_param)));
        }
      }
      catch (XmlRpc::XmlRpcException& e)
      {
        ROS_ERROR("Failed to parse polygon: %s", e.getMessage().c_str());
        ros::requestShutdown();
        return;
      }


      image_transport::ImageTransport it(node);
      image_pub_ = it.advertise("image_out", 1);
      image_sub_ = it.subscribe("image_in", 1, &DrawPolygonNodelet::imageCallback, this);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& image)
    {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);

      const cv::Point* points[1] = { &polygon_[0] };
      int count = polygon_.size();

      if (thickness_ < 1)
      {
        cv::fillPoly(cv_image->image, points, &count, 1, cv::Scalar(b_, g_, r_));
      }
      else
      {
        cv::polylines(cv_image->image, points, &count, 1, true, cv::Scalar(b_, g_, r_), thickness_);
      }

      image_pub_.publish(cv_image->toImageMsg());
    }

  private:
    int thickness_;
    int r_;
    int g_;
    int b_;

    std::vector<cv::Point> polygon_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(swri_image_util, DrawPolygonNodelet);
