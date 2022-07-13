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
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace swri_image_util
{
  class ImagePubNode : public rclcpp::Node
  {
  public:
    explicit ImagePubNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("image_pub", options)
    {
      this->declare_parameter<std::string>("image_file", "");
      this->declare_parameter<std::string>("mode", sensor_msgs::image_encodings::BGR8);
      this->declare_parameter<double>("rate", 1);

      std::string image_file = this->get_parameter("image_file").as_string();
      std::string mode = this->get_parameter("mode").as_string();
      double rate = this->get_parameter("rate").as_double();

      rate = std::max(0.1, rate);

      cv_image.header.stamp = rclcpp::Clock().now();
      if (mode == sensor_msgs::image_encodings::BGR8)
      {
        cv_image.image = cv::imread(image_file, cv::IMREAD_COLOR);
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
      }
      else
      {
        cv_image.image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        cv_image.encoding = sensor_msgs::image_encodings::MONO8;
      }

      if (!cv_image.image.empty())
      {
        rmw_qos_profile_t qos = rmw_qos_profile_default;
        qos.depth = 2;
        qos.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        image_pub_ = image_transport::create_publisher(this, "image", qos);
        pub_timer_ = this->create_wall_timer(
            std::chrono::duration<float>(1.0 / rate),
            std::bind(&ImagePubNode::publish, this));
      }
      else
      {
        RCLCPP_FATAL(this->get_logger(), "Failed to load image.");
        rclcpp::shutdown();
      }
    }

    void publish()
    {
      cv_image.header.stamp = rclcpp::Clock().now();
      image_pub_.publish(cv_image.toImageMsg());
    }

  private:
    rclcpp::TimerBase::SharedPtr pub_timer_;
    image_transport::Publisher image_pub_;

    cv_bridge::CvImage cv_image;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::ImagePubNode)
