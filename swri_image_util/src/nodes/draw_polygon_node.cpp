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

#ifdef USE_CVBRIDGE_H_FILES
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace swri_image_util
{
  class DrawPolygonNode : public rclcpp::Node
  {
  public:
    explicit DrawPolygonNode(const rclcpp::NodeOptions& options) :
        rclcpp::Node("draw_polygon", options)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      desc.name = "thickness";
      this->declare_parameter("thickness", -1, desc);
      desc.name = "r";
      this->declare_parameter("r", 0, desc);
      desc.name = "g";
      this->declare_parameter("g", 0, desc);
      desc.name = "b";
      this->declare_parameter("b", 0, desc);

      rcl_interfaces::msg::ParameterDescriptor xDesc;
      xDesc.name = "polygon.x";
      xDesc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
      xDesc.read_only = true;
      rcl_interfaces::msg::ParameterDescriptor yDesc;
      yDesc.name = "polygon.y";
      yDesc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
      yDesc.read_only = true;
      typedef std::pair<std::vector<int64_t>, rcl_interfaces::msg::ParameterDescriptor> CoordParamType;
      this->declare_parameters("polygon", std::map<std::string, CoordParamType>{
        {"x", {{}, xDesc}},
        {"y", {{}, yDesc}}
      });

      std::map<std::string, std::vector<int64_t> > points;

      this->get_parameters("polygon", points);
      if (points.find("x") == points.end() ||
          points.find("y") == points.end() ||
          points["x"].size() != points["y"].size())
      {
        RCLCPP_FATAL(this->get_logger(), "'polygon' param must have an equal number of 'x' and 'y' points.");
      }
      std::vector<int64_t>& x_points = points["x"];
      std::vector<int64_t>& y_points = points["y"];
      for (size_t i = 0; i < x_points.size(); i++)
      {
        polygon_.emplace_back(cv::Point(x_points[i], y_points[i]));
      }

      auto callback = [this](const sensor_msgs::msg::Image::ConstSharedPtr& image) -> void
      {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);

        const cv::Point* points[1] = {&polygon_[0]};
        int count = polygon_.size();

        auto params = this->get_parameters(std::vector<std::string>{"thickness", "r", "g", "b"});

        if (params.at(0).as_int() < 1)
        {
          cv::fillPoly(cv_image->image, points, &count, 1,
              cv::Scalar(params.at(3).as_int(),
                  params.at(2).as_int(),
                  params.at(1).as_int()));
        }
        else
        {
          cv::polylines(cv_image->image, points, &count, 1, true,
              cv::Scalar(params.at(3).as_int(),
                  params.at(2).as_int(),
                  params.at(1).as_int()),
                  params.at(0).as_int());
        }

        image_pub_.publish(cv_image->toImageMsg());
      };

      image_pub_ = image_transport::create_publisher(this, "image_out");
      image_sub_ = image_transport::create_subscription(this, "image_in", callback, "raw");
    }

  private:

    std::vector<cv::Point> polygon_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_image_util::DrawPolygonNode)
