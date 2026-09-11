// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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
#ifndef SWRI_ROUTE_UTIL_OBSTACLE_MARKER_NODE_H_
#define SWRI_ROUTE_UTIL_OBSTACLE_MARKER_NODE_H_

#include <cstddef>
#include <string>

#include <marti_nav_msgs/msg/obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace swri_route_util
{
// Converts obstacles into markers so that they can be displayed by
// tools such as RViz.  Subscribes to 'obstacles'
// (marti_nav_msgs/ObstacleArray) and publishes 'obstacle_markers'
// (visualization_msgs/MarkerArray).
class ObstacleMarkerNode : public rclcpp::Node
{
public:
  explicit ObstacleMarkerNode(const rclcpp::NodeOptions &options);

private:
  void handleObstacles(
    const marti_nav_msgs::msg::ObstacleArray::ConstSharedPtr obstacles);

  std::string marker_ns_;
  std_msgs::msg::ColorRGBA color_;
  double line_width_;

  // Number of markers published for the previous message.  Markers
  // that are no longer needed are explicitly deleted so that stale
  // obstacles do not linger in the display.
  size_t marker_count_;

  rclcpp::Subscription<marti_nav_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};
}  // namespace swri_route_util
#endif  // SWRI_ROUTE_UTIL_OBSTACLE_MARKER_NODE_H_
