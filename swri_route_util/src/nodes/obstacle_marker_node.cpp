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
#include <cstddef>
#include <string>
#include <vector>

#include <swri_route_util/obstacle_marker_node.h>
#include <swri_route_util/visualization.h>

namespace swri_route_util
{
ObstacleMarkerNode::ObstacleMarkerNode(const rclcpp::NodeOptions &options) :
  rclcpp::Node("obstacle_markers", options),
  line_width_(0.1),
  marker_count_(0)
{
  marker_ns_ = this->declare_parameter("marker_namespace", std::string("obstacles"));
  line_width_ = this->declare_parameter("line_width", line_width_);

  // The color that the obstacles are drawn with, as red, green, blue,
  // and alpha values in the range [0, 1].
  const std::vector<double> default_color = {1.0, 0.0, 0.0, 1.0};
  std::vector<double> color = this->declare_parameter("color", default_color);
  if (color.size() != default_color.size()) {
    RCLCPP_ERROR(this->get_logger(),
      "The 'color' parameter must have 4 elements (r, g, b, a), but it has %zu. "
      "Falling back to the default color.", color.size());
    color = default_color;
  }
  color_.r = color[0];
  color_.g = color[1];
  color_.b = color[2];
  color_.a = color[3];

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "obstacle_markers", 1);

  obstacles_sub_ = this->create_subscription<marti_nav_msgs::msg::ObstacleArray>(
    "obstacles", 1,
    std::bind(&ObstacleMarkerNode::handleObstacles, this, std::placeholders::_1));
}

void ObstacleMarkerNode::handleObstacles(
  const marti_nav_msgs::msg::ObstacleArray::ConstSharedPtr obstacles)
{
  visualization_msgs::msg::MarkerArray markers;
  markerArrayForObstacles(markers, *obstacles, marker_ns_, color_, line_width_);

  // Markers are numbered sequentially, so any id at or past the end
  // of this message belongs to an obstacle that has gone away and
  // needs to be deleted.
  const size_t marker_count = markers.markers.size();
  for (size_t i = marker_count; i < marker_count_; i++) {
    visualization_msgs::msg::Marker m;
    m.header = obstacles->header;
    m.ns = marker_ns_;
    m.id = static_cast<int>(i);
    m.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(m);
  }
  marker_count_ = marker_count;

  markers_pub_->publish(markers);
}
}  // namespace swri_route_util

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_route_util::ObstacleMarkerNode)
