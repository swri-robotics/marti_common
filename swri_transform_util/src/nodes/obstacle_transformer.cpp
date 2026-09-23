// *****************************************************************************
//
// Copyright (c) 2019, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <rclcpp/rclcpp.hpp>

#include <marti_nav_msgs/msg/obstacle.hpp>
#include <marti_nav_msgs/msg/obstacle_array.hpp>

#include "swri_transform_util/transform_manager.h"

namespace swri_transform_util
{
class ObstacleTransformer : public rclcpp::Node
{
public:
  explicit ObstacleTransformer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("obstacle_transformer", options)
  {
    this->declare_parameter("output_frame", "/wgs84");

    output_frame_ = this->get_parameter("output_frame").as_string();

    object_array_sub_ = this->create_subscription<marti_nav_msgs::msg::ObstacleArray>(
      "object_array",
      1,
      std::bind(&ObstacleTransformer::handleObstacleArray, this, std::placeholders::_1));

    viz_pub_ = this->create_publisher<marti_nav_msgs::msg::ObstacleArray>("viz_array", 1);
  }

private:
  void InitTransformManager()
  {
    tf_manager_ = std::make_shared<swri_transform_util::TransformManager>(shared_from_this());
  }

  void handleObstacleArray(const marti_nav_msgs::msg::ObstacleArray::ConstSharedPtr & obj)
  {
    if (viz_pub_->get_subscription_count() == 0 &&
      viz_pub_->get_intra_process_subscription_count() == 0)
    {
      return;
    }

    marti_nav_msgs::msg::ObstacleArray::UniquePtr obstacles =
      std::make_unique<marti_nav_msgs::msg::ObstacleArray>();
    *obstacles = *obj;
    obstacles->header.frame_id = output_frame_;

    if (!tf_manager_) {
      InitTransformManager();
    }

    swri_transform_util::Transform transform;
    if (!tf_manager_->GetTransform(output_frame_, obj->header.frame_id, transform)) {
      RCLCPP_WARN(this->get_logger(), "Failed to get transform.");
      return;
    }

    for (auto & ob : obstacles->obstacles) {
      tf2::Transform local_transform;
      tf2::fromMsg(ob.pose, local_transform);
      ob.pose.position.x = 0;
      ob.pose.position.y = 0;
      ob.pose.position.z = 0;
      ob.pose.orientation.x = 0.0;
      ob.pose.orientation.y = 0.0;
      ob.pose.orientation.z = 0.0;
      ob.pose.orientation.w = 1.0;
      for (auto & point : ob.polygon) {
        tf2::Vector3 p(point.x, point.y, 0.0);
        p = local_transform * p;

        p = transform * p;
        point.x = p.x();
        point.y = p.y();
      }
    }

    viz_pub_->publish(std::move(obstacles));
  }

  rclcpp::Subscription<marti_nav_msgs::msg::ObstacleArray>::SharedPtr object_array_sub_;
  rclcpp::Publisher<marti_nav_msgs::msg::ObstacleArray>::SharedPtr viz_pub_;

  // parameters
  std::string output_frame_;

  std::shared_ptr<swri_transform_util::TransformManager> tf_manager_;
};
}  // namespace swri_transform_util

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_transform_util::ObstacleTransformer)
