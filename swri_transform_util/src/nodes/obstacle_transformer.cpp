// *****************************************************************************
//
// Copyright (C) 2019 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <rclcpp/rclcpp.hpp>

#include <marti_nav_msgs/msg/obstacle.hpp>
#include <marti_nav_msgs/msg/obstacle_array.hpp>

#include <swri_roscpp/publisher.h>
#include <swri_roscpp/subscriber.h>

#include <swri_transform_util/transform_manager.h>

namespace swri_transform_util
{
class ObstacleTransformer : public rclcpp::Node
  {
  public:
    explicit ObstacleTransformer(const rclcpp::NodeOptions& options) :
      rclcpp::Node("obstacle_transformer", options),
      tf_manager_(shared_from_this())
    {
      this->declare_parameter("output_frame", "/wgs84");

      output_frame_ = this->get_parameter("output_frame").as_string();

      object_array_sub_ = swri::Subscriber(*this,
                                           "object_array",
                                           1,
                                           &ObstacleTransformer::handleObstacleArray,
                                           this);

      viz_pub_ = swri::advertise<marti_nav_msgs::msg::ObstacleArray>(*this, "viz_array", 1);
    }

  private:
    void handleObstacleArray(const marti_nav_msgs::msg::ObstacleArray::ConstSharedPtr& obj)
    {
      if (viz_pub_->get_subscription_count() == 0 && viz_pub_->get_intra_process_subscription_count() == 0)
      {
        return;
      }

      marti_nav_msgs::msg::ObstacleArray::UniquePtr obstacles = std::make_unique<marti_nav_msgs::msg::ObstacleArray>();
      *obstacles = *obj;
      obstacles->header.frame_id = output_frame_;

      swri_transform_util::Transform transform;
      if (!tf_manager_.GetTransform(output_frame_, obj->header.frame_id, transform))
      {
        RCLCPP_WARN(this->get_logger(), "Failed to get transform.");
        return;
      }

      for (auto& ob: obstacles->obstacles)
      {
        tf2::Transform local_transform;
        tf2::fromMsg(ob.pose, local_transform);
        ob.pose.position.x = 0;
        ob.pose.position.y = 0;
        ob.pose.position.z = 0;
        ob.pose.orientation.x = 0.0;
        ob.pose.orientation.y = 0.0;
        ob.pose.orientation.z = 0.0;
        ob.pose.orientation.w = 1.0;
        for (auto& point: ob.polygon)
        {
          tf2::Vector3 p(point.x, point.y, 0.0);
          p = local_transform*p;
           
          p = transform*p;
          point.x = p.x();
          point.y = p.y();
        }
      }

      viz_pub_->publish(std::move(obstacles));
    }
    
    swri::Subscriber object_array_sub_;
    rclcpp::Publisher<marti_nav_msgs::msg::ObstacleArray>::SharedPtr viz_pub_;

    // parameters
    std::string output_frame_;

    swri_transform_util::TransformManager tf_manager_;
  };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(swri_transform_util::ObstacleTransformer)
