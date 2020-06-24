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

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <marti_nav_msgs/Obstacle.h>
#include <marti_nav_msgs/ObstacleArray.h>

#include <swri_roscpp/publisher.h>
#include <swri_roscpp/subscriber.h>

#include <swri_transform_util/transform_manager.h>

namespace swri_transform_util
{
  class ObstacleTransformer : public nodelet::Nodelet
  {
  public:
    virtual void onInit()
    {
      ros::NodeHandle node = getMTNodeHandle();

      init_timer_ = node.createTimer(
          ros::Duration(1.0), &ObstacleTransformer::Initialize, this, true);
    }

  private:
    void Initialize(const ros::TimerEvent& unused)
    {
      ros::NodeHandle priv = getMTPrivateNodeHandle();
      ros::NodeHandle node = getMTNodeHandle();

      tf_manager_.Initialize();

      swri::param(priv, "output_frame", output_frame_, "/wgs84");

      object_array_sub_ = swri::Subscriber(node,
                                           "object_array",
                                           1,
                                           &ObstacleTransformer::handleObstacleArray,
                                           this);

      viz_pub_ = swri::advertise<marti_nav_msgs::ObstacleArray>(node, "viz_array", 1);
    }

    void handleObstacleArray(const marti_nav_msgs::ObstacleArrayConstPtr& obj)
    {
      if (viz_pub_.getNumSubscribers() == 0)
      {
        return;
      }

      marti_nav_msgs::ObstacleArrayPtr obstacles = boost::make_shared<marti_nav_msgs::ObstacleArray>();
      *obstacles = *obj;
      obstacles->header.frame_id = output_frame_;

      swri_transform_util::Transform transform;
      if (!tf_manager_.GetTransform(output_frame_, obj->header.frame_id, transform))
      {
        NODELET_WARN("Failed to get transform.");
        return;
      }

      for (auto& ob: obstacles->obstacles)
      {
        tf::Transform local_transform;
        tf::poseMsgToTF(ob.pose, local_transform);
        ob.pose.position.x = 0;
        ob.pose.position.y = 0;
        ob.pose.position.z = 0;
        ob.pose.orientation.x = 0.0;
        ob.pose.orientation.y = 0.0;
        ob.pose.orientation.z = 0.0;
        ob.pose.orientation.w = 1.0;
        for (auto& point: ob.polygon)
        {
          tf::Vector3 p(point.x, point.y, 0.0);
          p = local_transform*p;
           
          p = transform*p;
          point.x = p.x();
          point.y = p.y();
        }
      }

      viz_pub_.publish(obstacles);
    }

    

    ros::Timer init_timer_;

    swri::Subscriber object_array_sub_;
    ros::Publisher viz_pub_;

    // parameters
    std::string output_frame_;

    swri_transform_util::TransformManager tf_manager_;
  };
}

// Register nodelet plugin
#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(swri_transform_util, ObstacleTransformer)
