#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <marti_nav_msgs/ObstacleArray.h>

ros::Publisher pub;

void callback(const marti_nav_msgs::ObstacleArrayConstPtr& msg)
{
  ROS_INFO("Got Obstacle Array");
  visualization_msgs::MarkerArray markers;
  markers.markers.reserve(msg->obstacles.size());
  for (int i = 0; i < msg->obstacles.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = msg->header.stamp;
    marker.id = i;
    
    marker.ns = "obstacle shapes";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;

    marker.lifetime = ros::Duration(0);
    

    marker.pose = msg->obstacles[i].pose;
    //marker.pose.position.x = 0.1;
    //marker.pose.position.y = 0;
    //marker.pose.position.z = 0;
    marker.points.reserve(msg->obstacles[i].polygon.size());
    for (int v = 0; v < msg->obstacles[i].polygon.size(); v++)
    {
      marker.points.push_back(msg->obstacles[i].polygon[v]);
    }

    marker.points.push_back(msg->obstacles[i].polygon[0]);
    markers.markers.push_back(marker);
  }

  pub.publish(markers);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,
      "obstacle_to_marker_node",
      ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ROS_INFO("Starting converter...");

  pub = nh.advertise<visualization_msgs::MarkerArray>("object_detection_markers", 1);
  ros::Subscriber sub = nh.subscribe("/lidar/object_detections", 200, callback);
 
  //do stuff
  ros::spin();

  return 0;
}
