// *****************************************************************************
//
// Copyright (C) 2011 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-R8248
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <string>

// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class DummyImagePublisherNode
{
  public:
    DummyImagePublisherNode()
    {
      ros::NodeHandle node;
      ros::NodeHandle priv("~");
      image_pub_ = node.advertise<sensor_msgs::Image>("image", 100);
      
      priv.param("rate", rate_, 10.0);
      priv.param("width", width_, 640);
      priv.param("height", height_, 480);
      priv.param("encoding", encoding_, std::string("mono8"));
    }

    void Spin()
    {
      sensor_msgs::Image image;
      image.encoding = encoding_;
      image.width = width_;
      image.height = height_;
      image.step = width_;
      image.data.resize(height_ * width_);
    
      ros::Rate rate(rate_);
      while (ros::ok())
      {
        image.header.stamp = ros::Time::now();
        image_pub_.publish(image);
      
        ros::spinOnce();
        rate.sleep();
      }
    }

  private:
    double rate_;
    int32_t width_;
    int32_t height_;
    std::string encoding_;
    
    ros::Publisher image_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_image_publisher", ros::init_options::AnonymousName);

  DummyImagePublisherNode node;
  node.Spin();
}
