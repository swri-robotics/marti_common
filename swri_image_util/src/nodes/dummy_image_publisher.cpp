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
