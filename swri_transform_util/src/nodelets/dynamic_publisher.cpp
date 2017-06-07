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
#include <string>

#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <swri_roscpp/swri_roscpp.h>
#include <swri_transform_util/DynamicPublisherConfig.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace swri_transform_util {

class DynamicPublisher : public nodelet::Nodelet
{
  public:
  DynamicPublisher() : rate_(50), stamp_offset_(1.0)
  {
  }

  ~DynamicPublisher()
  {
  }

  void onInit()
  {
    ros::NodeHandle &node = getNodeHandle();
    init_timer_ = node.createTimer(
        ros::Duration(1.0), &DynamicPublisher::Initialize, this, true);
  }

  private:
  void Initialize(const ros::TimerEvent& unused)
  {
    ros::NodeHandle priv = getPrivateNodeHandle();
    ros::NodeHandle node = getNodeHandle();

    swri::param(priv, "rate", rate_, rate_);
    swri::param(priv, "stamp_offset", stamp_offset_, stamp_offset_);
    swri::param(priv, "child_frame", child_frame_, "child");
    swri::param(priv, "parent_frame", parent_frame_, "parent");
    swri::param(priv, "x", config_.x, config_.x);
    swri::param(priv, "y", config_.y, config_.y);
    swri::param(priv, "z", config_.z, config_.z);
    swri::param(priv, "yaw", config_.yaw, config_.yaw);
    swri::param(priv, "pitch", config_.pitch, config_.pitch);
    swri::param(priv, "roll", config_.roll, config_.roll);

    server_.setCallback(
        boost::bind(&DynamicPublisher::ConfigCallback, this, _1, _2));

    rate_ = std::max(1.0, rate_);
    pub_timer_ = node.createTimer(
        ros::Duration(1.0 / rate_), &DynamicPublisher::Publish, this);
  }

  void ConfigCallback(DynamicPublisherConfig &config, uint32_t level)
  {
    if (~level == 0)
    {
      // When the dynamic reconfigure server is started, it tries to
      // set default values by calling this function with all bits of
      // level set.  We reject these defaults in favor of our
      // currently stored values.
      config = config_;
    }
    else
    {
      config_ = config;
    }
  }

  void Publish(const ros::TimerEvent& unused)
  {
    tf::Transform transform;
    tf::Vector3 origin(config_.x, config_.y, config_.z);
    tf::Quaternion rotation;
    rotation.setRPY(config_.roll, config_.pitch, config_.yaw);
    transform = tf::Transform(rotation, origin);

    tf::StampedTransform stamped_transform(
        transform,
        ros::Time::now() + ros::Duration(stamp_offset_),
        parent_frame_,
        child_frame_);

    tf_broadcaster_.sendTransform(stamped_transform);
  }

  ros::Timer init_timer_;
  ros::Timer pub_timer_;

  tf::TransformBroadcaster tf_broadcaster_;

  double rate_;
  double stamp_offset_;
  std::string child_frame_;
  std::string parent_frame_;
  DynamicPublisherConfig config_;

  typedef dynamic_reconfigure::Server<DynamicPublisherConfig> ReconfigureServer;
  ReconfigureServer server_;
};

}  // namespace swri_transform_util

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  swri_transform_util::DynamicPublisher,
  nodelet::Nodelet)
