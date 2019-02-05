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

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <swri_roscpp/dynamic_parameters.h>
#include <swri_roscpp/swri_roscpp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace swri_transform_util {

class DynamicPublisher : public nodelet::Nodelet
{
  swri::DoubleParam roll_, pitch_, yaw_;
  swri::DoubleParam x_, y_, z_;

  swri::DynamicParameters params_;

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

    params_.initialize(priv);

    params_.get("x", x_, 0.0, "X offset (m)", -10000.0, 10000.0);
    params_.get("y", y_, 0.0, "Y offset (m)", -10000.0, 10000.0);
    params_.get("z", z_, 0.0, "Z offset (m)", -10000.0, 10000.0);
    params_.get("roll", roll_, 0.0, "Roll offset (rad)", -3.1415, 3.1415);
    params_.get("pitch", pitch_, 0.0, "Pitch offset (rad)", -3.1415, 3.1415);
    params_.get("yaw", yaw_, 0.0, "Yaw offset (rad)", -3.1415, 3.1415);

    params_.finalize();

    rate_ = std::max(1.0, rate_);
    pub_timer_ = node.createTimer(
        ros::Duration(1.0 / rate_), &DynamicPublisher::Publish, this);
  }

  void Publish(const ros::TimerEvent& unused)
  {
    params_.mutex().lock();
    tf::Vector3 origin(*x_, *y_, *z_);
    tf::Quaternion rotation;
    rotation.setRPY(*roll_, *pitch_, *yaw_);
    params_.mutex().unlock();

    tf::Transform transform(rotation, origin);

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
};

}  // namespace swri_transform_util

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  swri_transform_util::DynamicPublisher,
  nodelet::Nodelet)
