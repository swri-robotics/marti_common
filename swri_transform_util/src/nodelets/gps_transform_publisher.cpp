// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
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

#include <gps_common/GPSFix.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <swri_roscpp/parameters.h>
#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform_manager.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace swri_transform_util
{
class GpsTransformPublisher : public nodelet::Nodelet
{
 private:
  ros::Subscriber gps_sub_;

  tf::TransformBroadcaster tf_;

  swri_transform_util::TransformManager tf_manager_;

  std::string veh_frame_id_;
  std::string global_frame_id_;

 public:
  void onInit();


  void HandleGps(const gps_common::GPSFixPtr& gps_fix);
};

void GpsTransformPublisher::onInit()
{
  ros::NodeHandle priv = getPrivateNodeHandle();
  swri::param(priv,"child_frame_id", veh_frame_id_, std::string("base_link"));
  swri::param(priv,"parent_frame_id", global_frame_id_, std::string("map"));

  gps_sub_ = getNodeHandle().subscribe("gps", 100, &GpsTransformPublisher::HandleGps, this);

  tf_manager_.Initialize();
}

void GpsTransformPublisher::HandleGps(const gps_common::GPSFixPtr& gps_fix)
{
  tf::Transform transform;

  // Get the orientation from the GPS track.
  // NOTE: This will be unreliable when the vehicle is stopped or moving at low
  //       speed.
  double yaw = (90.0 - gps_fix->track) * swri_math_util::_deg_2_rad;
  yaw = swri_math_util::WrapRadians(yaw, swri_math_util::_pi);
  tf::Quaternion orientation;
  orientation.setRPY(0, 0, yaw);
  transform.setRotation(orientation);

  // Get the position by converting lat/lon to LocalXY.
  swri_transform_util::Transform to_local_xy;
  if (tf_manager_.GetTransform(global_frame_id_, swri_transform_util::_wgs84_frame, ros::Time(0), to_local_xy))
  {
    tf::Vector3 position(gps_fix->longitude, gps_fix->latitude, gps_fix->altitude);
    position = to_local_xy * position;
    transform.setOrigin(position);

    tf_.sendTransform(tf::StampedTransform(
        transform,
        gps_fix->header.stamp,
        global_frame_id_,
        veh_frame_id_));
  }
}
}  // namespace swri_transform_util

#include <swri_nodelet/class_list_macros.h>
SWRI_NODELET_EXPORT_CLASS(swri_transform_util, GpsTransformPublisher)

