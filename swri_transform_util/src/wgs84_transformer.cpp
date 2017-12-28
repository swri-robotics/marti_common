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

#include <swri_transform_util/wgs84_transformer.h>

#include <boost/make_shared.hpp>

#include <swri_math_util/trig_util.h>
#include <swri_transform_util/frames.h>

namespace swri_transform_util
{
  Wgs84Transformer::Wgs84Transformer()
  {
  }

  std::map<std::string, std::vector<std::string> > Wgs84Transformer::Supports() const
  {
    std::map<std::string, std::vector<std::string> >  supports;

    supports[_wgs84_frame].push_back(_tf_frame);
    supports[_tf_frame].push_back(_wgs84_frame);

    return supports;
  }

  bool Wgs84Transformer::GetTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const ros::Time& time,
    Transform& transform)
  {
    if (!initialized_)
    {
      Initialize();
    }

    if (!initialized_)
    {
      ROS_WARN_THROTTLE(2.0, "Wgs84Transformer not initialized");
      return false;
    }

    if (FrameIdsEqual(target_frame, _wgs84_frame))
    {
      tf::StampedTransform tf_transform;
      if (!Transformer::GetTransform(local_xy_frame_, source_frame , time, tf_transform))
      {
        ROS_WARN_THROTTLE(2.0, "Failed to get transform between %s and %s",
            source_frame.c_str(), local_xy_frame_.c_str());
        return false;
      }

      transform = boost::make_shared<TfToWgs84Transform>(tf_transform, local_xy_util_);

      return true;
    }
    else if (FrameIdsEqual(source_frame, _wgs84_frame))
    {
      tf::StampedTransform tf_transform;
      if (!Transformer::GetTransform(target_frame, local_xy_frame_, time, tf_transform))
      {
        ROS_WARN_THROTTLE(2.0, "Failed to get transform between %s and %s",
            local_xy_frame_.c_str(), target_frame.c_str());
        return false;
      }

      transform = boost::make_shared<Wgs84ToTfTransform>(tf_transform, local_xy_util_);

      return true;
    }

    ROS_WARN_THROTTLE(2.0, "Failed to get WGS84 transform.");
    return false;
  }

  bool Wgs84Transformer::Initialize()
  {
    if (!local_xy_util_)
    {
      local_xy_util_ = boost::make_shared<LocalXyWgs84Util>();
    }

    if (local_xy_util_->Initialized())
    {
      std::string local_xy_frame = local_xy_util_->Frame();
      if (tf_listener_->frameExists(local_xy_frame))
      {
        local_xy_frame_ = local_xy_frame;
        initialized_ = true;
      }
    }

    return initialized_;
  }
  
  TfToWgs84Transform::TfToWgs84Transform(
    const tf::StampedTransform& transform,
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util) :
    transform_(transform),
    local_xy_util_(local_xy_util)
  {
    stamp_ = transform.stamp_;
  }

  void TfToWgs84Transform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    // Transform into the LocalXY coordinate frame using the TF transform.
    tf::Vector3 local_xy = transform_ * v_in;

    // Convert to WGS84 latitude and longitude.
    double latitude, longitude;
    local_xy_util_->ToWgs84(local_xy.x(), local_xy.y(), latitude, longitude);
    v_out.setValue(longitude, latitude, local_xy.z());
  }
  
  tf::Quaternion TfToWgs84Transform::GetOrientation() const
  {
    tf::Quaternion reference_angle = tf::createQuaternionFromYaw(
      swri_math_util::ToRadians(local_xy_util_->ReferenceAngle()));
 
    return transform_.getRotation() * reference_angle;
  }

  TransformImplPtr TfToWgs84Transform::Inverse() const
  {
    tf::StampedTransform inverse_transform = transform_;
    inverse_transform.setData(transform_.inverse());
    inverse_transform.frame_id_ = transform_.child_frame_id_;
    inverse_transform.child_frame_id_ = transform_.frame_id_;
    TransformImplPtr inverse = boost::make_shared<Wgs84ToTfTransform>(
        inverse_transform,
        local_xy_util_);
    inverse->stamp_ = stamp_;
    return inverse;
  }
  
  Wgs84ToTfTransform::Wgs84ToTfTransform(
    const tf::StampedTransform& transform,
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util) :
    transform_(transform),
    local_xy_util_(local_xy_util)
  {
    stamp_ = transform.stamp_;
  }

  void Wgs84ToTfTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    // Convert to LocalXY coordinate frame.
    double x, y;
    local_xy_util_->ToLocalXy(v_in.y(), v_in.x(), x, y);
    v_out.setValue(x, y, v_in.z());

    // Transform from the LocalXY coordinate frame using the TF transform.
    v_out = transform_ * v_out;
  }
  
  tf::Quaternion Wgs84ToTfTransform::GetOrientation() const
  {
    tf::Quaternion reference_angle = tf::createQuaternionFromYaw(
      swri_math_util::ToRadians(local_xy_util_->ReferenceAngle()));
      
    return transform_.getRotation() * reference_angle.inverse();
  }

  TransformImplPtr Wgs84ToTfTransform::Inverse() const
  {
    tf::StampedTransform inverse_transform = transform_;
    inverse_transform.setData(transform_.inverse());
    inverse_transform.frame_id_ = transform_.child_frame_id_;
    inverse_transform.child_frame_id_ = transform_.frame_id_;
    TransformImplPtr inverse = boost::make_shared<TfToWgs84Transform>(
        inverse_transform,
        local_xy_util_);
    inverse->stamp_ = stamp_;
    return inverse;
  }
}

