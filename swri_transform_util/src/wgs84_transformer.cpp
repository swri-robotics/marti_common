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

#ifdef USE_TF2_H_FILES
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <swri_math_util/trig_util.h>
#include <swri_transform_util/frames.h>

namespace swri_transform_util
{
  Wgs84Transformer::Wgs84Transformer(LocalXyWgs84UtilPtr local_xy_util)
  {
    local_xy_util_ = local_xy_util;
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
    const tf2::TimePoint& time,
    Transform& transform)
  {
    if (!initialized_)
    {
      Initialize();
    }

    if (!initialized_)
    {
      RCLCPP_WARN(logger_, "Wgs84Transformer not initialized");
      return false;
    }

    if (FrameIdsEqual(target_frame, _wgs84_frame))
    {
      geometry_msgs::msg::TransformStamped tf_transform;
      if (!Transformer::GetTransform(local_xy_frame_, source_frame , time, tf_transform))
      {
        RCLCPP_WARN(logger_, "Failed to get transform between %s and %s",
            source_frame.c_str(), local_xy_frame_.c_str());
        return false;
      }

      transform = std::make_shared<TfToWgs84Transform>(tf_transform, local_xy_util_);

      return true;
    }
    else if (FrameIdsEqual(source_frame, _wgs84_frame))
    {
      geometry_msgs::msg::TransformStamped tf_transform;
      if (!Transformer::GetTransform(target_frame, local_xy_frame_, time, tf_transform))
      {
        RCLCPP_WARN(logger_, "Failed to get transform between %s and %s",
            local_xy_frame_.c_str(), target_frame.c_str());
        return false;
      }

      transform = std::make_shared<Wgs84ToTfTransform>(tf_transform, local_xy_util_);

      return true;
    }

    RCLCPP_WARN(logger_, "Failed to get WGS84 transform.");
    return false;
  }

  bool Wgs84Transformer::Initialize()
  {
    if (!local_xy_util_)
    {
      RCLCPP_ERROR(logger_, "Wgs84Transformer::Initialize: local_yx_util was unset!");
      return false;
    }

    if (local_xy_util_->Initialized())
    {
      std::string local_xy_frame = local_xy_util_->Frame();
      if (tf_buffer_->_frameExists(local_xy_frame))
      {
        local_xy_frame_ = local_xy_frame;
        initialized_ = true;
      }
    }

    return initialized_;
  }
  
  TfToWgs84Transform::TfToWgs84Transform(
    const geometry_msgs::msg::TransformStamped& transform,
    std::shared_ptr<LocalXyWgs84Util> local_xy_util) :
    local_xy_util_(local_xy_util)
  {
    transform_ = transform;
  }

  void TfToWgs84Transform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    // Transform into the LocalXY coordinate frame using the TF transform.
    tf2::Stamped<tf2::Transform> tf = GetStampedTransform();
    tf2::Vector3 local_xy = tf * v_in;

    // Convert to WGS84 latitude and longitude.
    double latitude, longitude;
    local_xy_util_->ToWgs84(local_xy.x(), local_xy.y(), latitude, longitude);
    v_out.setValue(longitude, latitude, local_xy.z());
  }
  
  tf2::Quaternion TfToWgs84Transform::GetOrientation() const
  {
    tf2::Stamped<tf2::Transform> tf;
    tf2::fromMsg(transform_, tf);
    tf2::Quaternion reference_angle;
    reference_angle.setRPY(0, 0, swri_math_util::ToRadians(local_xy_util_->ReferenceAngle()));
 
    return tf.getRotation() * reference_angle;
  }

  TransformImplPtr TfToWgs84Transform::Inverse() const
  {
    tf2::Stamped<tf2::Transform> inverse_transform = GetStampedTransform();
    inverse_transform.setData(inverse_transform.inverse());

#if USE_NEW_TF2_TOMSG == 1
    auto inverse_tf_msg = tf2::toMsg(inverse_transform);
#else
    auto inverse_tf_msg =
      tf2::toMsg<tf2::Stamped<tf2::Transform>, geometry_msgs::msg::TransformStamped>(inverse_transform);
#endif
    inverse_tf_msg.header.frame_id = transform_.child_frame_id;
    inverse_tf_msg.child_frame_id = transform_.header.frame_id;
    TransformImplPtr inverse = std::make_shared<Wgs84ToTfTransform>(
        inverse_tf_msg,
        local_xy_util_);
    return inverse;
  }
  
  Wgs84ToTfTransform::Wgs84ToTfTransform(
    const geometry_msgs::msg::TransformStamped& transform,
    std::shared_ptr<LocalXyWgs84Util> local_xy_util) :
    local_xy_util_(local_xy_util)
  {
    transform_ = transform;
  }

  void Wgs84ToTfTransform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    // Convert to LocalXY coordinate frame.
    double x, y;
    local_xy_util_->ToLocalXy(v_in.y(), v_in.x(), x, y);
    v_out.setValue(x, y, v_in.z());

    // Transform from the LocalXY coordinate frame using the TF transform.
    v_out = GetStampedTransform() * v_out;
  }
  
  tf2::Quaternion Wgs84ToTfTransform::GetOrientation() const
  {
    tf2::Quaternion reference_angle;
    reference_angle.setRPY(0, 0, swri_math_util::ToRadians(local_xy_util_->ReferenceAngle()));

    tf2::Stamped<tf2::Transform> tf = GetStampedTransform();
      
    return GetStampedTransform().getRotation() * reference_angle.inverse();
  }

  TransformImplPtr Wgs84ToTfTransform::Inverse() const
  {
    tf2::Stamped<tf2::Transform> inverse_transform;
    tf2::fromMsg(transform_, inverse_transform);
    inverse_transform.setData(inverse_transform.inverse());

    geometry_msgs::msg::TransformStamped inverse_tf_msg;
    inverse_tf_msg.header.frame_id = transform_.child_frame_id;
    inverse_tf_msg.child_frame_id = transform_.header.frame_id;
    TransformImplPtr inverse = std::make_shared<TfToWgs84Transform>(
        inverse_tf_msg,
        local_xy_util_);
    return inverse;
  }
}

