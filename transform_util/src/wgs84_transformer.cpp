// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#include <transform_util/wgs84_transformer.h>

#include <boost/make_shared.hpp>

#include <transform_util/frames.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    transformers,
    wgs84,
    transform_util::Wgs84Transformer,
    transform_util::Transformer)

namespace transform_util
{
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
      ROS_ERROR("Wgs84Transformer not initialized");
      return false;
    }

    if (target_frame == _wgs84_frame)
    {
      tf::StampedTransform tf_transform;
      if (!Transformer::GetTransform(local_xy_util_->FrameId(), source_frame , time, tf_transform))
      {
        ROS_ERROR("Failed to get transform between %s and %s",
            source_frame.c_str(), local_xy_util_->FrameId().c_str());
        return false;
      }

      transform = boost::make_shared<TfToWgs84Transform>(tf_transform, local_xy_util_);

      return true;
    }
    else if (source_frame == _wgs84_frame)
    {
      tf::StampedTransform tf_transform;
      if (!Transformer::GetTransform(target_frame, local_xy_util_->FrameId(), time, tf_transform))
      {
        ROS_ERROR("Failed to get transform between %s and %s",
            local_xy_util_->FrameId().c_str(), target_frame.c_str());
        return false;
      }

      transform = boost::make_shared<Wgs84ToTfTransform>(tf_transform, local_xy_util_);

      return true;
    }

    ROS_ERROR("Failed to get WGS84 transform.");
    return false;
  }

  bool Wgs84Transformer::Initialize()
  {
    // Initialize LocalXY util with an origin.
    local_xy_util_ = ParseLocalXyOrigin();

    return local_xy_util_;
  }

  TfToWgs84Transform::TfToWgs84Transform(
    const tf::Transform& transform,
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util) :
    transform_(transform),
    local_xy_util_(local_xy_util)
  {
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

  Wgs84ToTfTransform::Wgs84ToTfTransform(
    const tf::Transform& transform,
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util) :
    transform_(transform),
    local_xy_util_(local_xy_util)
  {
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
}

