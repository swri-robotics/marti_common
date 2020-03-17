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

#include <swri_transform_util/local_xy_util.h>

#include <cmath>
#include <functional>

#include <tf2/utils.h>

#include <swri_math_util/constants.h>
#include <swri_math_util/trig_util.h>
#include <swri_transform_util/earth_constants.h>
#include <swri_transform_util/transform_util.h>

namespace swri_transform_util
{
  void LocalXyFromWgs84(
      double latitude,
      double longitude,
      double reference_latitude,
      double reference_longitude,
      double& x,
      double& y)
  {
    LocalXyWgs84Util local_xy(reference_latitude, reference_longitude);
    local_xy.ToLocalXy(latitude, longitude, x, y);
  }

  void Wgs84FromLocalXy(
      double x,
      double y,
      double reference_latitude,
      double reference_longitude,
      double& latitude,
      double& longitude)
  {
    LocalXyWgs84Util local_xy(reference_latitude, reference_longitude);
    local_xy.ToWgs84(x, y, latitude, longitude);
  }

  LocalXyWgs84Util::LocalXyWgs84Util(
      double reference_latitude,
      double reference_longitude,
      double reference_angle,
      double reference_altitude,
      rclcpp::Node::SharedPtr node) :
    node_(node),
    reference_latitude_(reference_latitude * swri_math_util::_deg_2_rad),
    reference_longitude_(reference_longitude * swri_math_util::_deg_2_rad),
    reference_angle_(reference_angle * swri_math_util::_deg_2_rad),
    reference_altitude_(reference_altitude),
    rho_lat_(0),
    rho_lon_(0),
    cos_angle_(0),
    sin_angle_(0),
    frame_("map"),
    initialized_(false)
  {
    Initialize();
  }

  LocalXyWgs84Util::LocalXyWgs84Util(rclcpp::Node::SharedPtr node) :
    node_(node),
    reference_latitude_(0),
    reference_longitude_(0),
    reference_angle_(0),
    reference_altitude_(0),
    rho_lat_(0),
    rho_lon_(0),
    cos_angle_(0),
    sin_angle_(0),
    frame_("map"),
    initialized_(false)
  {
    RCLCPP_INFO(node->get_logger(), "Subscribing to /local_xy_origin");

    ResetInitialization();
  }

  void LocalXyWgs84Util::ResetInitialization()
  {
    std::string type;
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/local_xy_origin",
        1,
        std::bind(&LocalXyWgs84Util::HandlePoseStamped, this, std::placeholders::_1));
    initialized_ = false;
  }

  void LocalXyWgs84Util::Initialize()
  {
    reference_angle_ = swri_math_util::WrapRadians(reference_angle_, 0);

    cos_angle_ = std::cos(reference_angle_);
    sin_angle_ = std::sin(reference_angle_);

    double depth = -reference_altitude_;

    double p = _earth_eccentricity * std::sin(reference_latitude_);
    p = 1.0 - p * p;

    double rho_e = _earth_equator_radius *
        (1.0 - _earth_eccentricity * _earth_eccentricity) / (std::sqrt(p) * p);
    double rho_n = _earth_equator_radius / std::sqrt(p);

    rho_lat_ = rho_e - depth;
    rho_lon_ = (rho_n - depth) * std::cos(reference_latitude_);
    initialized_ = true;
  }

  void LocalXyWgs84Util::HandlePoseStamped(const geometry_msgs::msg::PoseStamped::UniquePtr pose)
  {
    HandleOrigin(pose->pose.position.y,
        pose->pose.position.x,
        pose->pose.position.z,
        tf2::getYaw(pose->pose.orientation),
        pose->header.frame_id);
  }

  void LocalXyWgs84Util::HandleOrigin(double latitude, double longitude, double altitude, double angle, const std::string& frame_id)
  {
    if (!initialized_)
    {
      bool ignore_reference_angle = false;
      node_->get_parameter_or("/local_xy_ignore_reference_angle", ignore_reference_angle, ignore_reference_angle);

      reference_latitude_ = latitude * swri_math_util::_deg_2_rad;
      reference_longitude_ = longitude * swri_math_util::_deg_2_rad;
      reference_altitude_ = altitude;

      if (!ignore_reference_angle)
      {
        reference_angle_ = angle;
      }

      std::string frame = frame_id;

      if (frame.empty())
      {
        // If the origin has an empty frame id, look for a frame in
        // the global parameter /local_xy_frame.  This provides
        // compatibility with older bag files.
        node_->get_parameter_or("/local_xy_frame", frame, frame_);
      }

      frame_ = frame;

      Initialize();
      pose_sub_.reset();
      return;
    }
  }

  double LocalXyWgs84Util::ReferenceLongitude() const
  {
    return reference_longitude_ * swri_math_util::_rad_2_deg;
  }

  double LocalXyWgs84Util::ReferenceLatitude() const
  {
    return reference_latitude_ * swri_math_util::_rad_2_deg;
  }

  double LocalXyWgs84Util::ReferenceAngle() const
  {
    return reference_angle_ * swri_math_util::_rad_2_deg;
  }

  double LocalXyWgs84Util::ReferenceAltitude() const
  {
    return reference_altitude_;
  }

  bool LocalXyWgs84Util::ToLocalXy(
      double latitude,
      double longitude,
      double& x,
      double& y) const
  {
    if (initialized_)
    {
      if (latitude < -90.0 || latitude > 90.0)
      {
        return false;
      }

      if (longitude < -180.0 || longitude > 180.0)
      {
        return false;
      }

      double rlat = latitude * swri_math_util::_deg_2_rad;
      double rlon = longitude * swri_math_util::_deg_2_rad;
      double dLat = (rlat - reference_latitude_) * rho_lat_;
      double dLon = (rlon - reference_longitude_) * rho_lon_;

      x =  cos_angle_ * dLon + sin_angle_ * dLat;
      y = -sin_angle_ * dLon + cos_angle_ * dLat;

      return true;
    }

    return initialized_;
  }

  bool LocalXyWgs84Util::ToWgs84(
      double x,
      double y,
      double& latitude,
      double& longitude) const
  {
    if (initialized_)
    {
      double dLon = cos_angle_ * x - sin_angle_ * y;
      double dLat = sin_angle_ * x + cos_angle_ * y;
      double rlat = (dLat / rho_lat_) + reference_latitude_;
      double rlon = (dLon / rho_lon_) + reference_longitude_;

      latitude = rlat * swri_math_util::_rad_2_deg;
      longitude = rlon * swri_math_util::_rad_2_deg;
    }

    return initialized_;
  }
}
