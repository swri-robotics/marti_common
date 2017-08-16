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
#ifndef SWRI_ROUTE_UTIL_ROUTE_SPEEDS_H_
#define SWRI_ROUTE_UTIL_ROUTE_SPEEDS_H_

#include <marti_common_msgs/KeyValueArray.h>
#include <marti_nav_msgs/ObstacleArray.h>
#include <marti_nav_msgs/RouteSpeedArray.h>
#include <swri_math_util/interpolation_1d.h>
#include <swri_route_util/route.h>
#include <swri_transform_util/transform.h>

namespace swri_route_util
{
struct SpeedForCurvatureParameters
{
  /// If true, use maximum lateral acceleration constant to calculate
  /// maximum speed.  Otherwise, the curvature vs speed curve will be
  /// used.
  bool use_speed_from_accel_constant_;
  /// Maximum lateral acceleration in accel mode in m/s^2
  double max_lateral_accel_mss_;

  /// Speed as a function of curvature, applies when
  /// use_speed_from_accel_constant is false
  swri_math_util::Interpolation1D speed_curve_;

  // Filter constant used when estimating route curvature.  Larger
  // values result in smoother curvature estimates with fewer spikes.
  double curvature_filter_size_;


  SpeedForCurvatureParameters();

  void loadFromRosParam(const ros::NodeHandle &pnh);

  void loadFromConfig(const marti_common_msgs::KeyValueArray &config);
  void readToConfig(marti_common_msgs::KeyValueArray &config) const;
};

void speedsForCurvature(
  marti_nav_msgs::RouteSpeedArray &speeds,
  const Route &route,
  const SpeedForCurvatureParameters &parameters);


struct SpeedForObstaclesParameters
{
  double origin_to_front_m_;
  double origin_to_rear_m_;
  double origin_to_left_m_;
  double origin_to_right_m_;

  double max_distance_m_;
  double min_distance_m_;
  double max_speed_;
  double min_speed_;

  double stop_buffer_m_;

  SpeedForObstaclesParameters();

  void loadFromRosParam(const ros::NodeHandle &pnh);
};

// ObstacleData is an intermediate representation for obstacles to
// avoid applying transforms and calculating radii repeatedly.
struct ObstacleData
{
  tf::Vector3 center;
  double radius;

  std::vector<tf::Vector3> polygon;
};


struct DistanceReport
{
  // True if the bounding circles touch but the actual polygons do not.
  bool near;
  // True if the actual polygons touch.
  bool collision;
  size_t route_index;
  tf::Vector3 vehicle_point;
  tf::Vector3 obstacle_point;
  double distance;
};

// Convert an obstacle array message into a ObstacleData by applying a
// transform and calculating the radius of each obstacle.
void generateObstacleData(
  std::vector<ObstacleData> &obstacle_data,
  const swri_transform_util::Transform g_route_from_obs,
  const marti_nav_msgs::ObstacleArray &obstacles_msg);

void speedsForObstacles(
    marti_nav_msgs::RouteSpeedArray &speeds,
    std::vector<DistanceReport> &reports,
    const Route &route,
    const marti_nav_msgs::RoutePosition &route_position,
    const std::vector<ObstacleData> &obstacles,
    const SpeedForObstaclesParameters &parameters);
}  // namespace swri_route_util
#endif  // SWRI_ROUTE_UTIL_ROUTE_SPEEDS_H_
