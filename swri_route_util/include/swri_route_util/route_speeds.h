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
  // values result in smoother curvature estiamtes with fewer spikes.
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
  SpeedForObstaclesParameters();
  
  void loadFromRosParam(const ros::NodeHandle &pnh);

  void loadFromConfig(const marti_common_msgs::KeyValueArray &config);
  void readToConfig(marti_common_msgs::KeyValueArray &config) const;
};

void speedsForObstacles(
    marti_nav_msgs::RouteSpeedArray &speeds,
    const Route &route,
    const marti_nav_msgs::ObstacleArray &obstacles,
    const SpeedForObstaclesParameters &parameters);
}  // namespace swri_route_util
#endif  // SWRI_ROUTE_UTIL_ROUTE_SPEEDS_H_
