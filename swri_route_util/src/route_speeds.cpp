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
#include <swri_route_util/route_speeds.h>

#include <unordered_map>
#include <swri_roscpp/swri_roscpp.h>
#include <swri_route_util/util.h>

namespace smu = swri_math_util;
namespace mcm = marti_common_msgs;
namespace mnm = marti_nav_msgs;

namespace swri_route_util
{
// Convenience function to add a key/value pair to a KeyValueArray message.
static void addItem(mcm::KeyValueArray &m, const std::string &key, const std::string &value)
{
  m.items.emplace_back();
  m.items.back().key = key;
  m.items.back().value = value;
}

SpeedForCurvatureParameters::SpeedForCurvatureParameters()
  :
  use_speed_from_accel_constant_(true),
  max_lateral_accel_mss_(0.2),
  curvature_filter_size_(20.0)
{
}

void SpeedForCurvatureParameters::loadFromRosParam(const ros::NodeHandle &pnh)
{
  // 20.0 seems to be a good value from looking through 17 recorded routes we
  // have with and without Omnistar corrections.
  swri::param(pnh, "curvature_filter_size", curvature_filter_size_, 20.0);


  swri::param(pnh, "lateral_acceleration_mode",
              use_speed_from_accel_constant_, use_speed_from_accel_constant_);
  swri::param(pnh, "max_lateral_acceleration",
              max_lateral_accel_mss_, max_lateral_accel_mss_);

  if (!speed_curve_.readFromParameter(pnh, "curvature_vs_speed", true)) {
    ROS_ERROR("Failed to load speed/curve parameter. Forcing lateral acceleration mode.");
    use_speed_from_accel_constant_ = true;
  } else {
    ROS_INFO("Loaded speed vs curvature curve (%s)", speed_curve_.interpolationTypeString().c_str());
    for (size_t i = 0; i < speed_curve_.numPoints(); i++) {
      std::pair<double, double> pt = speed_curve_.getPoint(i);
      ROS_INFO("  %zu -- %f [1/m] vs %f [m/s]", i, pt.first, pt.second);
    }
  }
}

void SpeedForCurvatureParameters::loadFromConfig(const mcm::KeyValueArray &config)
{
  std::unordered_map<std::string, std::string> config_map;
  for (size_t i = 0; i < config.items.size(); ++i) {
    config_map[config.items[i].key] = config.items[i].value;
  }

  if (config_map.count("curvature_filter_size")) {
    curvature_filter_size_ = boost::lexical_cast<double>(config_map.at("curvature_filter_size"));
    ROS_INFO("Setting curvature_filter_size to %lf", curvature_filter_size_);
    config_map.erase("curvature_filter_size");
  }

  if (config_map.count("lateral_acceleration_mode")) {
    use_speed_from_accel_constant_ =
      boost::lexical_cast<int>(config_map.at("lateral_acceleration_mode"));
    ROS_INFO("Setting lateral acceleration mode to %s",
             use_speed_from_accel_constant_ ? "true" : "false");
    config_map.erase("lateral_acceleration_mode");
  }

  if (config_map.count("max_lateral_acceleration")) {
    max_lateral_accel_mss_ = boost::lexical_cast<double>(config_map.at("max_lateral_acceleration"));
    ROS_INFO("Setting max_lateral_acceleration to %lf", max_lateral_accel_mss_);
    config_map.erase("max_lateral_acceleration");
  }

  if (config_map.count("curvature_vs_speed/interpolation_type")) {
    std::string interp_type = config_map.at("curvature_vs_speed/interpolation_type");
    if (interp_type == "zero_order_hold") {
      speed_curve_.setInterpolationType(smu::Interpolation1D::ZERO_ORDER_HOLD);
      ROS_INFO("Setting interpolation type to %s", interp_type.c_str());
    } else if (interp_type == "linear") {
      speed_curve_.setInterpolationType(smu::Interpolation1D::LINEAR);
      ROS_INFO("Setting interpolation type to %s", interp_type.c_str());
    } else {
      ROS_ERROR("Ignoring invalid interpolation type '%s'.", interp_type.c_str());
    }
    config_map.erase("curvature_vs_speed/interpolation_type");
  }

  // We read in the curve points by determining if point N exists in the
  // config set, starting with N = 0 and going until the first pair of points
  // fails.  We don't set the points in the curve because we want to make sure
  // that a new curve is actually specified before deleting the old curve.
  std::vector<double> x;
  std::vector<double> y;
  for (size_t i = 0; true; i++) {
    char base_key[1024];
    snprintf(base_key, sizeof(base_key), "curvature_vs_speed/values/%zu", i);
    const std::string x_key = std::string(base_key) + "/0";
    const std::string y_key = std::string(base_key) + "/1";

    if (config_map.count(x_key) && config_map.count(y_key)) {
      x.push_back(boost::lexical_cast<double>(config_map.at(x_key)));
      config_map.erase(x_key);
      y.push_back(boost::lexical_cast<double>(config_map.at(y_key)));
      config_map.erase(y_key);
    } else {
      break;
    }
  }

  // If a curve was provided, setup speed_curve using the new parameters.
  // Note that from the way x,y are constructed, they are guaranteed to have
  // the same length.
  if (x.size()) {
    ROS_INFO("Setting curvature_vs_speed curve: ");
    speed_curve_.clear();
    for (size_t i = 0; i < x.size(); i++) {
      speed_curve_.appendPoint(x[i], y[i]);
      ROS_INFO("  %zu -- %lf, %lf", i, x[i], y[i]);
    }
  }

  // Print warnings to help find ignored paramters
  for (auto const &it : config_map) {
    ROS_WARN("Ignoring unknown configuration value '%s'", it.first.c_str());
  }
}

void SpeedForCurvatureParameters::readToConfig(mcm::KeyValueArray &config) const
{
  mcm::KeyValueArrayPtr msg = boost::make_shared<mcm::KeyValueArray>();
  config.header.stamp = ros::Time::now();

  addItem(config, "curvature_filter_size",
          boost::lexical_cast<std::string>(curvature_filter_size_));
  addItem(config, "lateral_acceleration_mode",
          use_speed_from_accel_constant_ ? "1" : "0");
  addItem(config, "max_lateral_acceleration",
          boost::lexical_cast<std::string>(max_lateral_accel_mss_));
  addItem(config, "curvature_vs_speed/interpolation_type",
          speed_curve_.interpolationTypeString());

  for (size_t i = 0; i < speed_curve_.numPoints(); i++) {
    char base_key[1024];
    snprintf(base_key, sizeof(base_key), "curvature_vs_speed/values/%zu", i);

    addItem(config, std::string(base_key) + "/0",
            boost::lexical_cast<std::string>(speed_curve_.getPoint(i).first));
    addItem(config, std::string(base_key) + "/1",
            boost::lexical_cast<std::string>(speed_curve_.getPoint(i).second));
  }
}

static double estimateCurvature(const Route &route,
                                const size_t index,
                                double filter_size)
{
  // Sample the route at the specified point
  mnm::RoutePosition pos1;
  pos1.id = route.points[index].id();
  pos1.distance = 0.0;

  // Sample the route one filter size backwards.
  mnm::RoutePosition pos0 = pos1;
  pos0.distance -= filter_size;

  // Sample the route one filter size forwards.
  mnm::RoutePosition pos2 = pos1;
  pos2.distance += filter_size;

  // Get the actual route samples
  RoutePoint pt0;
  interpolateRoutePosition(pt0, route, pos0, true);
  RoutePoint pt1;
  interpolateRoutePosition(pt1, route, pos1, true);
  RoutePoint pt2;
  interpolateRoutePosition(pt2, route, pos2, true);

  // Approximate the incoming tangent vector
  const tf::Vector3 T0 = (pt1.position() - pt0.position()).normalized();
  // Approximate the outgoing tangent vector
  const tf::Vector3 T1 = (pt2.position() - pt1.position()).normalized();
  // k ~ ||dT / ds||
  return ((T1-T0)/filter_size).length();
}

static double maxSpeedForCurvature(double curvature,
                                   const SpeedForCurvatureParameters &params)
{
  double k = std::abs(curvature);

  if (params.use_speed_from_accel_constant_) {
    double a = std::abs(params.max_lateral_accel_mss_);

    double max_speed = 1000.0;
    
    if (k < 1e-4) {
      return max_speed;
    } else {
      return std::min(max_speed, std::sqrt(a/k));
    }
  } else {
    return params.speed_curve_.eval(k);
  }
}

void speedsForCurvature(
  mnm::RouteSpeedArray &speeds,
  const Route &route,
  const SpeedForCurvatureParameters &parameters)
{
  speeds.header.stamp = ros::Time::now();

  // We're going to generate a speed for every route point for now.
  speeds.speeds.resize(route.points.size());

  for (size_t i = 0; i < route.points.size(); ++i) {
    speeds.speeds[i].id = route.points[i].id();
    speeds.speeds[i].distance = 0.0;

    double k = estimateCurvature(route, i, parameters.curvature_filter_size_);
    speeds.speeds[i].speed = maxSpeedForCurvature(k, parameters);
  }  
}

SpeedForObstaclesParameters::SpeedForObstaclesParameters()
{
}

void SpeedForObstaclesParameters::loadFromRosParam(const ros::NodeHandle &pnh)
{
}

void SpeedForObstaclesParameters::loadFromConfig(const mcm::KeyValueArray &config)
{
}

void SpeedForObstaclesParameters::readToConfig(mcm::KeyValueArray &config) const
{
}

void speedsForObstacles(
  mnm::RouteSpeedArray &speeds,
  const Route &route,
  const mnm::ObstacleArray &obstacles,
  const SpeedForObstaclesParameters &parameters)
{
}
}  // namespace swri_route_util

