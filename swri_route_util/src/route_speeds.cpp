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

#include <swri_geometry_util/geometry_util.h>
#include <swri_roscpp/swri_roscpp.h>
#include <swri_route_util/util.h>

namespace smu = swri_math_util;
namespace stu = swri_transform_util;
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
  :
  origin_to_front_m_(2.0),
  origin_to_rear_m_(1.0),
  origin_to_left_m_(1.0),
  origin_to_right_m_(1.0),
  max_distance_m_(10.0),
  min_distance_m_(1.0),
  max_speed_(10.0),
  min_speed_(1.0),
  stop_buffer_m_(5.0)
{
}

void SpeedForObstaclesParameters::loadFromRosParam(const ros::NodeHandle &pnh)
{
  swri::param(pnh, "origin_to_front_m", origin_to_front_m_, 2.0);
  swri::param(pnh, "origin_to_rear_m", origin_to_rear_m_, 1.0);
  swri::param(pnh, "origin_to_left_m", origin_to_left_m_, 1.0);
  swri::param(pnh, "origin_to_right_m", origin_to_right_m_, 1.0);

  swri::param(pnh, "max_distance_m", max_distance_m_, 10.0);
  swri::param(pnh, "min_distance_m", min_distance_m_, 1.0);
  swri::param(pnh, "max_speed", max_speed_, 10.0);
  swri::param(pnh, "min_speed", min_speed_, 1.0);

  swri::param(pnh, "stop_buffer_m", stop_buffer_m_, 5.0);
}

void generateObstacleData(
  std::vector<ObstacleData> &obstacle_data,
  const stu::Transform g_route_from_obs,
  const mnm::ObstacleArray &obstacles_msg)
{
  obstacle_data.resize(obstacles_msg.obstacles.size());
  for (size_t i = 0; i < obstacle_data.size(); i++) {
    const mnm::Obstacle &obs_msg = obstacles_msg.obstacles[i];

    geometry_msgs::Pose pose = obs_msg.pose;
    if (pose.orientation.x == 0.0 &&
        pose.orientation.y == 0.0 &&
        pose.orientation.z == 0.0 &&
        pose.orientation.w == 0.0) {
      pose.orientation.w = 1.0;
    }

    tf::Transform g_obs_from_local;
    tf::poseMsgToTF(pose, g_obs_from_local);

    obstacle_data[i].center = g_route_from_obs*g_obs_from_local.getOrigin();
    obstacle_data[i].center.setZ(0.0);

    double max_radius = 0.0;
    obstacle_data[i].polygon.resize(obs_msg.polygon.size());
    for (size_t j = 0; j < obs_msg.polygon.size(); j++) {
      tf::Vector3 pt;
      tf::pointMsgToTF(obs_msg.polygon[j], pt);

      max_radius = std::max(max_radius, pt.length());
      obstacle_data[i].polygon[j] = g_route_from_obs*(g_obs_from_local*pt);
      obstacle_data[i].polygon[j].setZ(0.0);
    }
    obstacle_data[i].radius = max_radius;
  }
}

void speedsForObstacles(
  mnm::RouteSpeedArray &speeds,
  std::vector<DistanceReport> &reports,
  const Route &route,
  const mnm::RoutePosition &route_position,
  const std::vector<ObstacleData> &obstacles,
  const SpeedForObstaclesParameters &p)
{
  tf::Vector3 local_fl(p.origin_to_left_m_, p.origin_to_left_m_, 0.0);
  tf::Vector3 local_fr(p.origin_to_left_m_, -p.origin_to_right_m_, 0.0);
  tf::Vector3 local_br(-p.origin_to_right_m_, -p.origin_to_right_m_, 0.0);
  tf::Vector3 local_bl(-p.origin_to_right_m_, p.origin_to_left_m_, 0.0);
  double car_r = 0.0;
  car_r = std::max(car_r, local_fl.length());
  car_r = std::max(car_r, local_fr.length());
  car_r = std::max(car_r, local_br.length());
  car_r = std::max(car_r, local_bl.length());

  bool skip_point = true;

  for (size_t route_index = 0; route_index < route.points.size(); route_index++) {
    const RoutePoint &point = route.points[route_index];

    if (skip_point) {
      if (point.id() == route_position.id) {
        skip_point = false;
      }
      continue;
    }
    
    // Use half of the vehicle width override as the car radius when smaller
    // This is used for tight routes that are known to be safe such as when going
    // though cones. It prevents objects close to the vehicle from causing the vehicle
    // to stop/slow down too much unnecessarily.
    double veh_r = car_r;
    if (point.hasProperty("vehicle_width_override"))
    {
      ROS_DEBUG("Speeds for obstacle found vehicle_width_override property");
      double width = point.getTypedProperty<double>("vehicle_width_override");

      // Pick the smaller of the radii
      if (veh_r >= width/2.0)
      {
        veh_r = width/2.0;
        ROS_WARN_THROTTLE(1.0, "Vehicle width being overriden to %0.2f", (float)veh_r);
      }
    }

    for (const auto& obstacle: obstacles) {
      const tf::Vector3 v = obstacle.center - point.position();
      const double d = v.length() - veh_r - obstacle.radius;
      if (d > p.max_distance_m_) {
        // The obstacle is too far away from this point to be a concern
        continue;
      }

      tf::Vector3 closest_point = obstacle.center;

      double distance = std::numeric_limits<double>::max();
      for (size_t i = 1; i < obstacle.polygon.size(); i++) {
        double dist = swri_geometry_util::DistanceFromLineSegment(
          obstacle.polygon[i - 1],
          obstacle.polygon[i],
          point.position()) - veh_r;

        if (dist < distance) {
          distance = dist;
          closest_point = swri_geometry_util::ProjectToLineSegment(
            obstacle.polygon[i - 1],
            obstacle.polygon[i],
            point.position());
        }
      }

      if (obstacle.polygon.size() > 1) {
        double dist = swri_geometry_util::DistanceFromLineSegment(
          obstacle.polygon.back(),
          obstacle.polygon.front(),
          point.position()) - veh_r;

        if (dist < distance) {
          distance = dist;
          closest_point = swri_geometry_util::ProjectToLineSegment(
            obstacle.polygon.back(),
            obstacle.polygon.front(),
            point.position());
        }
      }

      if (distance > p.max_distance_m_) {
        // The obstacle is too far away from this point to be a concern
        continue;
      }

      // This obstacle is close enough to be a concern.  If the bounding
      // circles are still not touching, we apply a speed limit based on the
      // distance.
      if (distance > 0.0) {
        DistanceReport report;
        report.near = false;
        report.collision = false;
        report.route_index = route_index;
        report.vehicle_point = point.position() + (closest_point - point.position()).normalized() * veh_r;
        report.obstacle_point = closest_point;
        reports.push_back(report);

        const double s = std::max(0.0, (distance - p.min_distance_m_) / (
                                    p.max_distance_m_ - p.min_distance_m_));
        double speed = (1.0-s)*p.min_speed_ + s*p.max_speed_;

        speeds.speeds.emplace_back();
        speeds.speeds.back().id = point.id();
        speeds.speeds.back().distance = 0.0;
        speeds.speeds.back().speed = speed;

        continue;
      }

      DistanceReport report;
      report.near = false;
      report.collision = true;
      report.route_index = route_index;
      report.vehicle_point = point.position();
      report.obstacle_point = closest_point;
      reports.push_back(report);

      speeds.speeds.emplace_back();
      speeds.speeds.back().id = point.id();
      speeds.speeds.back().distance = -p.stop_buffer_m_ - p.origin_to_front_m_;
      speeds.speeds.back().speed = 0.0;

      speeds.speeds.emplace_back();
      speeds.speeds.back().id = point.id();
      speeds.speeds.back().distance = (-p.stop_buffer_m_ - p.origin_to_front_m_) / 2.0;
      speeds.speeds.back().speed = 0.0;

      speeds.speeds.emplace_back();
      speeds.speeds.back().id = point.id();
      speeds.speeds.back().distance = 0.0;
      speeds.speeds.back().speed = 0.0;

      continue;
    }
  }
}
}  // namespace swri_route_util
