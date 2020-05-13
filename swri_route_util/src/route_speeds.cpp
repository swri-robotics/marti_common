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
#include <swri_route_util/util.h>

namespace smu = swri_math_util;
namespace stu = swri_transform_util;
namespace mcm = marti_common_msgs;
namespace mnm = marti_nav_msgs;

namespace swri_route_util
{
// Convenience function to add a key/value pair to a KeyValueArray message.
static void addItem(mcm::msg::KeyValueArray &m, const std::string &key, const std::string &value)
{
  m.items.emplace_back();
  m.items.back().key = key;
  m.items.back().value = value;
}

SpeedForCurvatureParameters::SpeedForCurvatureParameters(const rclcpp::Node::SharedPtr& node)
  :
  node_(node),
  use_speed_from_accel_constant_(true),
  max_lateral_accel_mss_(0.2),
  speed_curve_(*node_),
  curvature_filter_size_(20.0)
{
}

void SpeedForCurvatureParameters::loadFromRosParam()
{
  // 20.0 seems to be a good value from looking through 17 recorded routes we
  // have with and without Omnistar corrections.
  curvature_filter_size_ = node_->declare_parameter("curvature_filter_size", 20.0);
  use_speed_from_accel_constant_ = node_->declare_parameter("lateral_acceleration_mode", true);
  max_lateral_accel_mss_ = node_->declare_parameter("max_lateral_acceleration", 0.2);

  if (!speed_curve_.readFromParameter("curvature_vs_speed")) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load speed/curve parameter. Forcing lateral acceleration mode.");
    use_speed_from_accel_constant_ = true;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Loaded speed vs curvature curve (%s)", speed_curve_.interpolationTypeString().c_str());
    for (size_t i = 0; i < speed_curve_.numPoints(); i++) {
      std::pair<double, double> pt = speed_curve_.getPoint(i);
      RCLCPP_INFO(node_->get_logger(), "  %zu -- %f [1/m] vs %f [m/s]", i, pt.first, pt.second);
    }
  }
}

void SpeedForCurvatureParameters::loadFromConfig(const mcm::msg::KeyValueArray &config)
{
  std::unordered_map<std::string, std::string> config_map;
  for (size_t i = 0; i < config.items.size(); ++i) {
    config_map[config.items[i].key] = config.items[i].value;
  }

  if (config_map.count("curvature_filter_size")) {
    curvature_filter_size_ = boost::lexical_cast<double>(config_map.at("curvature_filter_size"));
    RCLCPP_INFO(node_->get_logger(), "Setting curvature_filter_size to %lf", curvature_filter_size_);
    config_map.erase("curvature_filter_size");
  }

  if (config_map.count("lateral_acceleration_mode")) {
    use_speed_from_accel_constant_ =
      boost::lexical_cast<int>(config_map.at("lateral_acceleration_mode"));
    RCLCPP_INFO(node_->get_logger(), "Setting lateral acceleration mode to %s",
             use_speed_from_accel_constant_ ? "true" : "false");
    config_map.erase("lateral_acceleration_mode");
  }

  if (config_map.count("max_lateral_acceleration")) {
    max_lateral_accel_mss_ = boost::lexical_cast<double>(config_map.at("max_lateral_acceleration"));
    RCLCPP_INFO(node_->get_logger(), "Setting max_lateral_acceleration to %lf", max_lateral_accel_mss_);
    config_map.erase("max_lateral_acceleration");
  }

  if (config_map.count("curvature_vs_speed/interpolation_type")) {
    std::string interp_type = config_map.at("curvature_vs_speed/interpolation_type");
    if (interp_type == "zero_order_hold") {
      speed_curve_.setInterpolationType(smu::Interpolation1D::ZERO_ORDER_HOLD);
      RCLCPP_INFO(node_->get_logger(), "Setting interpolation type to %s", interp_type.c_str());
    } else if (interp_type == "linear") {
      speed_curve_.setInterpolationType(smu::Interpolation1D::LINEAR);
      RCLCPP_INFO(node_->get_logger(), "Setting interpolation type to %s", interp_type.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Ignoring invalid interpolation type '%s'.", interp_type.c_str());
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
    RCLCPP_INFO(node_->get_logger(), "Setting curvature_vs_speed curve: ");
    speed_curve_.clear();
    for (size_t i = 0; i < x.size(); i++) {
      speed_curve_.appendPoint(x[i], y[i]);
      RCLCPP_INFO(node_->get_logger(), "  %zu -- %lf, %lf", i, x[i], y[i]);
    }
  }

  // Print warnings to help find ignored paramters
  for (auto const &it : config_map) {
    RCLCPP_WARN(node_->get_logger(), "Ignoring unknown configuration value '%s'", it.first.c_str());
  }
}

void SpeedForCurvatureParameters::readToConfig(mcm::msg::KeyValueArray &config) const
{
  mcm::msg::KeyValueArray::SharedPtr msg = std::make_shared<mcm::msg::KeyValueArray>();
  config.header.stamp = rclcpp::Clock().now();

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
  mnm::msg::RoutePosition pos1;
  pos1.id = route.points[index].id();
  pos1.distance = 0.0;

  // Sample the route one filter size backwards.
  mnm::msg::RoutePosition pos0 = pos1;
  pos0.distance -= filter_size;

  // Sample the route one filter size forwards.
  mnm::msg::RoutePosition pos2 = pos1;
  pos2.distance += filter_size;

  // Get the actual route samples
  RoutePoint pt0;
  interpolateRoutePosition(pt0, route, pos0, true);
  RoutePoint pt1;
  interpolateRoutePosition(pt1, route, pos1, true);
  RoutePoint pt2;
  interpolateRoutePosition(pt2, route, pos2, true);

  // Approximate the incoming tangent vector
  const tf2::Vector3 T0 = (pt1.position() - pt0.position()).normalized();
  // Approximate the outgoing tangent vector
  const tf2::Vector3 T1 = (pt2.position() - pt1.position()).normalized();
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
  mnm::msg::RouteSpeedArray &speeds,
  const Route &route,
  const SpeedForCurvatureParameters &parameters)
{
  speeds.header.stamp = rclcpp::Clock().now();

  // We're going to generate a speed for every route point for now.
  speeds.speeds.resize(route.points.size());

  for (size_t i = 0; i < route.points.size(); ++i) {
    speeds.speeds[i].id = route.points[i].id();
    speeds.speeds[i].distance = 0.0;

    double k = estimateCurvature(route, i, parameters.curvature_filter_size_);
    speeds.speeds[i].speed = maxSpeedForCurvature(k, parameters);
  }
}

SpeedForObstaclesParameters::SpeedForObstaclesParameters(const rclcpp::Node::SharedPtr& node)
  :
  node_(node),
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

void SpeedForObstaclesParameters::loadFromRosParam()
{
  origin_to_front_m_ = node_->declare_parameter("origin_to_front_m", 0);
  origin_to_rear_m_ = node_->declare_parameter("origin_to_rear_m", 0);
  origin_to_left_m_ = node_->declare_parameter("origin_to_left_m", 0);
  origin_to_right_m_ = node_->declare_parameter("origin_to_right_m", 0);

  max_distance_m_ = node_->declare_parameter("max_distance_m", 0);
  min_distance_m_ = node_->declare_parameter("min_distance_m", 0);
  max_speed_ = node_->declare_parameter("max_speed", 0);
  min_speed_ = node_->declare_parameter("min_speed", 0);

  stop_buffer_m_ = node_->declare_parameter("stop_buffer_m", 0);
}

void generateObstacleData(
  std::vector<ObstacleData>& obstacle_data,
  const stu::Transform& g_route_from_obs,
  const mnm::msg::ObstacleArray& obstacles_msg)
{
  obstacle_data.resize(obstacles_msg.obstacles.size());
  for (size_t i = 0; i < obstacle_data.size(); i++) {
    const mnm::msg::Obstacle &obs_msg = obstacles_msg.obstacles[i];

    geometry_msgs::msg::Pose pose = obs_msg.pose;
    if (pose.orientation.x == 0.0 &&
        pose.orientation.y == 0.0 &&
        pose.orientation.z == 0.0 &&
        pose.orientation.w == 0.0) {
      pose.orientation.w = 1.0;
    }

    tf2::Transform g_obs_from_local;
    tf2::fromMsg(pose, g_obs_from_local);

    obstacle_data[i].center = g_route_from_obs*g_obs_from_local.getOrigin();
    obstacle_data[i].center.setZ(0.0);

    double max_radius = 0.0;
    obstacle_data[i].polygon.resize(obs_msg.polygon.size());
    for (size_t j = 0; j < obs_msg.polygon.size(); j++) {
      tf2::Vector3 pt;
      tf2::fromMsg(obs_msg.polygon[j], pt);

      max_radius = std::max(max_radius, pt.length());
      obstacle_data[i].polygon[j] = g_route_from_obs*(g_obs_from_local*pt);
      obstacle_data[i].polygon[j].setZ(0.0);
    }
    obstacle_data[i].radius = max_radius;
  }
}

void generateObstacleData(
  std::vector<ObstacleData>& obstacle_data,
  const stu::Transform& g_route_from_obs,
  const mnm::msg::TrackedObjectArray& obstacles_msg)
{
  obstacle_data.resize(obstacles_msg.objects.size());
  for (size_t i = 0; i < obstacle_data.size(); i++) {
    const mnm::msg::TrackedObject &obs_msg = obstacles_msg.objects[i];

    geometry_msgs::msg::Pose pose = obs_msg.pose.pose;
    if (pose.orientation.x == 0.0 &&
        pose.orientation.y == 0.0 &&
        pose.orientation.z == 0.0 &&
        pose.orientation.w == 0.0) {
      pose.orientation.w = 1.0;
    }

    tf2::Transform g_obs_from_local;
    tf2::fromMsg(pose, g_obs_from_local);

    obstacle_data[i].center = g_route_from_obs*g_obs_from_local.getOrigin();
    obstacle_data[i].center.setZ(0.0);

    double max_radius = 0.0;
    obstacle_data[i].polygon.resize(obs_msg.polygon.size());
    for (size_t j = 0; j < obs_msg.polygon.size(); j++) {
      tf2::Vector3 pt;
      tf2::fromMsg(obs_msg.polygon[j], pt);

      max_radius = std::max(max_radius, pt.length());
      obstacle_data[i].polygon[j] = g_route_from_obs*(g_obs_from_local*pt);
      obstacle_data[i].polygon[j].setZ(0.0);
    }
    obstacle_data[i].radius = max_radius;
  }
}

void speedsForObstacles(
  mnm::msg::RouteSpeedArray &speeds,
  std::vector<DistanceReport> &reports,
  const Route &route,
  const mnm::msg::RoutePosition &route_position,
  const std::vector<ObstacleData> &obstacles,
  const SpeedForObstaclesParameters &p,
  rclcpp::Logger logger)
{
  tf2::Vector3 local_fl(p.origin_to_left_m_, p.origin_to_left_m_, 0.0);
  tf2::Vector3 local_fr(p.origin_to_left_m_, -p.origin_to_right_m_, 0.0);
  tf2::Vector3 local_br(-p.origin_to_right_m_, -p.origin_to_right_m_, 0.0);
  tf2::Vector3 local_bl(-p.origin_to_right_m_, p.origin_to_left_m_, 0.0);
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
      RCLCPP_DEBUG(logger, "Speeds for obstacle found vehicle_width_override property");
      double width = point.getTypedProperty<double>("vehicle_width_override");

      // Pick the smaller of the radii
      if (veh_r >= width/2.0)
      {
        veh_r = width/2.0;
        RCLCPP_WARN(logger, "Vehicle width being overridden to %0.2f", (float)veh_r);
      }
    }

    for (const auto& obstacle: obstacles) {
      const tf2::Vector3 v = obstacle.center - point.position();
      const double d = v.length() - veh_r - obstacle.radius;
      if (d > p.max_distance_m_) {
        // The obstacle is too far away from this point to be a concern
        continue;
      }

      tf2::Vector3 closest_point = obstacle.center;

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
    }
  }
}

  DistanceReport::DistanceReport(bool near, bool collision, size_t routeIndex, const tf2::Vector3& vehiclePoint,
                                 const tf2::Vector3& obstaclePoint, double distance) : near(near), collision(collision),
                                                                                       route_index(routeIndex),
                                                                                       vehicle_point(vehiclePoint),
                                                                                       obstacle_point(obstaclePoint),
                                                                                       distance(distance)
  {}

  DistanceReport::DistanceReport()
  {}
}  // namespace swri_route_util
