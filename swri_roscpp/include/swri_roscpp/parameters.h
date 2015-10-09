#ifndef SWRI_ROSCPP_PARAMETERS_H_
#define SWRI_ROSCPP_PARAMETERS_H_

#include <ros/node_handle.h>

namespace swri
{
static
void param(const ros::NodeHandle &nh,
           const std::string &name,
           int &variable,
           const int default_value)
{
  nh.param(name, variable, default_value);
  ROS_INFO("Read parameter %s = %d", name.c_str(), variable);
}

static
void param(const ros::NodeHandle &nh,
           const std::string &name,
           double &variable,
           const double default_value)
{
  nh.param(name, variable, default_value);
  ROS_INFO("Read parameter %s = %f", name.c_str(), variable);
}

static
void param(const ros::NodeHandle &nh,
           const std::string &name,
           std::string &variable,
           const std::string default_value)
{
  nh.param(name, variable, default_value);
  ROS_INFO("Read parameter %s = \"%s\"", name.c_str(), variable.c_str());
}

static
void param(const ros::NodeHandle &nh,
           const std::string &name,
           bool &variable,
           const bool default_value)
{
  nh.param(name, variable, default_value);
  ROS_INFO("Read parameter %s = %s", name.c_str(), variable ? "true" : "false");
}
}  // namespace swri_param
#endif  // SWRI_ROSCPP_PARAMETERS_H_
