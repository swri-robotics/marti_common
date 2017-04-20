#include <string>
#include <ros/ros.h>
#include <swri_roscpp/parameters.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "param_test");
  ros::NodeHandle pnh("~");
  
  int int_var;
  swri::getParam(pnh, "used_int_var_1", int_var);
  swri::param(pnh, "used_int_var_2", int_var, 2);
  swri::param(pnh, "unset_int_var", int_var, 2);
  
  std::string str_var;
  swri::getParam(pnh, "used_str_var_1", str_var);
  swri::param(pnh, "used_str_var_2", str_var, "foo");
  swri::param(pnh, "unset_str_var", str_var, "foo");
  
  double dbl_var;
  swri::getParam(pnh, "used_dbl_var_1", dbl_var);
  swri::param(pnh, "used_dbl_var_2", dbl_var, 2.0);
  swri::param(pnh, "unset_dbl_var", dbl_var, 2.0);
  
  bool bool_var;
  swri::getParam(pnh, "used_bool_var_1", bool_var);
  swri::param(pnh, "used_bool_var_2", bool_var, true);
  swri::param(pnh, "unset_bool_var", bool_var, true);
  
  swri::warnUnusedParams(pnh);

  return 0;
}
