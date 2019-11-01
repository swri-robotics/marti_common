#include <swri_roscpp/parameters.h>

#include <string>
#include <vector>
#include <gtest/gtest.h>

TEST(ParamTests, getUnusedParamKeys)
{
  rclcpp::Node node("param_test");
  std::string ns = node.get_namespace();
  
  int int_var;
  swri::getParam(node, "used_int_var_1", int_var);
  swri::param(node, "used_int_var_2", int_var, 2);
  swri::param(node, "unset_int_var", int_var, 2);
  
  std::string str_var;
  swri::getParam(node, "used_str_var_1", str_var);
  swri::param(node, "used_str_var_2", str_var, "foo");
  swri::param(node, "unset_str_var", str_var, "foo");
  
  double dbl_var;
  swri::getParam(node, "used_dbl_var_1", dbl_var);
  swri::param(node, "used_dbl_var_2", dbl_var, 2.0);
  swri::param(node, "unset_dbl_var", dbl_var, 2.0);
  
  bool bool_var;
  swri::getParam(node, "used_bool_var_1", bool_var);
  swri::param(node, "used_bool_var_2", bool_var, true);
  swri::param(node, "unset_bool_var", bool_var, true);
  
  /*std::vector<std::string> unused = swri::getUnusedParamKeys(node);
  ASSERT_EQ(4, unused.size());
  ASSERT_EQ(ns + "/unused_bool_var", unused[0]);
  ASSERT_EQ(ns + "/unused_dbl_var", unused[1]);
  ASSERT_EQ(ns + "/unused_int_var", unused[2]);
  ASSERT_EQ(ns + "/unused_str_var", unused[3]);*/
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

