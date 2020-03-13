// *****************************************************************************
//
// Copyright (c) 2020, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/ConfigDescription.h>

#include <yaml-cpp/yaml.h>

/**
 * Convenience function for checking that several values in a reconfigure
 * response are what we expect
 */
void checkValues(const dynamic_reconfigure::ReconfigureResponse& srv_resp, int int_value, double double_value, std::string string_value, bool bool_value)
{
  bool got_int = false;
  ASSERT_EQ(srv_resp.config.ints.size(), 1);
  for (auto& value : srv_resp.config.ints)
  {
    if (value.name == "test_int")
    {
      ASSERT_EQ(value.value, int_value);
      got_int = true;
    }
  }
  ASSERT_TRUE(got_int);

  bool got_str = false;
  ASSERT_EQ(srv_resp.config.strs.size(), 1);
  for (auto& value : srv_resp.config.strs)
  {
    if (value.name == "test_string")
    {
      ASSERT_EQ(value.value, string_value);
      got_str = true;
    }
  }
  ASSERT_TRUE(got_str);

  bool got_bool = false;
  ASSERT_EQ(srv_resp.config.bools.size(), 1);
  for (auto& value : srv_resp.config.bools)
  {
    if (value.name == "test_bool")
    {
      ASSERT_EQ(value.value, bool_value);
      got_bool = true;
    }
  }
  ASSERT_TRUE(got_bool);

  bool got_double = false;
  bool got_float = false;
  ASSERT_EQ(srv_resp.config.doubles.size(), 2);
  for (auto& value : srv_resp.config.doubles)
  {
    if (value.name == "test_double")
    {
      ASSERT_DOUBLE_EQ(value.value, double_value);
      got_double = true;
    }
    if (value.name == "test_float")
    {
      got_float = true;
    }
  }
  ASSERT_TRUE(got_double);
  ASSERT_TRUE(got_float);
}

/**
 * This test will:
 * 1) Verify that the test node's service is advertised
 * 2) Check that the default values for its parameters are all 0 / empty
 * 3) Set all of their values through the service
 * 4) Verify that the values are set as expected
 */
TEST(DynamicParameters, testGetAndSetParams)
{
  ros::NodeHandle nh;
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;

  ros::ServiceClient client =
      nh.serviceClient<dynamic_reconfigure::Reconfigure>("/dynamic_parameters_test/set_parameters");
  bool result = client.waitForExistence(ros::Duration(5.0));
  ASSERT_TRUE(result);
  result = client.call(srv_req, srv_resp);
  ASSERT_TRUE(result);

  checkValues(srv_resp, 0, 0.0, "", false);

  dynamic_reconfigure::Config config = srv_resp.config;
  config.doubles[0].value = 1.0;
  config.ints[0].value = 2;
  config.strs[0].value = "test";
  config.bools[0].value = true;

  srv_req.config = config;
  result = client.call(srv_req, srv_resp);
  ASSERT_TRUE(result);

  checkValues(srv_resp, 2, 1.0, "test", true);
}

/**
 * Test that the parameter descriptions are being published as expected.
 */
TEST(DynamicParameters, testParamDescriptions)
{
  ros::NodeHandle nh("~");
  bool got_description = false;
  dynamic_reconfigure::ConfigDescription desc;

  ros::Subscriber sub = nh.subscribe<dynamic_reconfigure::ConfigDescription>(
      "/dynamic_parameters_test/parameter_descriptions",
      1,
      [&got_description, &desc](const dynamic_reconfigure::ConfigDescriptionConstPtr& msg) {
        got_description = true;
        desc = *msg;
  });

  ros::Rate rate(10);
  ros::Time start = ros::Time::now();
  while (!got_description && ros::Time::now() - start < ros::Duration(5))
  {
    ros::spinOnce();
    rate.sleep();
  }

  ASSERT_TRUE(got_description);

  bool got_edit_method = false;
  ASSERT_EQ(desc.groups.size(), 1);
  for (const auto& group : desc.groups)
  {
    ASSERT_EQ(group.name, "Default");
    ASSERT_EQ(group.parameters.size(), 5);
    for (const auto& param : group.parameters)
    {
      if (param.name == "test_int" && !param.edit_method.empty())
      {
        got_edit_method = true;
        YAML::Node node = YAML::Load(param.edit_method);
        ASSERT_TRUE(node.IsMap());
        ASSERT_TRUE(node["enum_description"]);
        ASSERT_TRUE(node["enum"]);
        YAML::Node enums = node["enum"];
        ASSERT_EQ(enums.size(), 4);
        YAML::Node first = enums[0];
        ASSERT_EQ(first["srcline"].as<int>(), 0);
        ASSERT_EQ(first["description"].as<std::string>(), "Unknown");
        ASSERT_EQ(first["srcfile"].as<std::string>(), "dynamic_parameters.h");
        ASSERT_EQ(first["cconsttype"].as<std::string>(), "const int");
        ASSERT_EQ(first["value"].as<int>(), 1);
        ASSERT_EQ(first["ctype"].as<std::string>(), "int");
        ASSERT_EQ(first["type"].as<std::string>(), "int");
        ASSERT_EQ(first["name"].as<std::string>(), "First");
        first = enums[1];
        ASSERT_EQ(first["srcline"].as<int>(), 0);
        ASSERT_EQ(first["description"].as<std::string>(), "Unknown");
        ASSERT_EQ(first["srcfile"].as<std::string>(), "dynamic_parameters.h");
        ASSERT_EQ(first["cconsttype"].as<std::string>(), "const int");
        ASSERT_EQ(first["value"].as<int>(), 2);
        ASSERT_EQ(first["ctype"].as<std::string>(), "int");
        ASSERT_EQ(first["type"].as<std::string>(), "int");
        ASSERT_EQ(first["name"].as<std::string>(), "Second");
      }
    }
  }
  ASSERT_TRUE(got_edit_method);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_parameters_test");
  testing::InitGoogleTest(&argc, argv);

  ros::start();
  int result = RUN_ALL_TESTS();
  ros::shutdown();

  return result;
}
