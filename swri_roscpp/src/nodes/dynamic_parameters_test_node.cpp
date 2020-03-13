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

#include <ros/ros.h>

#include <swri_roscpp/dynamic_parameters.h>

#include <boost/function.hpp>
#include <boost/bind/placeholders.hpp>

class DynamicParametersTestNode
{
public:
  DynamicParametersTestNode() :
    nh_("~")
  {
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
    ROS_INFO("Starting initialization timer...");
    init_timer_ = nh_.createWallTimer(ros::WallDuration(0.1),
                                      &DynamicParametersTestNode::initialize,
                                      this,
                                      true);
  }

  bool isTestBool() const
  {
    return test_bool_;
  }

  double getTestDouble() const
  {
    return test_double_;
  }

  int getTestInt() const
  {
    return test_int_;
  }

  float getTestFloat() const
  {
    return test_float_;
  }

  const std::string& getTestString() const
  {
    return test_string_;
  }

  const std::string& getTestEnum() const
  {
    return test_enum_;
  }

private:
  void initialize(const ros::WallTimerEvent &)
  {
    params_.initialize(nh_);
    params_.get("test_bool", test_bool_, false, "Test Bool");
    params_.get("test_double", test_double_, 0.0, "Test Double");
    params_.get("test_int", test_int_, 0, "Test Int");
    params_.get("test_float", test_float_, 0.0f, "Test Float");
    params_.get("test_string", test_string_, "", "Test String");
    std::vector<std::pair<std::string, int> > enums;
    enums.push_back({"First", 1});
    enums.push_back({"Second", 2});
    enums.push_back({"Third", 3});
    enums.push_back({"Zeroth", 0});
    params_.addEnums("test_int", enums);
    params_.finalize();

    params_.setCallback(boost::bind(&DynamicParametersTestNode::handleReconfigure, this, _1));
  }

  void handleReconfigure(swri::DynamicParameters& params)
  {
    ROS_INFO("handleReconfigure");
  }

  ros::NodeHandle nh_;
  ros::WallTimer init_timer_;

  swri::DynamicParameters params_;

  bool test_bool_{false};
  double test_double_{0.0};
  int test_int_{0};
  float test_float_{0.0};
  std::string test_string_;
  std::string test_enum_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_parameters_test");

  DynamicParametersTestNode node;
  ros::spin();

  return 0;
}
