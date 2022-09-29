// *****************************************************************************
//
// Copyright (c) 2022, Southwest Research Institute® (SwRI®)
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
#include <swri_roscpp/node_handle.h>
#include <std_msgs/Empty.h>

class SwriNodeHandleTest : public testing::Test
{
public:
  SwriNodeHandleTest() : nh_(ros::NodeHandle(), ros::NodeHandle("~"), "Testing the swri::NodeHandle")
  {
  }

protected:
  void SetUp() override
  {
  }
  
  void spinNumTimes(size_t num) const
  {
    for (auto i=0; i < num; ++i)
    {
      ros::spinOnce();
      delay_.sleep();
    }
  }

  ros::Duration delay_{0.1};
  swri::NodeHandle nh_;
};

void emptyCb(const std_msgs::EmptyConstPtr &msg)
{
  // Empty
}

TEST_F(SwriNodeHandleTest, testConnectCbAdvertise)
{
  bool connection_called = false;
  auto connect_cb = [&connection_called](const ros::SingleSubscriberPublisher &){
    connection_called = true;
  };
  ros::Publisher test_pub = nh_.advertise<std_msgs::Empty>("test", 1,
      connect_cb,
      ros::SubscriberStatusCallback(),
      ros::VoidConstPtr());

  ros::Subscriber empty_sub = nh_.subscribe<std_msgs::Empty>("test",
      1,
      emptyCb,
      "test subscribe");
  spinNumTimes(2);
  EXPECT_TRUE(connection_called);

  ASSERT_TRUE(nh_.getEnableDocs());
  auto doc_msg = nh_.getDocMsg();

  bool found_test_topic=false;
  for (const auto &topic : doc_msg.topics)
  {
    if (topic.name == "test")
    {
      found_test_topic = true;
    }
  }
  EXPECT_TRUE(found_test_topic);
}

TEST_F(SwriNodeHandleTest, testAdvertiseOptions)
{
  bool cb_called=false;
  auto empty_cb = [&cb_called](const std_msgs::EmptyConstPtr &msg){ cb_called = true; };
  // Don't use swri nodehandle to only have 1 "test" topic doc in documentation message
  ros::NodeHandle public_nh;
  ros::Subscriber empty_sub = public_nh.subscribe<std_msgs::Empty>("test", 1, empty_cb);

  ros::AdvertiseOptions ops;
  ops.init<std_msgs::Empty>("test", 1, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback());
  ops.tracked_object = ros::VoidConstPtr();
  ops.latch = false;
  ros::Publisher empty_pub = nh_.advertise(ops);
  empty_pub.publish(boost::make_shared<std_msgs::Empty>());

  spinNumTimes(3);
  EXPECT_TRUE(cb_called);

  ASSERT_TRUE(nh_.getEnableDocs());
  auto doc_msg = nh_.getDocMsg();

  bool found_test_topic=false;
  for (const auto &topic : doc_msg.topics)
  {
    if (topic.name == "test")
    {
      found_test_topic = true;
    }
  }
  EXPECT_TRUE(found_test_topic);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swri_node_handle_test");
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
