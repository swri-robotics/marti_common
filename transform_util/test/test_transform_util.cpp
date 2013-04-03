// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-62987
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <math_util/constants.h>
#include <transform_util/transform_util.h>

TEST(TransformUtilTests, GetBearing)
{
  EXPECT_FLOAT_EQ(0, transform_util::GetBearing(30, 50, 35, 50));
  EXPECT_FLOAT_EQ(180, transform_util::GetBearing(35, 50, 30, 50));
  EXPECT_FLOAT_EQ(90, transform_util::GetBearing(0, 50, 0, 55));
  EXPECT_FLOAT_EQ(-90, transform_util::GetBearing(0, 55, 0, 50));
}

TEST(TransformUtilTests, SnapToRightAngle)
{
  tf::Quaternion identity = tf::Quaternion::getIdentity();

  EXPECT_TRUE(identity == transform_util::SnapToRightAngle(identity));

  tf::Quaternion q1;
  q1.setRPY(0.0, 0.0, math_util::_half_pi);
  q1.normalize();

  tf::Quaternion q2;
  q2.setRPY(0.4, 0.3, 1.6);

  tf::Quaternion q3 = transform_util::SnapToRightAngle(q2);

  EXPECT_FLOAT_EQ(q1.x(), q3.x());
  EXPECT_FLOAT_EQ(q1.y(), q3.y());
  EXPECT_FLOAT_EQ(q1.z(), q3.z());
  EXPECT_FLOAT_EQ(q1.w(), q3.w());
}

// TODO(malban): Add unit tests for GetRelativeTransform()

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Initialize the ROS core parameters can be loaded from the launch file
  ros::init(argc, argv, "test_transform_util");

  return RUN_ALL_TESTS();
}
