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
#include <tf/transform_datatypes.h>

#include <transform_util/transform_manager.h>
#include <transform_util/frames.h>

TEST(TransformManagerTests, Initialize)
{
  transform_util::TransformManager tf_manager;;
  tf_manager.Initialize();
}

TEST(TransformManagerTests, Identity1)
{
  transform_util::TransformManager tf_manager;
  tf_manager.Initialize();

  tf::Vector3 p1(56, 234, 0);

  transform_util::Transform transform;
  ASSERT_TRUE(tf_manager.GetTransform(
      "/near_field",
      "/near_field",
      transform));

  tf::Vector3 p2 = transform * p1;

  EXPECT_FLOAT_EQ(p1.x(), p2.x());
  EXPECT_FLOAT_EQ(p1.y(), p2.y());
}

TEST(TransformManagerTests, Identity2)
{
  transform_util::TransformManager tf_manager;
  tf_manager.Initialize();

  tf::Vector3 p1(435, -900, 0);

  transform_util::Transform transform;
  ASSERT_TRUE(tf_manager.GetTransform(
      "/some_frame",
      "/some_frame",
      transform));

  tf::Vector3 p2 = transform * p1;

  EXPECT_FLOAT_EQ(p1.x(), p2.x());
  EXPECT_FLOAT_EQ(p1.y(), p2.y());
}

/*
TEST(TransformManagerTests, TfToTf)
{
  tf::TransformListener tf;

  sleep(1);

  EXPECT_EQ(std::string("dummy"), tf.allFramesAsString());
  EXPECT_FALSE(tf.frameExists("/near_field"));
  EXPECT_FALSE(tf.frameExists("/far_field"));

  transform_util::TransformManager tf_manager;
  tf_manager.Initialize();

  // Local Origin
  tf::Vector3 far_field(0, 0, 0);

  transform_util::Transform transform;
  ASSERT_TRUE(tf_manager.GetTransform(
      "/near_field",
      "/far_field",
      transform));

  tf::Vector3 near_field = transform * far_field;

  EXPECT_FLOAT_EQ(-500, near_field.x());
  EXPECT_FLOAT_EQ(-500, near_field.y());
}
*/

TEST(TransformManagerTests, WgsToUtm)
{
  transform_util::TransformManager tf_manager;
  tf_manager.Initialize();

  // San Antonio International Airport
  tf::Vector3 wgs84(-98.471944, 29.526667, 0);

  transform_util::Transform transform;
  ASSERT_TRUE(tf_manager.GetTransform(
      transform_util::_utm_frame,
      transform_util::_wgs84_frame,
      transform));

  tf::Vector3 utm = transform * wgs84;

  EXPECT_FLOAT_EQ(551170, utm.x());
  EXPECT_FLOAT_EQ(3266454, utm.y());
}

TEST(TransformManagerTests, UtmToWgs84)
{
  transform_util::TransformManager tf_manager;
  tf_manager.Initialize();

  // San Antonio International Airport
  tf::Vector3 utm(551170, 3266454, 0);

  transform_util::Transform transform;
  ASSERT_TRUE(tf_manager.GetTransform(
      transform_util::_wgs84_frame,
      transform_util::_utm_frame,
      transform));

  tf::Vector3 wgs84 = transform * utm;

  EXPECT_FLOAT_EQ(29.526667, wgs84.x());
  EXPECT_FLOAT_EQ(-98.471944, wgs84.y());
}

/*
TEST(TransformManagerTests, Wgs84ToTf1)
{
  transform_util::TransformManager tf_manager;
  tf_manager.Initialize();

  // Local Origin
  tf::Vector3 wgs84(-98.61370577, 29.45196669, 0);

  transform_util::Transform transform;
  ASSERT_TRUE(tf_manager.GetTransform(
      "/far_field",
      transform_util::_wgs84_frame,
      transform));

  tf::Vector3 tf = transform * wgs84;

  EXPECT_FLOAT_EQ(0, tf.x());
  EXPECT_FLOAT_EQ(0, tf.y());
}

TEST(TransformManagerTests, Wgs84ToTf2)
{
  transform_util::TransformManager tf_manager;
  tf_manager.Initialize();

  // Local Origin
  tf::Vector3 wgs84(-98.61370577, 29.45196669, 0);

  transform_util::Transform transform;
  ASSERT_TRUE(tf_manager.GetTransform(
      "/near_field",
      transform_util::_wgs84_frame,
      transform));

  tf::Vector3 tf = transform * wgs84;

  EXPECT_FLOAT_EQ(-500, tf.x());
  EXPECT_FLOAT_EQ(-500, tf.y());
}
*/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Initialize the ROS core parameters can be loaded from the launch file
  ros::init(argc, argv, "test_transform_manager");

  return RUN_ALL_TESTS();
}
