// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
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

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <swri_transform_util/transform_manager.h>
#include <swri_transform_util/frames.h>

swri_transform_util::TransformManager _tf_manager;

TEST(TransformManagerTests, Identity1)
{
  tf::Vector3 p1(56, 234, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/near_field",
      "/near_field",
      transform));

  tf::Vector3 p2 = transform * p1;

  EXPECT_FLOAT_EQ(p1.x(), p2.x());
  EXPECT_FLOAT_EQ(p1.y(), p2.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * p2;
  EXPECT_FLOAT_EQ(p1.x(), p3.x());
  EXPECT_FLOAT_EQ(p1.y(), p3.y());
}

TEST(TransformManagerTests, IdentityNoSlash)
{
  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/near_field",
      "near_field",
      transform));
  ASSERT_TRUE(_tf_manager.GetTransform(
      "near_field",
      "/near_field",
      transform));
}

TEST(TransformManagerTests, Identity2)
{
  tf::Vector3 p1(435, -900, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/some_frame",
      "/some_frame",
      transform));

  tf::Vector3 p2 = transform * p1;

  EXPECT_FLOAT_EQ(p1.x(), p2.x());
  EXPECT_FLOAT_EQ(p1.y(), p2.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * p2;
  EXPECT_FLOAT_EQ(p1.x(), p3.x());
  EXPECT_FLOAT_EQ(p1.y(), p3.y());
}

TEST(TransformManagerTests, TfToTf1)
{
  // Local Origin
  tf::Vector3 far_field(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/near_field",
      "/far_field",
      transform));

  tf::Vector3 near_field = transform * far_field;

  EXPECT_FLOAT_EQ(-500, near_field.x());
  EXPECT_FLOAT_EQ(-500, near_field.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * near_field;
  EXPECT_FLOAT_EQ(far_field.x(), p3.x());
  EXPECT_FLOAT_EQ(far_field.y(), p3.y());
}

TEST(TransformManagerTests, TfToTf1NoSlash)
{
  // Local Origin
  tf::Vector3 far_field(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/near_field",
      "far_field",
      transform));

  tf::Vector3 near_field = transform * far_field;

  EXPECT_FLOAT_EQ(-500, near_field.x());
  EXPECT_FLOAT_EQ(-500, near_field.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * near_field;
  EXPECT_FLOAT_EQ(far_field.x(), p3.x());
  EXPECT_FLOAT_EQ(far_field.y(), p3.y());
}

TEST(TransformManagerTests, TfToTf2)
{
  // Local Origin
  tf::Vector3 far_field(0, 0, 0);

  tf::StampedTransform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/near_field",
      "/far_field",
      transform));

  tf::Vector3 near_field = transform * far_field;

  EXPECT_FLOAT_EQ(-500, near_field.x());
  EXPECT_FLOAT_EQ(-500, near_field.y());
}

TEST(TransformManagerTests, WgsToUtm)
{
  // San Antonio International Airport
  tf::Vector3 wgs84(-98.471944, 29.526667, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      swri_transform_util::_utm_frame,
      swri_transform_util::_wgs84_frame,
      transform));

  tf::Vector3 utm = transform * wgs84;

  EXPECT_FLOAT_EQ(551170, utm.x());
  EXPECT_FLOAT_EQ(3266454, utm.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * utm;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST(TransformManagerTests, WgsToUtmNoSlash)
{
  // San Antonio International Airport
  tf::Vector3 wgs84(-98.471944, 29.526667, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "utm",
      "wgs84",
      transform));

  tf::Vector3 utm = transform * wgs84;

  EXPECT_FLOAT_EQ(551170, utm.x());
  EXPECT_FLOAT_EQ(3266454, utm.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * utm;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST(TransformManagerTests, UtmToWgs84)
{
  // San Antonio International Airport
  tf::Vector3 utm(551170, 3266454, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      swri_transform_util::_wgs84_frame,
      swri_transform_util::_utm_frame,
      transform));

  tf::Vector3 wgs84 = transform * utm;

  EXPECT_FLOAT_EQ(29.526667, wgs84.y());
  EXPECT_FLOAT_EQ(-98.471944, wgs84.x());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * wgs84;
  EXPECT_FLOAT_EQ(utm.x(), p3.x());
  EXPECT_FLOAT_EQ(utm.y(), p3.y());
}

TEST(TransformManagerTests, TfToUtm1)
{
  // Local Origin
  tf::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      swri_transform_util::_utm_frame,
      "/far_field",
      transform));

  tf::Vector3 utm = transform * tf;

  EXPECT_FLOAT_EQ(537460.3372816057, utm.x());
  EXPECT_FLOAT_EQ(3258123.434110421, utm.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * utm;
  EXPECT_NEAR(tf.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(tf.y(), p3.y(), 0.00000001);
}

TEST(TransformManagerTests, TfToUtm1NoSlash)
{
  // Local Origin
  tf::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "utm",
      "far_field",
      transform));

  tf::Vector3 utm = transform * tf;

  EXPECT_FLOAT_EQ(537460.3372816057, utm.x());
  EXPECT_FLOAT_EQ(3258123.434110421, utm.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * utm;
  EXPECT_NEAR(tf.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(tf.y(), p3.y(), 0.00000001);
}

TEST(TransformManagerTests, TfToUtm2)
{
  tf::Vector3 tf(500, 500, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      swri_transform_util::_utm_frame,
      "/far_field",
      transform));

  tf::Vector3 utm = transform * tf;

  EXPECT_NEAR(537460.3372816057 + 500.0, utm.x(), 1.9);
  EXPECT_NEAR(3258123.434110421 + 500.0, utm.y(), 1.5);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * utm;
  EXPECT_NEAR(tf.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(tf.y(), p3.y(), 0.00000001);
}

TEST(TransformManagerTests, UtmToTf1)
{
  // Local Origin
  tf::Vector3 utm(537460.3372816057, 3258123.434110421, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/far_field",
      swri_transform_util::_utm_frame,
      transform));

  tf::Vector3 tf = transform * utm;

  EXPECT_NEAR(0, tf.x(), 0.0005);
  EXPECT_NEAR(0, tf.y(), 0.0005);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * tf;
  EXPECT_NEAR(utm.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(utm.y(), p3.y(), 0.00000001);
}

TEST(TransformManagerTests, UtmToTf2)
{
  // Local Origin
  tf::Vector3 utm(537460.3372816057, 3258123.434110421, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/near_field",
      swri_transform_util::_utm_frame,
      transform));

  tf::Vector3 tf = transform * utm;

  EXPECT_NEAR(-500, tf.x(), 0.0005);
  EXPECT_NEAR(-500, tf.y(), 0.0005);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * tf;
  EXPECT_NEAR(utm.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(utm.y(), p3.y(), 0.00000001);
}

TEST(TransformManagerTests, UtmToTf3)
{
  // Local Origin
  tf::Vector3 utm(537460.3372816057 - 500, 3258123.434110421 - 500, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/far_field",
      swri_transform_util::_utm_frame,
      transform));

  tf::Vector3 tf = transform * utm;

  EXPECT_NEAR(-500, tf.x(), 1.9);
  EXPECT_NEAR(-500, tf.y(), 1.5);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * tf;
  EXPECT_NEAR(utm.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(utm.y(), p3.y(), 0.00000001);
}

TEST(TransformManagerTests, UtmToTf4)
{
  // San Antonio International Airport
  tf::Vector3 utm(551170, 3266454, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/far_field",
      swri_transform_util::_utm_frame,
      transform));

  tf::Vector3 tf = transform * utm;

  EXPECT_FLOAT_EQ(13752.988, tf.x());
  EXPECT_FLOAT_EQ(8280.0176, tf.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * tf;
  EXPECT_NEAR(utm.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(utm.y(), p3.y(), 0.00000001);
}

TEST(TransformManagerTests, Wgs84ToTf1)
{
  // Local Origin
  tf::Vector3 wgs84(-98.61370577, 29.45196669, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/far_field",
      swri_transform_util::_wgs84_frame,
      transform));

  tf::Vector3 tf = transform * wgs84;

  EXPECT_FLOAT_EQ(0, tf.x());
  EXPECT_FLOAT_EQ(0, tf.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * tf;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST(TransformManagerTests, Wgs84ToTf1NoSlash)
{
  // Local Origin
  tf::Vector3 wgs84(-98.61370577, 29.45196669, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "far_field",
      "wgs84",
      transform));

  tf::Vector3 tf = transform * wgs84;

  EXPECT_FLOAT_EQ(0, tf.x());
  EXPECT_FLOAT_EQ(0, tf.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * tf;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST(TransformManagerTests, Wgs84ToTf2)
{
  // Local Origin
  tf::Vector3 wgs84(-98.61370577, 29.45196669, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "/near_field",
      swri_transform_util::_wgs84_frame,
      transform));

  tf::Vector3 tf = transform * wgs84;

  EXPECT_FLOAT_EQ(-500, tf.x());
  EXPECT_FLOAT_EQ(-500, tf.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * tf;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST(TransformManagerTests, TfToWgs84_1)
{
  // Local Origin
  tf::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      swri_transform_util::_wgs84_frame,
      "/far_field",
      transform));

  tf::Vector3 wgs84 = transform * tf;

  EXPECT_FLOAT_EQ(-98.61370577, wgs84.x());
  EXPECT_FLOAT_EQ(29.45196669, wgs84.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * wgs84;
  EXPECT_FLOAT_EQ(tf.x(), p3.x());
  EXPECT_FLOAT_EQ(tf.y(), p3.y());
}

TEST(TransformManagerTests, TfToWgs84_1NoSlash)
{
  // Local Origin
  tf::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      "wgs84",
      "far_field",
      transform));

  tf::Vector3 wgs84 = transform * tf;

  EXPECT_FLOAT_EQ(-98.61370577, wgs84.x());
  EXPECT_FLOAT_EQ(29.45196669, wgs84.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * wgs84;
  EXPECT_FLOAT_EQ(tf.x(), p3.x());
  EXPECT_FLOAT_EQ(tf.y(), p3.y());
}

TEST(TransformManagerTests, TfToWgs84_2)
{
  tf::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager.GetTransform(
      swri_transform_util::_wgs84_frame,
      "/near_field",
      transform));

  tf::Vector3 wgs84 = transform * tf;

  EXPECT_FLOAT_EQ(-98.6085519577, wgs84.x());
  EXPECT_FLOAT_EQ(29.4564773982, wgs84.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf::Vector3 p3 = inverse * wgs84;
  EXPECT_NEAR(tf.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(tf.y(), p3.y(), 0.00000001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Initialize the ROS core parameters can be loaded from the launch file
  ros::init(argc, argv, "test_transform_manager");

  _tf_manager.Initialize();

  ros::AsyncSpinner spinner(1);
  spinner.start();
  sleep(10);

  bool result = RUN_ALL_TESTS();
  spinner.stop();
  return result;
}
