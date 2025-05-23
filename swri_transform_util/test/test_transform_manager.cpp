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

#include <atomic>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.hpp>

#include <swri_transform_util/transform_manager.h>
#include <swri_transform_util/frames.h>

static std::shared_ptr<rclcpp::Node> _node;
static std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
static std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
static std::shared_ptr<swri_transform_util::TransformManager> _tf_manager;

class TransformManagerTests : public ::testing::Test
{
public:
  void SetUp() override
  {
    ASSERT_TRUE(_tf_manager != nullptr);
    // Wait until the local_xy_origin is setup correctly
    bool origin_init = false;
    for (size_t i=0; i < 100; ++i)
    {
      if (_tf_manager->SupportsTransform("far_field", "far_field__identity"))
      {
        origin_init = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ASSERT_TRUE(origin_init);
  }
};

TEST_F(TransformManagerTests, Identity1)
{
  tf2::Vector3 p1(56, 234, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/near_field",
      "/near_field",
      transform));

  tf2::Vector3 p2 = transform * p1;

  EXPECT_FLOAT_EQ(p1.x(), p2.x());
  EXPECT_FLOAT_EQ(p1.y(), p2.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * p2;
  EXPECT_FLOAT_EQ(p1.x(), p3.x());
  EXPECT_FLOAT_EQ(p1.y(), p3.y());
}

TEST_F(TransformManagerTests, IdentityNoSlash)
{
  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/near_field",
      "near_field",
      transform));
  ASSERT_TRUE(_tf_manager->GetTransform(
      "near_field",
      "/near_field",
      transform));
}

TEST_F(TransformManagerTests, Identity2)
{
  tf2::Vector3 p1(435, -900, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/some_frame",
      "/some_frame",
      transform));

  tf2::Vector3 p2 = transform * p1;

  EXPECT_FLOAT_EQ(p1.x(), p2.x());
  EXPECT_FLOAT_EQ(p1.y(), p2.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * p2;
  EXPECT_FLOAT_EQ(p1.x(), p3.x());
  EXPECT_FLOAT_EQ(p1.y(), p3.y());
}

TEST_F(TransformManagerTests, TfToTf1)
{
  // Local Origin
  tf2::Vector3 far_field(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/near_field",
      "/far_field",
      transform));

  tf2::Vector3 near_field = transform * far_field;

  EXPECT_FLOAT_EQ(-500, near_field.x());
  EXPECT_FLOAT_EQ(-500, near_field.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * near_field;
  EXPECT_FLOAT_EQ(far_field.x(), p3.x());
  EXPECT_FLOAT_EQ(far_field.y(), p3.y());
}

TEST_F(TransformManagerTests, TfToTf1NoSlash)
{
  // Local Origin
  tf2::Vector3 far_field(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/near_field",
      "far_field",
      transform));

  tf2::Vector3 near_field = transform * far_field;

  EXPECT_FLOAT_EQ(-500, near_field.x());
  EXPECT_FLOAT_EQ(-500, near_field.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * near_field;
  EXPECT_FLOAT_EQ(far_field.x(), p3.x());
  EXPECT_FLOAT_EQ(far_field.y(), p3.y());
}

TEST_F(TransformManagerTests, TfToTf2)
{
  // Local Origin
  tf2::Vector3 far_field(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/near_field",
      "/far_field",
      transform));

  tf2::Vector3 near_field = transform * far_field;

  EXPECT_FLOAT_EQ(-500, near_field.x());
  EXPECT_FLOAT_EQ(-500, near_field.y());
}

TEST_F(TransformManagerTests, WgsToUtm)
{
  // San Antonio International Airport
  tf2::Vector3 wgs84(-98.471944, 29.526667, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      swri_transform_util::_utm_frame,
      swri_transform_util::_wgs84_frame,
      transform));

  tf2::Vector3 utm = transform * wgs84;

  EXPECT_FLOAT_EQ(551170, utm.x());
  EXPECT_FLOAT_EQ(3266454, utm.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * utm;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST_F(TransformManagerTests, WgsToUtmNoSlash)
{
  // San Antonio International Airport
  tf2::Vector3 wgs84(-98.471944, 29.526667, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "utm",
      "wgs84",
      transform));

  tf2::Vector3 utm = transform * wgs84;

  EXPECT_FLOAT_EQ(551170, utm.x());
  EXPECT_FLOAT_EQ(3266454, utm.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * utm;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST_F(TransformManagerTests, UtmToWgs84)
{
  // San Antonio International Airport
  tf2::Vector3 utm(551170, 3266454, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      swri_transform_util::_wgs84_frame,
      swri_transform_util::_utm_frame,
      transform));

  tf2::Vector3 wgs84 = transform * utm;

  EXPECT_FLOAT_EQ(29.526667, wgs84.y());
  EXPECT_FLOAT_EQ(-98.471944, wgs84.x());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * wgs84;
  EXPECT_FLOAT_EQ(utm.x(), p3.x());
  EXPECT_FLOAT_EQ(utm.y(), p3.y());
}

TEST_F(TransformManagerTests, TfToUtm1)
{
  // Local Origin
  tf2::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      swri_transform_util::_utm_frame,
      "/far_field",
      transform));

  tf2::Vector3 utm = transform * tf;

  EXPECT_FLOAT_EQ(537460.3372816057, utm.x());
  EXPECT_FLOAT_EQ(3258123.434110421, utm.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * utm;
  EXPECT_NEAR(tf.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(tf.y(), p3.y(), 0.00000001);
}

TEST_F(TransformManagerTests, TfToUtm1NoSlash)
{
  // Local Origin
  tf2::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "utm",
      "far_field",
      transform));

  tf2::Vector3 utm = transform * tf;

  EXPECT_FLOAT_EQ(537460.3372816057, utm.x());
  EXPECT_FLOAT_EQ(3258123.434110421, utm.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * utm;
  EXPECT_NEAR(tf.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(tf.y(), p3.y(), 0.00000001);
}

TEST_F(TransformManagerTests, TfToUtm2)
{
  tf2::Vector3 tf(500, 500, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      swri_transform_util::_utm_frame,
      "/far_field",
      transform));

  tf2::Vector3 utm = transform * tf;

  EXPECT_NEAR(537460.3372816057 + 500.0, utm.x(), 1.9);
  EXPECT_NEAR(3258123.434110421 + 500.0, utm.y(), 1.5);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * utm;
  EXPECT_NEAR(tf.x(), p3.x(), 0.05);
  EXPECT_NEAR(tf.y(), p3.y(), 0.05);
}

TEST_F(TransformManagerTests, UtmToTf1)
{
  // Local Origin
  tf2::Vector3 utm(537460.3372816057, 3258123.434110421, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/far_field",
      swri_transform_util::_utm_frame,
      transform));

  tf2::Vector3 tf = transform * utm;

  EXPECT_NEAR(0, tf.x(), 0.0005);
  EXPECT_NEAR(0, tf.y(), 0.0005);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * tf;
  EXPECT_NEAR(utm.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(utm.y(), p3.y(), 0.00000001);
}

TEST_F(TransformManagerTests, UtmToTf2)
{
  // Local Origin
  tf2::Vector3 utm(537460.3372816057, 3258123.434110421, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/near_field",
      swri_transform_util::_utm_frame,
      transform));

  tf2::Vector3 tf = transform * utm;

  EXPECT_NEAR(-500, tf.x(), 0.0005);
  EXPECT_NEAR(-500, tf.y(), 0.0005);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * tf;
  EXPECT_NEAR(utm.x(), p3.x(), 0.00000001);
  EXPECT_NEAR(utm.y(), p3.y(), 0.00000001);
}

TEST_F(TransformManagerTests, UtmToTf3)
{
  // Local Origin
  tf2::Vector3 utm(537460.3372816057 - 500, 3258123.434110421 - 500, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/far_field",
      swri_transform_util::_utm_frame,
      transform));

  tf2::Vector3 tf = transform * utm;

  EXPECT_NEAR(-500, tf.x(), 1.9);
  EXPECT_NEAR(-500, tf.y(), 1.5);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * tf;
  EXPECT_NEAR(utm.x(), p3.x(), 0.05);
  EXPECT_NEAR(utm.y(), p3.y(), 0.05);
}

TEST_F(TransformManagerTests, UtmToTf4)
{
  // San Antonio International Airport
  tf2::Vector3 utm(551170, 3266454, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/far_field",
      swri_transform_util::_utm_frame,
      transform));

  tf2::Vector3 tf = transform * utm;

  EXPECT_FLOAT_EQ(13742.387, tf.x());
  EXPECT_FLOAT_EQ(8288.1162, tf.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * tf;
  EXPECT_NEAR(utm.x(), p3.x(), 0.6);
  EXPECT_NEAR(utm.y(), p3.y(), 0.4);
}

TEST_F(TransformManagerTests, Wgs84ToTf1)
{
  // Local Origin
  tf2::Vector3 wgs84(-98.61370577, 29.45196669, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/far_field",
      swri_transform_util::_wgs84_frame,
      transform));

  tf2::Vector3 tf = transform * wgs84;

  EXPECT_NEAR(0, tf.x(), 1e-8);
  EXPECT_NEAR(0, tf.y(), 1e-8);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * tf;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST_F(TransformManagerTests, Wgs84ToTf1NoSlash)
{
  // Local Origin
  tf2::Vector3 wgs84(-98.61370577, 29.45196669, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "far_field",
      "wgs84",
      transform));

  tf2::Vector3 tf = transform * wgs84;

  EXPECT_NEAR(0, tf.x(), 1e-8);
  EXPECT_NEAR(0, tf.y(), 1e-8);

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * tf;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST_F(TransformManagerTests, Wgs84ToTf2)
{
  // Local Origin
  tf2::Vector3 wgs84(-98.61370577, 29.45196669, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "/near_field",
      swri_transform_util::_wgs84_frame,
      transform));

  tf2::Vector3 tf = transform * wgs84;

  EXPECT_FLOAT_EQ(-500, tf.x());
  EXPECT_FLOAT_EQ(-500, tf.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * tf;
  EXPECT_FLOAT_EQ(wgs84.x(), p3.x());
  EXPECT_FLOAT_EQ(wgs84.y(), p3.y());
}

TEST_F(TransformManagerTests, TfToWgs84_1)
{
  // Local Origin
  tf2::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      swri_transform_util::_wgs84_frame,
      "/far_field",
      transform));

  tf2::Vector3 wgs84 = transform * tf;

  EXPECT_FLOAT_EQ(-98.61370577, wgs84.x());
  EXPECT_FLOAT_EQ(29.45196669, wgs84.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * wgs84;
  EXPECT_NEAR(tf.x(), p3.x(), 1e-8);
  EXPECT_NEAR(tf.y(), p3.y(), 1e-8);
}

TEST_F(TransformManagerTests, TfToWgs84_1NoSlash)
{
  // Local Origin
  tf2::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      "wgs84",
      "far_field",
      transform));

  tf2::Vector3 wgs84 = transform * tf;

  EXPECT_FLOAT_EQ(-98.61370577, wgs84.x());
  EXPECT_FLOAT_EQ(29.45196669, wgs84.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * wgs84;
  EXPECT_NEAR(tf.x(), p3.x(), 1e-8);
  EXPECT_NEAR(tf.y(), p3.y(), 1e-8);
}

TEST_F(TransformManagerTests, TfToWgs84_2)
{
  tf2::Vector3 tf(0, 0, 0);

  swri_transform_util::Transform transform;
  ASSERT_TRUE(_tf_manager->GetTransform(
      swri_transform_util::_wgs84_frame,
      "/near_field",
      transform));

  tf2::Vector3 wgs84 = transform * tf;

  EXPECT_FLOAT_EQ(-98.6085519577, wgs84.x());
  EXPECT_FLOAT_EQ(29.4564773982, wgs84.y());

  swri_transform_util::Transform inverse = transform.Inverse();
  tf2::Vector3 p3 = inverse * wgs84;
  EXPECT_NEAR(tf.x(), p3.x(), 0.02);
  EXPECT_NEAR(tf.y(), p3.y(), 0.02);
}

// Run all the tests that were declared with TEST_F()
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  _node = rclcpp::Node::make_shared("transform_manager_test");
  _tf_buffer = std::make_shared<tf2_ros::Buffer>(_node->get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer, _node);

  _tf_manager = std::make_shared<swri_transform_util::TransformManager>(_node, _tf_buffer);

  // background spinner thread using main executor
  std::atomic<bool> tests_done(false);
  std::thread spinner = std::thread([&tests_done]() {
      while (not tests_done)
      {
        rclcpp::spin_some(_node);
      }
  });

  int result = RUN_ALL_TESTS();
  tests_done = true;
  if (spinner.joinable())
  {
    spinner.join();
  }
  rclcpp::shutdown();
  _tf_manager.reset();
  _tf_listener.reset();
  _tf_buffer.reset();
  _node.reset();
  return result;
}
