// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
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

#include <memory>

#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <swri_transform_util/local_xy_util.h>
#include <swri_transform_util/transform.h>
#include <swri_transform_util/wgs84_transformer.h>

namespace
{
  geometry_msgs::msg::TransformStamped MakeTf(
    const std::string& parent = "map",
    const std::string& child = "map",
    double x = 0.0)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = x;
    tf.transform.rotation.w = 1.0;
    return tf;
  }
}

TEST(TransformEqualityTests, IdentityTransformsAreEquivalent)
{
  EXPECT_TRUE(swri_transform_util::Transform() == swri_transform_util::Transform());
  EXPECT_FALSE(swri_transform_util::Transform() != swri_transform_util::Transform());
}

TEST(TransformEqualityTests, TfTransformsCompareByGeometry)
{
  tf2::Transform a(tf2::Quaternion::getIdentity(), tf2::Vector3(1.0, 2.0, 3.0));
  tf2::Transform b(tf2::Quaternion::getIdentity(), tf2::Vector3(1.0, 2.0, 3.0));
  tf2::Transform c(tf2::Quaternion::getIdentity(), tf2::Vector3(1.0, 2.0, 4.0));

  EXPECT_TRUE(swri_transform_util::Transform(a) == swri_transform_util::Transform(b));
  EXPECT_TRUE(swri_transform_util::Transform(a) != swri_transform_util::Transform(c));
}

TEST(TransformEqualityTests, DifferentImplementationsAreNotEquivalent)
{
  auto local_xy = std::make_shared<swri_transform_util::LocalXyWgs84Util>(0.0, 0.0);
  swri_transform_util::Transform wgs84(
    std::make_shared<swri_transform_util::Wgs84ToTfTransform>(MakeTf(), local_xy));

  // The two map (0, 0, 0) to the same place and share an orientation, so
  // comparing origins and orientations cannot tell them apart. They are not
  // interchangeable: one converts degrees into meters, the other does nothing.
  // See https://github.com/swri-robotics/mapviz/issues/909.
  swri_transform_util::Transform identity;
  ASSERT_EQ(identity.GetOrigin(), wgs84.GetOrigin());
  ASSERT_EQ(identity.GetOrientation(), wgs84.GetOrientation());

  EXPECT_TRUE(wgs84 != identity);
  EXPECT_TRUE(identity != wgs84);
}

TEST(TransformEqualityTests, Wgs84TransformsCompareByOriginAndGeometry)
{
  auto local_xy = std::make_shared<swri_transform_util::LocalXyWgs84Util>(29.45, -98.6);
  auto same_origin = std::make_shared<swri_transform_util::LocalXyWgs84Util>(29.45, -98.6);
  auto other_origin = std::make_shared<swri_transform_util::LocalXyWgs84Util>(0.0, 0.0);

  swri_transform_util::Transform transform(
    std::make_shared<swri_transform_util::Wgs84ToTfTransform>(MakeTf(), local_xy));

  // A separately constructed but identical transform is interchangeable.
  EXPECT_TRUE(transform == swri_transform_util::Transform(
      std::make_shared<swri_transform_util::Wgs84ToTfTransform>(MakeTf(), same_origin)));

  // A moved local XY origin is not.
  EXPECT_TRUE(transform != swri_transform_util::Transform(
      std::make_shared<swri_transform_util::Wgs84ToTfTransform>(MakeTf(), other_origin)));

  // Neither is different TF geometry, or a different frame.
  EXPECT_TRUE(transform != swri_transform_util::Transform(
      std::make_shared<swri_transform_util::Wgs84ToTfTransform>(MakeTf("map", "map", 5.0), local_xy)));
  EXPECT_TRUE(transform != swri_transform_util::Transform(
      std::make_shared<swri_transform_util::Wgs84ToTfTransform>(MakeTf("map", "odom"), local_xy)));
}

TEST(TransformEqualityTests, Wgs84TransformsIgnoreTheTfTimestamp)
{
  auto local_xy = std::make_shared<swri_transform_util::LocalXyWgs84Util>(29.45, -98.6);

  geometry_msgs::msg::TransformStamped later = MakeTf();
  later.header.stamp.sec = 1000;

  // Looking the same transform up again must not count as a change; only the
  // geometry decides where points land.
  EXPECT_TRUE(swri_transform_util::Transform(
      std::make_shared<swri_transform_util::Wgs84ToTfTransform>(MakeTf(), local_xy)) ==
    swri_transform_util::Transform(
      std::make_shared<swri_transform_util::Wgs84ToTfTransform>(later, local_xy)));
}

TEST(TransformEqualityTests, OppositeDirectionsAreNotEquivalent)
{
  auto local_xy = std::make_shared<swri_transform_util::LocalXyWgs84Util>(29.45, -98.6);

  swri_transform_util::Transform to_tf(
    std::make_shared<swri_transform_util::Wgs84ToTfTransform>(MakeTf(), local_xy));
  swri_transform_util::Transform to_wgs84(
    std::make_shared<swri_transform_util::TfToWgs84Transform>(MakeTf(), local_xy));

  EXPECT_TRUE(to_tf != to_wgs84);
  EXPECT_TRUE(to_wgs84 != to_tf);
}

TEST(LocalXyEquivalenceTests, ComparesReferencePoints)
{
  using swri_transform_util::AreEquivalent;
  using swri_transform_util::LocalXyWgs84Util;

  auto origin = std::make_shared<LocalXyWgs84Util>(29.45, -98.6);

  EXPECT_TRUE(AreEquivalent(origin, origin));
  EXPECT_TRUE(AreEquivalent(origin, std::make_shared<LocalXyWgs84Util>(29.45, -98.6)));
  EXPECT_FALSE(AreEquivalent(origin, std::make_shared<LocalXyWgs84Util>(29.45, -98.7)));
  EXPECT_FALSE(AreEquivalent(origin, std::make_shared<LocalXyWgs84Util>(0.0, 0.0)));
  EXPECT_FALSE(AreEquivalent(origin, nullptr));
  EXPECT_TRUE(AreEquivalent(nullptr, nullptr));
}
