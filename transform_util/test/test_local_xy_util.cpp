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

#include <transform_util/local_xy_util.h>

TEST(LocalXyUtilTests, TestOrigin)
{
  transform_util::LocalXyWgs84Util local_xy_util(29.45196669, -98.61370577);

  EXPECT_FLOAT_EQ(29.45196669, local_xy_util.ReferenceLatitude());
  EXPECT_FLOAT_EQ(-98.61370577, local_xy_util.ReferenceLongitude());

  double x, y;
  local_xy_util.ToLocalXy(29.45196669, -98.61370577, x, y);
  EXPECT_FLOAT_EQ(0, x);
  EXPECT_FLOAT_EQ(0, y);

  double lat, lon;
  local_xy_util.ToWgs84(0, 0, lat, lon);
  EXPECT_FLOAT_EQ(29.45196669, lat);
  EXPECT_FLOAT_EQ(-98.61370577, lon);
}

TEST(LocalXyUtilTests, TestOffset1)
{
  transform_util::LocalXyWgs84Util local_xy_util(29.45196669, -98.61370577);

  double x = 26.3513;
  double y = 4633.46;

  double lat, lon;
  local_xy_util.ToWgs84(x, y, lat, lon);
  EXPECT_NE(local_xy_util.ReferenceLatitude(), lat);
  EXPECT_NE(local_xy_util.ReferenceLongitude(), lon);

  EXPECT_FLOAT_EQ(29.4937686007, lat);
  EXPECT_FLOAT_EQ(-98.6134341407, lon);

  double x2, y2;
  local_xy_util.ToLocalXy(lat, lon, x2, y2);
  EXPECT_FLOAT_EQ(x, x2);
  EXPECT_FLOAT_EQ(y, y2);
}

TEST(LocalXyUtilTests, TestOffset2)
{
  transform_util::LocalXyWgs84Util local_xy_util(29.45196669, -98.61370577);

  double x = -4626.3513;
  double y = -97.46;

  double lat, lon;
  local_xy_util.ToWgs84(x, y, lat, lon);
  EXPECT_NE(local_xy_util.ReferenceLatitude(), lat);
  EXPECT_NE(local_xy_util.ReferenceLongitude(), lon);
  EXPECT_FLOAT_EQ(29.4510874304, lat);
  EXPECT_FLOAT_EQ(-98.6613942088, lon);

  double x2, y2;
  local_xy_util.ToLocalXy(lat, lon, x2, y2);
  EXPECT_FLOAT_EQ(x, x2);
  EXPECT_FLOAT_EQ(y, y2);
}

TEST(LocalXyUtilTests, LocalXyFromWgs84)
{
  double x, y;
  transform_util::LocalXyFromWgs84(
      29.4937686007, -98.6134341407,
      29.45196669, -98.61370577,
      x, y);
  EXPECT_FLOAT_EQ(26.3513, x);
  EXPECT_FLOAT_EQ(4633.46, y);

  transform_util::LocalXyFromWgs84(
      29.4510874304, -98.6613942088,
      29.45196669, -98.61370577,
      x, y);
  EXPECT_FLOAT_EQ(-4626.3513, x);
  EXPECT_FLOAT_EQ(-97.46, y);
}

TEST(LocalXyUtilTests, Wgs84FromLocalXy)
{
  double lat, lon;
  transform_util::Wgs84FromLocalXy(
      26.3513, 4633.46,
      29.45196669, -98.61370577,
      lat, lon);
  EXPECT_FLOAT_EQ(29.4937686007, lat);
  EXPECT_FLOAT_EQ(-98.6134341407, lon);

  transform_util::Wgs84FromLocalXy(
      -4626.3513, -97.46,
      29.45196669, -98.61370577,
      lat, lon);
  EXPECT_FLOAT_EQ(29.4510874304, lat);
  EXPECT_FLOAT_EQ(-98.6613942088, lon);
}

TEST(LocalXyUtilTests, Continuity)
{
  // (FOR) - Fortaleza International Airport
  transform_util::LocalXyWgs84Util local_xy_util(-3.775833, -38.532222);

  double x = 0;
  double y = 0;

  double last_lon = 0;

  for (int i = 0; i < 1000; i++)
  {
    double new_lat;
    double new_lon;
    double new_x;
    double new_y;

    local_xy_util.ToWgs84(x + (double)i * 1.11 / 100.0, y, new_lat, new_lon);
    local_xy_util.ToLocalXy(new_lat, new_lon, new_x, new_y);

    EXPECT_FLOAT_EQ(x + (double)i * 1.11 / 100.0, new_x);
    EXPECT_FLOAT_EQ(y, new_y);

    if (i > 0)
    {
      // The difference should be 1.11cm which is approximately
      // 1/10th of 1 microdegree near the equator
      EXPECT_NEAR(0.0000001, std::fabs(new_lon - last_lon), 0.00000001);
    }

    last_lon = new_lon;
  }
}

TEST(LocalXyUtilTests, TestFrameId)
{
  transform_util::LocalXyWgs84Util local_xy_util(
      29.45196669,
      -98.61370577,
      0, 0,
      "/local_xy_frame");

  EXPECT_EQ(std::string("/local_xy_frame"), local_xy_util.FrameId());

  transform_util::LocalXyWgs84Util local_xy_util2(
      29.45196669,
      -98.61370577);

  EXPECT_EQ(std::string(""), local_xy_util2.FrameId());
}

TEST(LocalXyUtilTests, TestParseOrigin)
{
  transform_util::LocalXyWgs84UtilPtr local_xy_util =
      transform_util::ParseLocalXyOrigin();

  ASSERT_TRUE(local_xy_util);

  EXPECT_FLOAT_EQ(29.45196669, local_xy_util->ReferenceLatitude());
  EXPECT_FLOAT_EQ(-98.61370577, local_xy_util->ReferenceLongitude());
  EXPECT_FLOAT_EQ(233.719, local_xy_util->ReferenceAltitude());
  EXPECT_FLOAT_EQ(0, local_xy_util->ReferenceHeading());

  double x, y;
  local_xy_util->ToLocalXy(29.45196669, -98.61370577, x, y);
  EXPECT_FLOAT_EQ(0, x);
  EXPECT_FLOAT_EQ(0, y);

  double lat, lon;
  local_xy_util->ToWgs84(0, 0, lat, lon);
  EXPECT_FLOAT_EQ(29.45196669, lat);
  EXPECT_FLOAT_EQ(-98.61370577, lon);

  EXPECT_EQ(std::string("/far_field"), local_xy_util->FrameId());
}

TEST(LocalXyUtilTests, TestKnownOffset)
{
  transform_util::LocalXyWgs84UtilPtr local_xy_util =
      transform_util::ParseLocalXyOrigin();

  ASSERT_TRUE(local_xy_util);

  double x, y;
  local_xy_util->ToLocalXy(29.4564773982, -98.6085519577, x, y);
  EXPECT_FLOAT_EQ(500, x);
  EXPECT_FLOAT_EQ(500, y);

  double lat, lon;

  local_xy_util->ToWgs84(500, 500, lat, lon);
  EXPECT_FLOAT_EQ(29.4564773982, lat);
  EXPECT_FLOAT_EQ(-98.6085519577, lon);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Initialize the ROS core parameters can be loaded from the launch file
  ros::init(argc, argv, "test_local_xy_util");

  return RUN_ALL_TESTS();
}
