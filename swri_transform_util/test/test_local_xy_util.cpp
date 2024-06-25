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

#include <rclcpp/rclcpp.hpp>

#include <swri_transform_util/local_xy_util.h>

TEST(LocalXyUtilTests, TestOrigin)
{
  swri_transform_util::LocalXyWgs84Util local_xy_util(29.45196669, -98.61370577);

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
  swri_transform_util::LocalXyWgs84Util local_xy_util(29.45196669, -98.61370577);

  double x = 26.3513;
  double y = 4633.46;

  double lat, lon;
  local_xy_util.ToWgs84(x, y, lat, lon);
  EXPECT_NE(local_xy_util.ReferenceLatitude(), lat);
  EXPECT_NE(local_xy_util.ReferenceLongitude(), lon);

  EXPECT_FLOAT_EQ(29.4937684617, lat);
  EXPECT_FLOAT_EQ(-98.6134340294, lon);

  double x2, y2;
  local_xy_util.ToLocalXy(lat, lon, x2, y2);
  EXPECT_FLOAT_EQ(x, x2);
  EXPECT_FLOAT_EQ(y, y2);
}

TEST(LocalXyUtilTests, TestOffset2)
{
  swri_transform_util::LocalXyWgs84Util local_xy_util(29.45196669, -98.61370577);

  double x = -4626.3513;
  double y = -97.46;

  double lat, lon;
  local_xy_util.ToWgs84(x, y, lat, lon);
  EXPECT_NE(local_xy_util.ReferenceLatitude(), lat);
  EXPECT_NE(local_xy_util.ReferenceLongitude(), lon);
  EXPECT_FLOAT_EQ(29.4510788901, lat);
  EXPECT_FLOAT_EQ(-98.6613937867, lon);

  double x2, y2;
  local_xy_util.ToLocalXy(lat, lon, x2, y2);
  EXPECT_FLOAT_EQ(x, x2);
  EXPECT_FLOAT_EQ(y, y2);
}


TEST(LocalXyUtilTests, TestOffset3)
{
  // Set origin at dateline
  swri_transform_util::LocalXyWgs84Util local_xy_util(0, -180);

  double x = -100.0;
  double y = 10.0;

  // Offset is west, across the dateline
  double lat, lon;
  local_xy_util.ToWgs84(x, y, lat, lon);
  EXPECT_NEAR(0.00009045, lat, 0.0000001);  // ~1cm accuracy
  EXPECT_NEAR(179.9991017, lon, 0.0000001);

  double x2, y2;
  local_xy_util.ToLocalXy(lat, lon, x2, y2);
  EXPECT_FLOAT_EQ(x, x2);
  EXPECT_FLOAT_EQ(y, y2);
}

TEST(LocalXyUtilTests, TestOffset4)
{
  // Set origin just west of dateline
  swri_transform_util::LocalXyWgs84Util local_xy_util(0, 179.9999);

  double x = 100.0;
  double y = -10.0;

  // Offset is east, across the dateline
  double lat, lon;
  local_xy_util.ToWgs84(x, y, lat, lon);
  EXPECT_NEAR(-0.00009045, lat, 0.0000001);  // ~1cm accuracy
  EXPECT_NEAR(-179.9992017, lon, 0.0000001);

  double x2, y2;
  local_xy_util.ToLocalXy(lat, lon, x2, y2);
  EXPECT_FLOAT_EQ(x, x2);
  EXPECT_FLOAT_EQ(y, y2);
}

TEST(LocalXyUtilTests, LocalXyFromWgs84)
{
  double x, y;
  swri_transform_util::LocalXyFromWgs84(
      29.4937686007, -98.6134341407,
      29.45196669, -98.61370577,
      x, y);
  EXPECT_FLOAT_EQ(26.3405, x);
  EXPECT_FLOAT_EQ(4633.4741, y);

  swri_transform_util::LocalXyFromWgs84(
      29.4510874304, -98.6613942088,
      29.45196669, -98.61370577,
      x, y);
  EXPECT_FLOAT_EQ(-4626.3906, x);
  EXPECT_FLOAT_EQ(-96.5133, y);
}

TEST(LocalXyUtilTests, Wgs84FromLocalXy)
{
  double lat, lon;
  swri_transform_util::Wgs84FromLocalXy(
      26.3513, 4633.46,
      29.45196669, -98.61370577,
      lat, lon);
  EXPECT_FLOAT_EQ(29.4937684617, lat);
  EXPECT_FLOAT_EQ(-98.6134340294, lon);

  swri_transform_util::Wgs84FromLocalXy(
      -4626.3513, -97.46,
      29.45196669, -98.61370577,
      lat, lon);
  EXPECT_FLOAT_EQ(29.4510788901, lat);
  EXPECT_FLOAT_EQ(-98.6613937867, lon);
}

TEST(LocalXyUtilTests, Continuity)
{
  // (FOR) - Fortaleza International Airport
  swri_transform_util::LocalXyWgs84Util local_xy_util(-3.775833, -38.532222);

  double x = 0;
  double y = 0;

  double last_lon = 0;

  for (int i = 0; i < 1000; i++)
  {
    double new_lat;
    double new_lon;
    double new_x;
    double new_y;

    local_xy_util.ToWgs84(x + i * 1.11 / 100.0, y, new_lat, new_lon);
    local_xy_util.ToLocalXy(new_lat, new_lon, new_x, new_y);

    EXPECT_FLOAT_EQ(x + i * 1.11 / 100.0, new_x);
    EXPECT_NEAR(y, new_y, 1e-9);

    if (i > 0)
    {
      // The difference should be 1.11cm which is approximately
      // 1/10th of 1 microdegree near the equator
      EXPECT_NEAR(0.0000001, std::fabs(new_lon - last_lon), 0.00000001);
    }

    last_lon = new_lon;
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Initialize the ROS core parameters can be loaded from the launch file
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
