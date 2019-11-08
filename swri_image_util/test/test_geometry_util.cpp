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

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include <tf2/transform_datatypes.h>

#include <swri_math_util/constants.h>
#include <swri_image_util/geometry_util.h>

TEST(GeometryUtilTests, Intersects)
{
  swri_image_util::BoundingBox b1(-2, -2, 4, 4);
  swri_image_util::BoundingBox b2(-1, -1, 2, 2);
  swri_image_util::BoundingBox b3(-4, -4, 2.5, 2.5);
  swri_image_util::BoundingBox b4(1.5, 1.5, 6, 6);
  swri_image_util::BoundingBox b5(4, 4, 6, 6);

  EXPECT_TRUE(swri_image_util::Intersects(b1, b2));
  EXPECT_TRUE(swri_image_util::Intersects(b1, b3));
  EXPECT_TRUE(swri_image_util::Intersects(b1, b4));
  EXPECT_FALSE(swri_image_util::Intersects(b1, b5));

  EXPECT_TRUE(swri_image_util::Intersects(b2, b1));
  EXPECT_FALSE(swri_image_util::Intersects(b2, b3));
  EXPECT_FALSE(swri_image_util::Intersects(b2, b4));
  EXPECT_FALSE(swri_image_util::Intersects(b2, b5));

  EXPECT_TRUE(swri_image_util::Intersects(b3, b1));
  EXPECT_FALSE(swri_image_util::Intersects(b3, b2));
  EXPECT_FALSE(swri_image_util::Intersects(b3, b4));
  EXPECT_FALSE(swri_image_util::Intersects(b3, b5));

  EXPECT_TRUE(swri_image_util::Intersects(b4, b1));
  EXPECT_FALSE(swri_image_util::Intersects(b4, b2));
  EXPECT_FALSE(swri_image_util::Intersects(b4, b3));
  EXPECT_TRUE(swri_image_util::Intersects(b4, b5));

  EXPECT_FALSE(swri_image_util::Intersects(b5, b1));
  EXPECT_FALSE(swri_image_util::Intersects(b5, b2));
  EXPECT_FALSE(swri_image_util::Intersects(b5, b3));
  EXPECT_TRUE(swri_image_util::Intersects(b5, b4));
}

TEST(GeometryUtilTests, GetOverlappingArea1)
{
  cv::Rect rect(-5, -5, 10, 10);
  cv::Rect rect2(-10, -5, 20, 10);

  cv::Mat identity(2, 3, CV_32FC1);
  identity.at<float>(0,0) = 1;
  identity.at<float>(0,1) = 0;
  identity.at<float>(0,2) = 0;
  identity.at<float>(1,0) = 0;
  identity.at<float>(1,1) = 1;
  identity.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(100, swri_image_util::GetOverlappingArea(rect, identity));
  EXPECT_FLOAT_EQ(200, swri_image_util::GetOverlappingArea(rect2, identity));

  cv::Mat shift_15_x(2, 3, CV_32FC1);
  shift_15_x.at<float>(0,0) = 1;
  shift_15_x.at<float>(0,1) = 0;
  shift_15_x.at<float>(0,2) = 15;
  shift_15_x.at<float>(1,0) = 0;
  shift_15_x.at<float>(1,1) = 1;
  shift_15_x.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(0, swri_image_util::GetOverlappingArea(rect, shift_15_x));
  EXPECT_FLOAT_EQ(50, swri_image_util::GetOverlappingArea(rect2, shift_15_x));

  cv::Mat shift_5_x(2, 3, CV_32FC1);
  shift_5_x.at<float>(0,0) = 1;
  shift_5_x.at<float>(0,1) = 0;
  shift_5_x.at<float>(0,2) = 5;
  shift_5_x.at<float>(1,0) = 0;
  shift_5_x.at<float>(1,1) = 1;
  shift_5_x.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(50, swri_image_util::GetOverlappingArea(rect, shift_5_x));
  EXPECT_FLOAT_EQ(150, swri_image_util::GetOverlappingArea(rect2, shift_5_x));

  cv::Mat shift_5_xy(2, 3, CV_32FC1);
  shift_5_xy.at<float>(0,0) = 1;
  shift_5_xy.at<float>(0,1) = 0;
  shift_5_xy.at<float>(0,2) = 5;
  shift_5_xy.at<float>(1,0) = 0;
  shift_5_xy.at<float>(1,1) = 1;
  shift_5_xy.at<float>(1,2) = 5;

  EXPECT_FLOAT_EQ(25, swri_image_util::GetOverlappingArea(rect, shift_5_xy));
  EXPECT_FLOAT_EQ(75, swri_image_util::GetOverlappingArea(rect2, shift_5_xy));
}

TEST(GeometryUtilTests, GetOverlappingArea2)
{
  cv::Rect rect(-5, -5, 10, 10);
  cv::Rect rect2(-10, -5, 20, 10);

  cv::Mat rotate90(2, 3, CV_32FC1);
  rotate90.at<float>(0,0) = 0;
  rotate90.at<float>(0,1) = 1;
  rotate90.at<float>(0,2) = 0;
  rotate90.at<float>(1,0) = -1;
  rotate90.at<float>(1,1) = 0;
  rotate90.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(100, swri_image_util::GetOverlappingArea(rect, rotate90));
  EXPECT_FLOAT_EQ(100, swri_image_util::GetOverlappingArea(rect2, rotate90));

  cv::Mat rotate180(2, 3, CV_32FC1);
  rotate180.at<float>(0,0) = -1;
  rotate180.at<float>(0,1) = 0;
  rotate180.at<float>(0,2) = 0;
  rotate180.at<float>(1,0) = 0;
  rotate180.at<float>(1,1) = -1;
  rotate180.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(100, swri_image_util::GetOverlappingArea(rect, rotate180));
  EXPECT_FLOAT_EQ(200, swri_image_util::GetOverlappingArea(rect2, rotate180));

  cv::Mat rotate45(2, 3, CV_32FC1);
  rotate45.at<float>(0,0) = std::cos(swri_math_util::_half_pi * 0.5);
  rotate45.at<float>(0,1) = std::sin(swri_math_util::_half_pi * 0.5);
  rotate45.at<float>(0,2) = 0;
  rotate45.at<float>(1,0) = -std::sin(swri_math_util::_half_pi * 0.5);
  rotate45.at<float>(1,1) = std::cos(swri_math_util::_half_pi * 0.5);
  rotate45.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(82.842712, swri_image_util::GetOverlappingArea(rect, rotate45));
  EXPECT_FLOAT_EQ(136.3961, swri_image_util::GetOverlappingArea(rect2, rotate45));
}

TEST(GeometryUtilTests, TestProjectEllipse1)
{
  cv::Mat ellipsoid1(3, 3, CV_32FC1);
  ellipsoid1.at<float>(0,0) = 1;
  ellipsoid1.at<float>(0,1) = 0;
  ellipsoid1.at<float>(0,2) = 0;
  ellipsoid1.at<float>(1,0) = 0;
  ellipsoid1.at<float>(1,1) = 1;
  ellipsoid1.at<float>(1,2) = 0;
  ellipsoid1.at<float>(2,0) = 0;
  ellipsoid1.at<float>(2,1) = 0;
  ellipsoid1.at<float>(2,2) = 1;

  cv::Mat ellipse1(2, 2, CV_32FC1);
  ellipse1.at<float>(0,0) = 1;
  ellipse1.at<float>(0,1) = 0;
  ellipse1.at<float>(1,0) = 0;
  ellipse1.at<float>(1,1) = 1;

  cv::Mat projected1 = swri_image_util::ProjectEllipsoid(ellipsoid1);

  EXPECT_EQ(ellipse1.at<float>(0, 0), projected1.at<float>(0, 0));
  EXPECT_EQ(ellipse1.at<float>(0, 1), projected1.at<float>(0, 1));
  EXPECT_EQ(ellipse1.at<float>(1, 0), projected1.at<float>(1, 0));
  EXPECT_EQ(ellipse1.at<float>(1, 1), projected1.at<float>(1, 1));
}

TEST(GeometryUtilTests, TestProjectEllipse2)
{
  cv::Mat ellipsoid1(3, 3, CV_32FC1);
  ellipsoid1.at<float>(0,0) = 10;
  ellipsoid1.at<float>(0,1) = 0;
  ellipsoid1.at<float>(0,2) = 0;
  ellipsoid1.at<float>(1,0) = 0;
  ellipsoid1.at<float>(1,1) = 15;
  ellipsoid1.at<float>(1,2) = 0;
  ellipsoid1.at<float>(2,0) = 0;
  ellipsoid1.at<float>(2,1) = 0;
  ellipsoid1.at<float>(2,2) = -35;

  cv::Mat ellipse1(2, 2, CV_32FC1);
  ellipse1.at<float>(0,0) = 10;
  ellipse1.at<float>(0,1) = 0;
  ellipse1.at<float>(1,0) = 0;
  ellipse1.at<float>(1,1) = 15;

  cv::Mat projected1 = swri_image_util::ProjectEllipsoid(ellipsoid1);

  EXPECT_EQ(ellipse1.at<float>(0, 0), projected1.at<float>(0, 0));
  EXPECT_EQ(ellipse1.at<float>(0, 1), projected1.at<float>(0, 1));
  EXPECT_EQ(ellipse1.at<float>(1, 0), projected1.at<float>(1, 0));
  EXPECT_EQ(ellipse1.at<float>(1, 1), projected1.at<float>(1, 1));
}

TEST(GeometryUtilTests, TestProjectEllipse3)
{
  cv::Mat ellipsoid1(3, 3, CV_32FC1);
  ellipsoid1.at<float>(0,0) = 10;
  ellipsoid1.at<float>(0,1) = 0;
  ellipsoid1.at<float>(0,2) = 0;
  ellipsoid1.at<float>(1,0) = 0;
  ellipsoid1.at<float>(1,1) = 15;
  ellipsoid1.at<float>(1,2) = 0;
  ellipsoid1.at<float>(2,0) = 0;
  ellipsoid1.at<float>(2,1) = 0;
  ellipsoid1.at<float>(2,2) = std::numeric_limits<double>::max() * 0.5;

  cv::Mat ellipse1(2, 2, CV_32FC1);
  ellipse1.at<float>(0,0) = 10;
  ellipse1.at<float>(0,1) = 0;
  ellipse1.at<float>(1,0) = 0;
  ellipse1.at<float>(1,1) = 15;

  cv::Mat projected1 = swri_image_util::ProjectEllipsoid(ellipsoid1);

  EXPECT_EQ(ellipse1.at<float>(0, 0), projected1.at<float>(0, 0));
  EXPECT_EQ(ellipse1.at<float>(0, 1), projected1.at<float>(0, 1));
  EXPECT_EQ(ellipse1.at<float>(1, 0), projected1.at<float>(1, 0));
  EXPECT_EQ(ellipse1.at<float>(1, 1), projected1.at<float>(1, 1));
}


// TODO(malban): Test projecting an ellipsoid that is not axis aligned.

TEST(GeometryUtilTests, TestProjectEllipseInvalid1)
{
  cv::Mat ellipsoid2(3, 3, CV_32FC1);
  ellipsoid2.at<float>(0,0) = 1;
  ellipsoid2.at<float>(0,1) = 0;
  ellipsoid2.at<float>(0,2) = 0;
  ellipsoid2.at<float>(1,0) = 0;
  ellipsoid2.at<float>(1,1) = 1;
  ellipsoid2.at<float>(1,2) = 0;
  ellipsoid2.at<float>(2,0) = 0;
  ellipsoid2.at<float>(2,1) = 0;
  ellipsoid2.at<float>(2,2) = 0;

  cv::Mat projected2 = swri_image_util::ProjectEllipsoid(ellipsoid2);

  EXPECT_TRUE(projected2.empty());
}

TEST(GeometryUtilTests, TestProjectEllipseInvalid2)
{
  cv::Mat ellipsoid2(2, 2, CV_32FC1);
  ellipsoid2.at<float>(0,0) = 1;
  ellipsoid2.at<float>(0,1) = 0;
  ellipsoid2.at<float>(1,0) = 0;
  ellipsoid2.at<float>(1,1) = 1;

  cv::Mat projected2 = swri_image_util::ProjectEllipsoid(ellipsoid2);

  EXPECT_TRUE(projected2.empty());
}

TEST(GeometryUtilTests, TestProjectEllipseInvalid3)
{
  cv::Mat ellipsoid2(2, 2, CV_32SC1);
  ellipsoid2.at<int32_t>(0,0) = 1;
  ellipsoid2.at<int32_t>(0,1) = 0;
  ellipsoid2.at<int32_t>(1,0) = 0;
  ellipsoid2.at<int32_t>(1,1) = 1;

  cv::Mat projected2 = swri_image_util::ProjectEllipsoid(ellipsoid2);

  EXPECT_TRUE(projected2.empty());
}

TEST(GeometryUtilTests, TestGetEllipsePoints1)
{
  cv::Mat ellipse(2, 2, CV_32FC1);
  ellipse.at<float>(0,0) = 1;
  ellipse.at<float>(0,1) = 0;
  ellipse.at<float>(1,0) = 0;
  ellipse.at<float>(1,1) = 1;

  std::vector<tf2::Vector3> points = swri_image_util::GetEllipsePoints(
      ellipse, tf2::Vector3(0, 0, 0), 1, 8);

  ASSERT_EQ(8, points.size());

  EXPECT_FLOAT_EQ(1, points[0].x());
  EXPECT_NEAR(0, points[0].y(), 0.000000001);

  EXPECT_NEAR(0.7071067811865475243818940365, points[1].x(),  0.000000001);
  EXPECT_NEAR(0.7071067811865475243818940365, points[1].y(),  0.000000001);

  EXPECT_NEAR(0, points[2].x(), 0.000000001);
  EXPECT_FLOAT_EQ(1, points[2].y());

  EXPECT_NEAR(-0.7071067811865475243818940365, points[3].x(),  0.000000001);
  EXPECT_NEAR(0.7071067811865475243818940365, points[3].y(),  0.000000001);

  EXPECT_FLOAT_EQ(-1, points[4].x());
  EXPECT_NEAR(0, points[4].y(), 0.000000001);

  EXPECT_NEAR(-0.7071067811865475243818940365, points[5].x(),  0.000000001);
  EXPECT_NEAR(-0.7071067811865475243818940365, points[5].y(),  0.000000001);

  EXPECT_NEAR(0, points[6].x(), 0.000000001);
  EXPECT_FLOAT_EQ(-1, points[6].y());

  EXPECT_NEAR(0.7071067811865475243818940365, points[7].x(),  0.000000001);
  EXPECT_NEAR(-0.7071067811865475243818940365, points[7].y(),  0.000000001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
