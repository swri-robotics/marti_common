// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-R8248
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

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include <tf/transform_datatypes.h>

#include <math_util/constants.h>
#include <image_util/geometry_util.h>

TEST(GeometryUtilTests, Intersects)
{
  image_util::BoundingBox b1(-2, -2, 4, 4);
  image_util::BoundingBox b2(-1, -1, 2, 2);
  image_util::BoundingBox b3(-4, -4, 2.5, 2.5);
  image_util::BoundingBox b4(1.5, 1.5, 6, 6);
  image_util::BoundingBox b5(4, 4, 6, 6);

  EXPECT_TRUE(image_util::Intersects(b1, b2));
  EXPECT_TRUE(image_util::Intersects(b1, b3));
  EXPECT_TRUE(image_util::Intersects(b1, b4));
  EXPECT_FALSE(image_util::Intersects(b1, b5));

  EXPECT_TRUE(image_util::Intersects(b2, b1));
  EXPECT_FALSE(image_util::Intersects(b2, b3));
  EXPECT_FALSE(image_util::Intersects(b2, b4));
  EXPECT_FALSE(image_util::Intersects(b2, b5));

  EXPECT_TRUE(image_util::Intersects(b3, b1));
  EXPECT_FALSE(image_util::Intersects(b3, b2));
  EXPECT_FALSE(image_util::Intersects(b3, b4));
  EXPECT_FALSE(image_util::Intersects(b3, b5));

  EXPECT_TRUE(image_util::Intersects(b4, b1));
  EXPECT_FALSE(image_util::Intersects(b4, b2));
  EXPECT_FALSE(image_util::Intersects(b4, b3));
  EXPECT_TRUE(image_util::Intersects(b4, b5));

  EXPECT_FALSE(image_util::Intersects(b5, b1));
  EXPECT_FALSE(image_util::Intersects(b5, b2));
  EXPECT_FALSE(image_util::Intersects(b5, b3));
  EXPECT_TRUE(image_util::Intersects(b5, b4));
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

  EXPECT_FLOAT_EQ(100, image_util::GetOverlappingArea(rect, identity));
  EXPECT_FLOAT_EQ(200, image_util::GetOverlappingArea(rect2, identity));

  cv::Mat shift_15_x(2, 3, CV_32FC1);
  shift_15_x.at<float>(0,0) = 1;
  shift_15_x.at<float>(0,1) = 0;
  shift_15_x.at<float>(0,2) = 15;
  shift_15_x.at<float>(1,0) = 0;
  shift_15_x.at<float>(1,1) = 1;
  shift_15_x.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(0, image_util::GetOverlappingArea(rect, shift_15_x));
  EXPECT_FLOAT_EQ(50, image_util::GetOverlappingArea(rect2, shift_15_x));

  cv::Mat shift_5_x(2, 3, CV_32FC1);
  shift_5_x.at<float>(0,0) = 1;
  shift_5_x.at<float>(0,1) = 0;
  shift_5_x.at<float>(0,2) = 5;
  shift_5_x.at<float>(1,0) = 0;
  shift_5_x.at<float>(1,1) = 1;
  shift_5_x.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(50, image_util::GetOverlappingArea(rect, shift_5_x));
  EXPECT_FLOAT_EQ(150, image_util::GetOverlappingArea(rect2, shift_5_x));

  cv::Mat shift_5_xy(2, 3, CV_32FC1);
  shift_5_xy.at<float>(0,0) = 1;
  shift_5_xy.at<float>(0,1) = 0;
  shift_5_xy.at<float>(0,2) = 5;
  shift_5_xy.at<float>(1,0) = 0;
  shift_5_xy.at<float>(1,1) = 1;
  shift_5_xy.at<float>(1,2) = 5;

  EXPECT_FLOAT_EQ(25, image_util::GetOverlappingArea(rect, shift_5_xy));
  EXPECT_FLOAT_EQ(75, image_util::GetOverlappingArea(rect2, shift_5_xy));
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

  EXPECT_FLOAT_EQ(100, image_util::GetOverlappingArea(rect, rotate90));
  EXPECT_FLOAT_EQ(100, image_util::GetOverlappingArea(rect2, rotate90));

  cv::Mat rotate180(2, 3, CV_32FC1);
  rotate180.at<float>(0,0) = -1;
  rotate180.at<float>(0,1) = 0;
  rotate180.at<float>(0,2) = 0;
  rotate180.at<float>(1,0) = 0;
  rotate180.at<float>(1,1) = -1;
  rotate180.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(100, image_util::GetOverlappingArea(rect, rotate180));
  EXPECT_FLOAT_EQ(200, image_util::GetOverlappingArea(rect2, rotate180));

  cv::Mat rotate45(2, 3, CV_32FC1);
  rotate45.at<float>(0,0) = std::cos(math_util::_half_pi * 0.5);
  rotate45.at<float>(0,1) = std::sin(math_util::_half_pi * 0.5);
  rotate45.at<float>(0,2) = 0;
  rotate45.at<float>(1,0) = -std::sin(math_util::_half_pi * 0.5);
  rotate45.at<float>(1,1) = std::cos(math_util::_half_pi * 0.5);
  rotate45.at<float>(1,2) = 0;

  EXPECT_FLOAT_EQ(82.842712, image_util::GetOverlappingArea(rect, rotate45));
  EXPECT_FLOAT_EQ(136.3961, image_util::GetOverlappingArea(rect2, rotate45));
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

  cv::Mat projected1 = image_util::ProjectEllipsoid(ellipsoid1);

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

  cv::Mat projected1 = image_util::ProjectEllipsoid(ellipsoid1);

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

  cv::Mat projected1 = image_util::ProjectEllipsoid(ellipsoid1);

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

  cv::Mat projected2 = image_util::ProjectEllipsoid(ellipsoid2);

  EXPECT_TRUE(projected2.empty());
}

TEST(GeometryUtilTests, TestProjectEllipseInvalid2)
{
  cv::Mat ellipsoid2(2, 2, CV_32FC1);
  ellipsoid2.at<float>(0,0) = 1;
  ellipsoid2.at<float>(0,1) = 0;
  ellipsoid2.at<float>(1,0) = 0;
  ellipsoid2.at<float>(1,1) = 1;

  cv::Mat projected2 = image_util::ProjectEllipsoid(ellipsoid2);

  EXPECT_TRUE(projected2.empty());
}

TEST(GeometryUtilTests, TestProjectEllipseInvalid3)
{
  cv::Mat ellipsoid2(2, 2, CV_32SC1);
  ellipsoid2.at<int32_t>(0,0) = 1;
  ellipsoid2.at<int32_t>(0,1) = 0;
  ellipsoid2.at<int32_t>(1,0) = 0;
  ellipsoid2.at<int32_t>(1,1) = 1;

  cv::Mat projected2 = image_util::ProjectEllipsoid(ellipsoid2);

  EXPECT_TRUE(projected2.empty());
}

TEST(GeometryUtilTests, TestGetEllipsePoints1)
{
  cv::Mat ellipse(2, 2, CV_32FC1);
  ellipse.at<float>(0,0) = 1;
  ellipse.at<float>(0,1) = 0;
  ellipse.at<float>(1,0) = 0;
  ellipse.at<float>(1,1) = 1;

  std::vector<tf::Vector3> points = image_util::GetEllipsePoints(
      ellipse, tf::Vector3(0, 0, 0), 1, 8);

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
