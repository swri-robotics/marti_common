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

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include <image_util/geometry_util.h>

TEST(ImageUtilTests, Intersects)
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

TEST(ImageUtilTests, GetOverlappingArea1)
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

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
