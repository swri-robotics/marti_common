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
#include <cstdlib>

#include <gtest/gtest.h>

#include <opencv2/core/core.hpp>

#include <math_util/constants.h>
#include <image_util/motion_estimation.h>

TEST(ImageUtilTests, TestLeastSquaresRigid2D_1)
{
  std::srand(0);

  cv::Mat transform(2, 3, CV_32F);
  transform.at<float>(0,0) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(0,1) = std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(0,2) = -10;
  transform.at<float>(1,0) = -std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(1,1) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(1,2) = 15;

  cv::Mat p1(1, 1000, CV_32FC2);

  for (int32_t i = 0; i < p1.cols; i++)
  {
    p1.at<cv::Vec2f>(0, i)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec2f>(0, i)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.cols; i++)
  {
    p2.at<cv::Vec2f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec2f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  cv::Mat estimated = image_util::LeastSqauresRigid2DTransform(p1, p2);

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .0005);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .0005);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .01);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .0005);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .0005);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .01);
}

TEST(ImageUtilTests, TestLeastSquaresRigid2D_2)
{
  std::srand(0);

  cv::Mat transform(2, 3, CV_32F);
  transform.at<float>(0,0) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(0,1) = std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(0,2) = -10;
  transform.at<float>(1,0) = -std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(1,1) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(1,2) = 15;

  cv::Mat p1(1000, 1, CV_32FC2);

  for (int32_t i = 0; i < p1.rows; i++)
  {
    p1.at<cv::Vec2f>(i, 0)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec2f>(i, 0)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.rows; i++)
  {
    p2.at<cv::Vec2f>(i, 0)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec2f>(i, 0)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  cv::Mat estimated = image_util::LeastSqauresRigid2DTransform(p1, p2);

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .0005);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .0005);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .01);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .0005);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .0005);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .01);
}

TEST(ImageUtilTests, TestComputeRigid2D_1)
{
  std::srand(0);

  cv::Mat transform(2, 3, CV_32F);
  transform.at<float>(0,0) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(0,1) = std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(0,2) = -10;
  transform.at<float>(1,0) = -std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(1,1) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(1,2) = 15;

  cv::Mat p1(1, 1000, CV_32FC2);

  for (int32_t i = 0; i < p1.cols; i++)
  {
    p1.at<cv::Vec2f>(0, i)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec2f>(0, i)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.cols; i++)
  {
    p2.at<cv::Vec2f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec2f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  cv::Mat estimated = image_util::ComputeRigid2DTransform(p1, p2);

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .0005);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .0005);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .01);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .0005);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .0005);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .01);
}

TEST(ImageUtilTests, TestComputeRigid2D_2)
{
  std::srand(0);

  cv::Mat transform(2, 3, CV_32F);
  transform.at<float>(0,0) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(0,1) = std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(0,2) = -10;
  transform.at<float>(1,0) = -std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(1,1) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(1,2) = 15;

  cv::Mat p1(1000, 1, CV_32FC2);

  for (int32_t i = 0; i < p1.rows; i++)
  {
    p1.at<cv::Vec2f>(i, 0)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec2f>(i, 0)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.rows; i++)
  {
    p2.at<cv::Vec2f>(i, 0)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec2f>(i, 0)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  cv::Mat estimated = image_util::ComputeRigid2DTransform(p1, p2);

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .0005);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .0005);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .01);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .0005);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .0005);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .01);
}

TEST(ImageUtilTests, TestComputeRigid2D_3)
{
  std::srand(0);

  cv::Mat transform(2, 3, CV_32F);
  transform.at<float>(0,0) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(0,1) = std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(0,2) = -10;
  transform.at<float>(1,0) = -std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(1,1) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(1,2) = 15;

  cv::Mat p1(1000, 1, CV_32FC2);

  for (int32_t i = 0; i < p1.rows; i++)
  {
    p1.at<cv::Vec2f>(i, 0)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec2f>(i, 0)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.rows; i++)
  {
    p2.at<cv::Vec2f>(i, 0)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec2f>(i, 0)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  for (int32_t i = 0; i < p2.rows; i+=5)
  {
    p2.at<cv::Vec2f>(i, 0)[0] += ((double)std::rand() / RAND_MAX) * 30 - 15;
    p2.at<cv::Vec2f>(i, 0)[1] += ((double)std::rand() / RAND_MAX) * 30 - 15;
  }

  cv::Mat estimated = image_util::ComputeRigid2DTransform(p1, p2);

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .006);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .006);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .3);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .006);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .006);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .3);
}

TEST(ImageUtilTests, TestComputeRigid2D_4)
{
  std::srand(0);

  cv::Mat transform(2, 3, CV_32F);
  transform.at<float>(0,0) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(0,1) = std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(0,2) = -10;
  transform.at<float>(1,0) = -std::sin(math_util::_half_pi * 0.5);
  transform.at<float>(1,1) = std::cos(math_util::_half_pi * 0.5);
  transform.at<float>(1,2) = 15;

  cv::Mat p1(1000, 1, CV_32FC2);

  for (int32_t i = 0; i < p1.rows; i++)
  {
    p1.at<cv::Vec2f>(i, 0)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec2f>(i, 0)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.rows; i++)
  {
    p2.at<cv::Vec2f>(i, 0)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec2f>(i, 0)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  for (int32_t i = 0; i < p2.rows; i+=5)
  {
    p2.at<cv::Vec2f>(i, 0)[0] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
    p2.at<cv::Vec2f>(i, 0)[1] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
  }

  cv::Mat estimated = image_util::ComputeRigid2DTransform(p1, p2);

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .0005);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .0005);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .01);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .0005);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .0005);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .01);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
