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

TEST(ImageUtilTests, TestComputeRigid3D_1)
{
  std::srand(0);

  cv::Mat transform(3, 4, CV_32F);
  transform.at<float>(0,0) = 1;
  transform.at<float>(0,1) = 0;
  transform.at<float>(0,2) = 0;
  transform.at<float>(0,3) = 0;
  transform.at<float>(1,0) = 0;
  transform.at<float>(1,1) = 1;
  transform.at<float>(1,2) = 0;
  transform.at<float>(1,3) = 0;
  transform.at<float>(2,0) = 0;
  transform.at<float>(2,1) = 0;
  transform.at<float>(2,2) = 1;
  transform.at<float>(2,3) = 0;

  cv::Mat p1(1, 1000, CV_32FC3);

  for (int32_t i = 0; i < p1.cols; i++)
  {
    p1.at<cv::Vec3f>(0, i)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec3f>(0, i)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec3f>(0, i)[2] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.cols; i++)
  {
    p2.at<cv::Vec3f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec3f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec3f>(0, i)[2] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  cv::Mat estimated = image_util::ComputeRigid3DTransform(p1, p2);

  ASSERT_FALSE(estimated.empty());

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .005);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .005);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .005);
  EXPECT_NEAR(transform.at<float>(0,3), estimated.at<float>(0,3), .2);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .005);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .005);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .005);
  EXPECT_NEAR(transform.at<float>(1,3), estimated.at<float>(1,3), .2);
  EXPECT_NEAR(transform.at<float>(2,0), estimated.at<float>(2,0), .005);
  EXPECT_NEAR(transform.at<float>(2,1), estimated.at<float>(2,1), .005);
  EXPECT_NEAR(transform.at<float>(2,2), estimated.at<float>(2,2), .005);
  EXPECT_NEAR(transform.at<float>(2,3), estimated.at<float>(2,3), .2);
}

TEST(ImageUtilTests, TestComputeRigid3D_2)
{
  std::srand(0);

  cv::Mat transform(3, 4, CV_32F);
  transform.at<float>(0,0) = 1;
  transform.at<float>(0,1) = 0;
  transform.at<float>(0,2) = 0;
  transform.at<float>(0,3) = 0;
  transform.at<float>(1,0) = 0;
  transform.at<float>(1,1) = 1;
  transform.at<float>(1,2) = 0;
  transform.at<float>(1,3) = 0;
  transform.at<float>(2,0) = 0;
  transform.at<float>(2,1) = 0;
  transform.at<float>(2,2) = 1;
  transform.at<float>(2,3) = 0;

  cv::Mat p1(1, 1000, CV_32FC3);

  for (int32_t i = 0; i < p1.cols; i++)
  {
    p1.at<cv::Vec3f>(0, i)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec3f>(0, i)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec3f>(0, i)[2] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.cols; i++)
  {
    p2.at<cv::Vec3f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec3f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec3f>(0, i)[2] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  for (int32_t i = 0; i < p2.cols; i+=5)
  {
    p2.at<cv::Vec3f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
    p2.at<cv::Vec3f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
    p2.at<cv::Vec3f>(0, i)[2] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
  }

  cv::Mat estimated = image_util::ComputeRigid3DTransform(p1, p2);

  ASSERT_FALSE(estimated.empty());

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .007);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .007);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .007);
  EXPECT_NEAR(transform.at<float>(0,3), estimated.at<float>(0,3), .25);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .007);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .007);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .007);
  EXPECT_NEAR(transform.at<float>(1,3), estimated.at<float>(1,3), .25);
  EXPECT_NEAR(transform.at<float>(2,0), estimated.at<float>(2,0), .007);
  EXPECT_NEAR(transform.at<float>(2,1), estimated.at<float>(2,1), .007);
  EXPECT_NEAR(transform.at<float>(2,2), estimated.at<float>(2,2), .007);
  EXPECT_NEAR(transform.at<float>(2,3), estimated.at<float>(2,3), .25);
}

TEST(ImageUtilTests, TestComputeRigid3D_3)
{
  std::srand(0);

  cv::Mat transform(3, 4, CV_32F);
  transform.at<float>(0,0) = 1;
  transform.at<float>(0,1) = 0;
  transform.at<float>(0,2) = 0;
  transform.at<float>(0,3) = 20;
  transform.at<float>(1,0) = 0;
  transform.at<float>(1,1) = 1;
  transform.at<float>(1,2) = 0;
  transform.at<float>(1,3) = 25;
  transform.at<float>(2,0) = 0;
  transform.at<float>(2,1) = 0;
  transform.at<float>(2,2) = 1;
  transform.at<float>(2,3) = -15;

  cv::Mat p1(1, 1000, CV_32FC3);

  for (int32_t i = 0; i < p1.cols; i++)
  {
    p1.at<cv::Vec3f>(0, i)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec3f>(0, i)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec3f>(0, i)[2] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.cols; i++)
  {
    p2.at<cv::Vec3f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec3f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec3f>(0, i)[2] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  for (int32_t i = 0; i < p2.cols; i+=5)
  {
    p2.at<cv::Vec3f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
    p2.at<cv::Vec3f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
    p2.at<cv::Vec3f>(0, i)[2] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
  }

  cv::Mat estimated = image_util::ComputeRigid3DTransform(p1, p2);

  ASSERT_FALSE(estimated.empty());

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .007);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .007);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .007);
  EXPECT_NEAR(transform.at<float>(0,3), estimated.at<float>(0,3), .25);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .007);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .007);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .007);
  EXPECT_NEAR(transform.at<float>(1,3), estimated.at<float>(1,3), .25);
  EXPECT_NEAR(transform.at<float>(2,0), estimated.at<float>(2,0), .007);
  EXPECT_NEAR(transform.at<float>(2,1), estimated.at<float>(2,1), .007);
  EXPECT_NEAR(transform.at<float>(2,2), estimated.at<float>(2,2), .007);
  EXPECT_NEAR(transform.at<float>(2,3), estimated.at<float>(2,3), .25);
}

TEST(ImageUtilTests, TestComputeRigid3D_4)
{
  std::srand(0);

  cv::Mat transform(3, 4, CV_32F);
  transform.at<float>(0,0) = 0.492403876506104;
  transform.at<float>(0,1) = 0.586824088833465;
  transform.at<float>(0,2) = -0.642787609686539;
  transform.at<float>(0,3) = 20;
  transform.at<float>(1,0) = 0.413175911166535;
  transform.at<float>(1,1) = 0.492403876506104;
  transform.at<float>(1,2) = 0.766044443118978;
  transform.at<float>(1,3) = 25;
  transform.at<float>(2,0) = 0.766044443118978;
  transform.at<float>(2,1) = -0.642787609686539;
  transform.at<float>(2,2) = 0;
  transform.at<float>(2,3) = -15;

  cv::Mat p1(1, 1000, CV_32FC3);

  for (int32_t i = 0; i < p1.cols; i++)
  {
    p1.at<cv::Vec3f>(0, i)[0] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec3f>(0, i)[1] = ((double)std::rand() / RAND_MAX) * 100 - 50;
    p1.at<cv::Vec3f>(0, i)[2] = ((double)std::rand() / RAND_MAX) * 100 - 50;
  }

  cv::Mat p2;
  cv::transform(p1, p2, transform);

  for (int32_t i = 0; i < p2.cols; i++)
  {
    p2.at<cv::Vec3f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec3f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2 - 1;
    p2.at<cv::Vec3f>(0, i)[2] += ((double)std::rand() / RAND_MAX) * 2 - 1;
  }

  for (int32_t i = 0; i < p2.cols; i+=5)
  {
    p2.at<cv::Vec3f>(0, i)[0] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
    p2.at<cv::Vec3f>(0, i)[1] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
    p2.at<cv::Vec3f>(0, i)[2] += ((double)std::rand() / RAND_MAX) * 2000 - 1000;
  }

  cv::Mat estimated = image_util::ComputeRigid3DTransform(p1, p2);

  ASSERT_FALSE(estimated.empty());

  EXPECT_NEAR(transform.at<float>(0,0), estimated.at<float>(0,0), .008);
  EXPECT_NEAR(transform.at<float>(0,1), estimated.at<float>(0,1), .008);
  EXPECT_NEAR(transform.at<float>(0,2), estimated.at<float>(0,2), .008);
  EXPECT_NEAR(transform.at<float>(0,3), estimated.at<float>(0,3), .25);
  EXPECT_NEAR(transform.at<float>(1,0), estimated.at<float>(1,0), .008);
  EXPECT_NEAR(transform.at<float>(1,1), estimated.at<float>(1,1), .008);
  EXPECT_NEAR(transform.at<float>(1,2), estimated.at<float>(1,2), .008);
  EXPECT_NEAR(transform.at<float>(1,3), estimated.at<float>(1,3), .25);
  EXPECT_NEAR(transform.at<float>(2,0), estimated.at<float>(2,0), .008);
  EXPECT_NEAR(transform.at<float>(2,1), estimated.at<float>(2,1), .008);
  EXPECT_NEAR(transform.at<float>(2,2), estimated.at<float>(2,2), .008);
  EXPECT_NEAR(transform.at<float>(2,3), estimated.at<float>(2,3), .25);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
