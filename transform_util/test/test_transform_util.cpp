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

#include <cstdlib>

#include <boost/array.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <math_util/constants.h>
#include <math_util/math_util.h>
#include <transform_util/transform_util.h>

// TODO(malban): Add unit tests for GetRelativeTransform()

TEST(TransformUtilTests, GetBearing)
{
  EXPECT_FLOAT_EQ(0, transform_util::GetBearing(30, 50, 35, 50));
  EXPECT_FLOAT_EQ(180, transform_util::GetBearing(35, 50, 30, 50));
  EXPECT_FLOAT_EQ(90, transform_util::GetBearing(0, 50, 0, 55));
  EXPECT_FLOAT_EQ(-90, transform_util::GetBearing(0, 55, 0, 50));
}

TEST(TransformUtilTests, SnapToRightAngle1)
{
  tf::Quaternion identity = tf::Quaternion::getIdentity();

  EXPECT_TRUE(identity == transform_util::SnapToRightAngle(identity));

  tf::Quaternion q1;
  q1.setRPY(0.0, 0.0, math_util::_half_pi);
  q1.normalize();

  tf::Quaternion q2;
  q2.setRPY(0.4, 0.3, 1.6);

  tf::Quaternion q3 = transform_util::SnapToRightAngle(q2);

  EXPECT_FLOAT_EQ(q1.x(), q3.x());
  EXPECT_FLOAT_EQ(q1.y(), q3.y());
  EXPECT_FLOAT_EQ(q1.z(), q3.z());
  EXPECT_FLOAT_EQ(q1.w(), q3.w());
}


TEST(TransformUtilTests, SnapToRightAngle2)
{
  tf::Quaternion q1;
  q1.setRPY(0.0, 0.0, math_util::_half_pi);

  EXPECT_EQ(0, q1.angleShortestPath(transform_util::SnapToRightAngle(q1)));

  tf::Quaternion q2;
  q2.setRPY(0.0, 0.0, math_util::_pi);

  EXPECT_EQ(0, q2.angleShortestPath(transform_util::SnapToRightAngle(q2)));

  tf::Quaternion q3;
  q3.setRPY(0.0, 0.0, math_util::_pi + .34);
  EXPECT_EQ(0, q2.angleShortestPath(transform_util::SnapToRightAngle(q3)));

  tf::Quaternion q4;
  q4.setRPY(-0.4, 0.23, math_util::_pi + 0.34);
  EXPECT_EQ(0, q2.angleShortestPath(transform_util::SnapToRightAngle(q4)));
}

TEST(TransformUtilTests, SnapToRightAngleRandom)
{
  std::srand(0);

  for (int32_t i = 0; i < 1000; i++)
  {
    double y = math_util::Round(((double)std::rand() / RAND_MAX) * 4.0 - 2.0) * math_util::_half_pi;
    double p = math_util::Round(((double)std::rand() / RAND_MAX) * 4.0 - 2.0) * math_util::_half_pi;
    double r = math_util::Round(((double)std::rand() / RAND_MAX) * 2.0 - 1.0) * math_util::_half_pi;

    tf::Quaternion q1;
    q1.setRPY(r, p, y);

    double dy = ((double)std::rand() / RAND_MAX) * math_util::_half_pi * .5 - math_util::_half_pi * .25;
    double dp = ((double)std::rand() / RAND_MAX) * math_util::_half_pi * .5 - math_util::_half_pi * .25;
    double dr = ((double)std::rand() / RAND_MAX) * math_util::_half_pi * .5 - math_util::_half_pi * .25;

    tf::Quaternion q2;
    q2.setRPY(r + dr, p + dp, y + dy);


    EXPECT_NEAR(0, q1.angleShortestPath(transform_util::SnapToRightAngle(q1)), 0.00000003);
    EXPECT_NEAR(0, q1.angleShortestPath(transform_util::SnapToRightAngle(q2)), 0.00000003);
  }
}

TEST(TransformUtilTests, SnapToRightAngleDegenerate1)
{
  double y = math_util::_half_pi;
  double p = math_util::_half_pi;
  double r = 0.0;

  double dy = -0.390704844938401241183356660258141346275806427001953125;
  double dp = 0.2254961365127778616379572440564516000449657440185546875;
  double dr = 0.38988573881896215755915591216762550175189971923828125;

  tf::Quaternion q1;
  q1.setRPY(r, p, y);

  tf::Quaternion q2;
  q2.setRPY(r + dr, p + dp, y + dy);

  EXPECT_NEAR(0, q1.angleShortestPath(transform_util::SnapToRightAngle(q1)), 0.00000003);
  EXPECT_NEAR(0, q1.angleShortestPath(transform_util::SnapToRightAngle(q2)), 0.00000003);
}

TEST(TransformUtilTests, SnapToRightAngleDegenerate2)
{
  double y = -math_util::_pi;
  double p = -math_util::_half_pi;
  double r = 0.0;

  double dy = -0.382504303669122636133437254102318547666072845458984375;
  double dp = 0.363762144414233323796992181087261997163295745849609375;
  double dr = -0.38101948131676321995797707131714560091495513916015625;

  tf::Quaternion q1;
  q1.setRPY(r, p, y);

  tf::Quaternion q2;
  q2.setRPY(r + dr, p + dp, y + dy);

  EXPECT_NEAR(0, q1.angleShortestPath(transform_util::SnapToRightAngle(q1)), 0.00000003);
  EXPECT_NEAR(0, q1.angleShortestPath(transform_util::SnapToRightAngle(q2)), 0.00000003);
}

TEST(TransformUtilTests, TestUpperLeftLowerRight)
{
  tf::Matrix3x3 ul(1, 2, 3, 4, 5, 6, 7, 8, 9);
  tf::Matrix3x3 lr(10, 11, 12, 13, 14, 15, 16, 17, 18);

  boost::array<double, 36> array;

  transform_util::SetUpperLeft(ul, array);
  transform_util::SetLowerRight(lr, array);

  tf::Matrix3x3 ul2 = transform_util::GetUpperLeft(array);
  tf::Matrix3x3 lr2 = transform_util::GetLowerRight(array);

  EXPECT_EQ(ul, ul2);
  EXPECT_EQ(lr, lr2);

  EXPECT_EQ(1, array[0]);
  EXPECT_EQ(18, array[35]);
}

TEST(TransformUtilTests, GetPrimaryAxis)
{
  tf::Vector3 v1(-1, 0, 0);
  tf::Vector3 v2(-.7, .3, 0);

  tf::Vector3 v3(0, 1, 0);
  tf::Vector3 v4(.6, .61, .3);

  tf::Vector3 v5(0, 0, 1);
  tf::Vector3 v6(-.23, .3, .5);

  tf::Vector3 v7(0, 0, 0);

  EXPECT_EQ(v1, transform_util::GetPrimaryAxis(v1));
  EXPECT_EQ(v1, transform_util::GetPrimaryAxis(v2));

  EXPECT_EQ(v3, transform_util::GetPrimaryAxis(v3));
  EXPECT_EQ(v3, transform_util::GetPrimaryAxis(v4));

  EXPECT_EQ(v5, transform_util::GetPrimaryAxis(v5));
  EXPECT_EQ(v5, transform_util::GetPrimaryAxis(v6));

  EXPECT_EQ(v7, transform_util::GetPrimaryAxis(v7));
}

TEST(TransformUtilTests, ValidIsRotation)
{
  tf::Matrix3x3 valid_rotations[] = {
  tf::Matrix3x3( 1,  0,  0,   0,  1,  0,   0,  0,  1),
  tf::Matrix3x3( 0,  0,  1,   0,  1,  0,  -1,  0,  0),
  tf::Matrix3x3(-1,  0,  0,   0,  1,  0,   0,  0, -1),
  tf::Matrix3x3( 0,  0, -1,   0,  1,  0,   1,  0,  0),
  tf::Matrix3x3( 0, -1,  0,   1,  0,  0,   0,  0,  1),
  tf::Matrix3x3( 0,  0,  1,   1,  0,  0,   0,  1,  0),
  tf::Matrix3x3( 0,  1,  0,   1,  0,  0,   0,  0, -1),
  tf::Matrix3x3( 0,  0, -1,   1,  0,  0,   0, -1,  0),
  tf::Matrix3x3( 0,  1,  0,  -1,  0,  0,   0,  0,  1),
  tf::Matrix3x3( 0,  0,  1,  -1,  0,  0,   0, -1,  0),
  tf::Matrix3x3( 0, -1,  0,  -1,  0,  0,   0,  0, -1),
  tf::Matrix3x3( 0,  0, -1,  -1,  0,  0,   0,  1,  0),
  tf::Matrix3x3( 1,  0,  0,   0,  0, -1,   0,  1,  0),
  tf::Matrix3x3( 0,  1,  0,   0,  0, -1,  -1,  0,  0),
  tf::Matrix3x3(-1,  0,  0,   0,  0, -1,   0, -1,  0),
  tf::Matrix3x3( 0, -1,  0,   0,  0, -1,   1,  0,  0),
  tf::Matrix3x3( 1,  0,  0,   0, -1,  0,   0,  0, -1),
  tf::Matrix3x3( 0,  0, -1,   0, -1,  0,  -1,  0,  0),
  tf::Matrix3x3(-1,  0,  0,   0, -1,  0,   0,  0,  1),
  tf::Matrix3x3( 0,  0,  1,   0, -1,  0,   1,  0,  0),
  tf::Matrix3x3( 1,  0,  0,   0,  0,  1,   0, -1,  0),
  tf::Matrix3x3( 0, -1,  0,   0,  0,  1,  -1,  0,  0),
  tf::Matrix3x3(-1,  0,  0,   0,  0,  1,   0,  1,  0),
  tf::Matrix3x3( 0,  1,  0,   0,  0,  1,   1,  0,  0)};

  for (int i = 0; i < 24; i++)
  {
    EXPECT_TRUE(transform_util::IsRotation(valid_rotations[i]));
  }
}

TEST(TransformUtilTests, InvalidIsRotation)
{
  tf::Matrix3x3 invalid_rotations[] = {
  tf::Matrix3x3( 2,  0,  0,   0,  1,  0,   0,  0,  1),
  tf::Matrix3x3( 0,  0,  1,   0,  0,  0,  -1,  0,  0),
  tf::Matrix3x3(-1,  1,  0,   0,  1,  0,   0,  0, -1)};

  for (int i = 0; i < 24; i++)
  {
    EXPECT_FALSE(transform_util::IsRotation(invalid_rotations[i]));
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Initialize the ROS core parameters can be loaded from the launch file
  ros::init(argc, argv, "test_transform_util");

  return RUN_ALL_TESTS();
}
