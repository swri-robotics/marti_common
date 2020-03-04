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

#include <cstdlib>

#include <array.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <swri_math_util/constants.h>
#include <swri_math_util/math_util.h>
#include <swri_transform_util/transform_util.h>

TEST(TransformUtilTests, GetRelativeTransform)
{
  tf::Transform offset = swri_transform_util::GetRelativeTransform(
                29.441679990508018, -98.602031700252184, -1.2030287,
                29.441609529848606, -98.601997698933161, -1.21015707397341
                );

  tf::Vector3 origin = offset.getOrigin();
  EXPECT_FLOAT_EQ(-8.47174665, origin.x());
  EXPECT_FLOAT_EQ(-0.3306987, origin.y());
  EXPECT_FLOAT_EQ(0.0, origin.z());
}

TEST(TransformUtilTests, GetBearing)
{
  EXPECT_FLOAT_EQ(0, swri_transform_util::GetBearing(30, 50, 35, 50));
  EXPECT_FLOAT_EQ(180, swri_transform_util::GetBearing(35, 50, 30, 50));
  EXPECT_FLOAT_EQ(90, swri_transform_util::GetBearing(0, 50, 0, 55));
  EXPECT_FLOAT_EQ(-90, swri_transform_util::GetBearing(0, 55, 0, 50));
}

TEST(TransformUtilTests, SnapToRightAngle1)
{
  tf::Quaternion identity = tf::Quaternion::getIdentity();

  EXPECT_TRUE(identity == swri_transform_util::SnapToRightAngle(identity));

  tf::Quaternion q1;
  q1.setRPY(0.0, 0.0, swri_math_util::_half_pi);
  q1.normalize();

  tf::Quaternion q2;
  q2.setRPY(0.4, 0.3, 1.6);

  tf::Quaternion q3 = swri_transform_util::SnapToRightAngle(q2);

  EXPECT_FLOAT_EQ(q1.x(), q3.x());
  EXPECT_FLOAT_EQ(q1.y(), q3.y());
  EXPECT_FLOAT_EQ(q1.z(), q3.z());
  EXPECT_FLOAT_EQ(q1.w(), q3.w());
}


TEST(TransformUtilTests, SnapToRightAngle2)
{
  tf::Quaternion q1;
  q1.setRPY(0.0, 0.0, swri_math_util::_half_pi);

  EXPECT_EQ(0, q1.angleShortestPath(swri_transform_util::SnapToRightAngle(q1)));

  tf::Quaternion q2;
  q2.setRPY(0.0, 0.0, swri_math_util::_pi);

  EXPECT_EQ(0, q2.angleShortestPath(swri_transform_util::SnapToRightAngle(q2)));

  tf::Quaternion q3;
  q3.setRPY(0.0, 0.0, swri_math_util::_pi + .34);
  EXPECT_EQ(0, q2.angleShortestPath(swri_transform_util::SnapToRightAngle(q3)));

  tf::Quaternion q4;
  q4.setRPY(-0.4, 0.23, swri_math_util::_pi + 0.34);
  EXPECT_EQ(0, q2.angleShortestPath(swri_transform_util::SnapToRightAngle(q4)));
}

TEST(TransformUtilTests, SnapToRightAngleRandom)
{
  std::srand(0);

  for (int32_t i = 0; i < 1000; i++)
  {
    double y = swri_math_util::Round((static_cast<double>(std::rand()) / RAND_MAX) * 4.0 - 2.0) * swri_math_util::_half_pi;
    double p = swri_math_util::Round((static_cast<double>(std::rand()) / RAND_MAX) * 4.0 - 2.0) * swri_math_util::_half_pi;
    double r = swri_math_util::Round((static_cast<double>(std::rand()) / RAND_MAX) * 2.0 - 1.0) * swri_math_util::_half_pi;

    tf::Quaternion q1;
    q1.setRPY(r, p, y);

    double dy = (static_cast<double>(std::rand()) / RAND_MAX) * swri_math_util::_half_pi * .5 - swri_math_util::_half_pi * .25;
    double dp = (static_cast<double>(std::rand()) / RAND_MAX) * swri_math_util::_half_pi * .5 - swri_math_util::_half_pi * .25;
    double dr = (static_cast<double>(std::rand()) / RAND_MAX) * swri_math_util::_half_pi * .5 - swri_math_util::_half_pi * .25;

    tf::Quaternion q2;
    q2.setRPY(r + dr, p + dp, y + dy);


    EXPECT_NEAR(0, q1.angleShortestPath(swri_transform_util::SnapToRightAngle(q1)), 0.00000003);
    EXPECT_NEAR(0, q1.angleShortestPath(swri_transform_util::SnapToRightAngle(q2)), 0.00000003);
  }
}

TEST(TransformUtilTests, SnapToRightAngleDegenerate1)
{
  double y = swri_math_util::_half_pi;
  double p = swri_math_util::_half_pi;
  double r = 0.0;

  double dy = -0.390704844938401241183356660258141346275806427001953125;
  double dp = 0.2254961365127778616379572440564516000449657440185546875;
  double dr = 0.38988573881896215755915591216762550175189971923828125;

  tf::Quaternion q1;
  q1.setRPY(r, p, y);

  tf::Quaternion q2;
  q2.setRPY(r + dr, p + dp, y + dy);

  EXPECT_NEAR(0, q1.angleShortestPath(swri_transform_util::SnapToRightAngle(q1)), 0.00000003);
  EXPECT_NEAR(0, q1.angleShortestPath(swri_transform_util::SnapToRightAngle(q2)), 0.00000003);
}

TEST(TransformUtilTests, SnapToRightAngleDegenerate2)
{
  double y = -swri_math_util::_pi;
  double p = -swri_math_util::_half_pi;
  double r = 0.0;

  double dy = -0.382504303669122636133437254102318547666072845458984375;
  double dp = 0.363762144414233323796992181087261997163295745849609375;
  double dr = -0.38101948131676321995797707131714560091495513916015625;

  tf::Quaternion q1;
  q1.setRPY(r, p, y);

  tf::Quaternion q2;
  q2.setRPY(r + dr, p + dp, y + dy);

  EXPECT_NEAR(0, q1.angleShortestPath(swri_transform_util::SnapToRightAngle(q1)), 0.00000003);
  EXPECT_NEAR(0, q1.angleShortestPath(swri_transform_util::SnapToRightAngle(q2)), 0.00000003);
}

TEST(TransformUtilTests, TestUpperLeftLowerRight)
{
  tf::Matrix3x3 ul(1, 2, 3, 4, 5, 6, 7, 8, 9);
  tf::Matrix3x3 lr(10, 11, 12, 13, 14, 15, 16, 17, 18);

  std::array<double, 36> array;

  swri_transform_util::SetUpperLeft(ul, array);
  swri_transform_util::SetLowerRight(lr, array);

  tf::Matrix3x3 ul2 = swri_transform_util::GetUpperLeft(array);
  tf::Matrix3x3 lr2 = swri_transform_util::GetLowerRight(array);

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

  EXPECT_EQ(v1, swri_transform_util::GetPrimaryAxis(v1));
  EXPECT_EQ(v1, swri_transform_util::GetPrimaryAxis(v2));

  EXPECT_EQ(v3, swri_transform_util::GetPrimaryAxis(v3));
  EXPECT_EQ(v3, swri_transform_util::GetPrimaryAxis(v4));

  EXPECT_EQ(v5, swri_transform_util::GetPrimaryAxis(v5));
  EXPECT_EQ(v5, swri_transform_util::GetPrimaryAxis(v6));

  EXPECT_EQ(v7, swri_transform_util::GetPrimaryAxis(v7));
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
    EXPECT_TRUE(swri_transform_util::IsRotation(valid_rotations[i]));
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
    EXPECT_FALSE(swri_transform_util::IsRotation(invalid_rotations[i]));
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
