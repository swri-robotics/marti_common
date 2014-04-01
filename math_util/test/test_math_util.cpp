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

#include <math_util/constants.h>
#include <math_util/math_util.h>

TEST(MathUtilTests, Round)
{
  EXPECT_EQ(0.0, math_util::Round(0.1));
  EXPECT_EQ(0.0, math_util::Round(-0.1));
  EXPECT_EQ(0.0, math_util::Round(0.45));
  EXPECT_EQ(90.0, math_util::Round(90.1));
  EXPECT_EQ(90.0, math_util::Round(89.7));
  EXPECT_EQ(-10.0, math_util::Round(-9.9));
  EXPECT_EQ(-10.0, math_util::Round(-10.1));
}

TEST(MathUtilTests, ToNearest)
{
  EXPECT_EQ(0.0, math_util::ToNearest(0.1, 1));
  EXPECT_EQ(0.0, math_util::ToNearest(-0.1, 1));
  EXPECT_EQ(0.0, math_util::ToNearest(0.45, 1));
  EXPECT_EQ(90.0, math_util::ToNearest(90.1, 1));
  EXPECT_EQ(90.0, math_util::ToNearest(89.7, 1));
  EXPECT_EQ(-10.0, math_util::ToNearest(-9.9, 1));
  EXPECT_EQ(-10.0, math_util::ToNearest(-10.1, 1));
  EXPECT_EQ(4, math_util::ToNearest(4, 2));
  EXPECT_EQ(-4, math_util::ToNearest(-4, 2));

  EXPECT_EQ(90.0, math_util::ToNearest(85.4, 90));
  EXPECT_EQ(270.0, math_util::ToNearest(301.4, 90));
  EXPECT_EQ(0.0, math_util::ToNearest(35.4, 90));
  EXPECT_EQ(0.0, math_util::ToNearest(-35.3, 90));
  EXPECT_EQ(-270.0, math_util::ToNearest(-301.4, 90));

  EXPECT_EQ(0.0, math_util::ToNearest(85.4, 0));
  EXPECT_EQ(0.0, math_util::ToNearest(301.4, 0));
  EXPECT_EQ(0.0, math_util::ToNearest(35.4, 0));
  EXPECT_EQ(0.0, math_util::ToNearest(-35.3, 0));
  EXPECT_EQ(0.0, math_util::ToNearest(-301.4, 0));

  EXPECT_EQ(0, math_util::ToNearest(-0.45, math_util::_half_pi));
  EXPECT_EQ(0, math_util::ToNearest(0.45, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_half_pi, math_util::ToNearest(1.2, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_half_pi, math_util::ToNearest(1.6, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::ToNearest(2.4, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::ToNearest(math_util::_pi, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_2pi, math_util::ToNearest(6.1, math_util::_half_pi));
}

TEST(MathUtilTests, UpToNearest)
{
  EXPECT_EQ(1.0, math_util::UpToNearest(0.1, 1));
  EXPECT_EQ(0.0, math_util::UpToNearest(-0.1, 1));
  EXPECT_EQ(1.0, math_util::UpToNearest(0.45, 1));
  EXPECT_EQ(91.0, math_util::UpToNearest(90.1, 1));
  EXPECT_EQ(90.0, math_util::UpToNearest(89.7, 1));
  EXPECT_EQ(-9.0, math_util::UpToNearest(-9.9, 1));
  EXPECT_EQ(-10.0, math_util::UpToNearest(-10.1, 1));
  EXPECT_EQ(4, math_util::UpToNearest(4, 2));
  EXPECT_EQ(-4, math_util::UpToNearest(-4, 2));

  EXPECT_EQ(90.0, math_util::UpToNearest(85.4, 90));
  EXPECT_EQ(360.0, math_util::UpToNearest(301.4, 90));
  EXPECT_EQ(90.0, math_util::UpToNearest(35.4, 90));
  EXPECT_EQ(0.0, math_util::UpToNearest(-35.3, 90));
  EXPECT_EQ(-270.0, math_util::UpToNearest(-301.4, 90));

  EXPECT_EQ(0.0, math_util::UpToNearest(85.4, 0));
  EXPECT_EQ(0.0, math_util::UpToNearest(301.4, 0));
  EXPECT_EQ(0.0, math_util::UpToNearest(35.4, 0));
  EXPECT_EQ(0.0, math_util::UpToNearest(-35.3, 0));
  EXPECT_EQ(0.0, math_util::UpToNearest(-301.4, 0));

  EXPECT_EQ(0, math_util::UpToNearest(-0.45, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_half_pi, math_util::UpToNearest(0.45, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_half_pi, math_util::UpToNearest(1.2, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::UpToNearest(1.6, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::UpToNearest(2.4, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::UpToNearest(math_util::_pi, math_util::_half_pi));
  EXPECT_FLOAT_EQ(math_util::_2pi, math_util::UpToNearest(6.1, math_util::_half_pi));
}

TEST(MathUtilTests, IsNear)
{
   EXPECT_TRUE(math_util::IsNear(1, 1, 0));
   EXPECT_TRUE(math_util::IsNear(3535.353, 3535.353, 0));
   EXPECT_TRUE(math_util::IsNear(10.1, 10, 0.1));
   EXPECT_TRUE(math_util::IsNear(10, 10.1, 0.1));
   EXPECT_TRUE(math_util::IsNear(-10.1, -10, 0.1));
   EXPECT_TRUE(math_util::IsNear(-10, -10.1, 0.1));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
