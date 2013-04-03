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
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
