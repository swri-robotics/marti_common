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
#include <math_util/trig_util.h>

TEST(TrigUtilTests, WrapRadians)
{
  // Test values wrapped between [-pi, pi]
  EXPECT_FLOAT_EQ(0, math_util::WrapRadians(0, 0));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::WrapRadians(math_util::_pi, 0));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::WrapRadians(math_util::_pi * 3.0, 0));
  EXPECT_FLOAT_EQ(math_util::_pi * -0.5, math_util::WrapRadians(math_util::_pi * 1.5, 0));
  EXPECT_FLOAT_EQ(math_util::_pi * 0.5, math_util::WrapRadians(math_util::_pi * -1.5, 0));

  // Test values wrapped between [0, 2pi]
  EXPECT_FLOAT_EQ(0, math_util::WrapRadians(0, math_util::_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::WrapRadians(math_util::_pi, math_util::_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::WrapRadians(math_util::_pi * 3.0, math_util::_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::WrapRadians(-math_util::_pi, math_util::_pi));
  EXPECT_FLOAT_EQ(math_util::_pi, math_util::WrapRadians(math_util::_pi * -3.0, math_util::_pi));
  EXPECT_FLOAT_EQ(math_util::_pi * 0.5, math_util::WrapRadians(math_util::_pi * 2.5, math_util::_pi));
  EXPECT_FLOAT_EQ(math_util::_pi * 1.5, math_util::WrapRadians(math_util::_pi * 3.5, math_util::_pi));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
