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

#include <gtest/gtest.h>

#include <swri_math_util/constants.h>
#include <swri_math_util/math_util.h>

TEST(MathUtilTests, Round)
{
  EXPECT_EQ(0.0, swri_math_util::Round(0.1));
  EXPECT_EQ(0.0, swri_math_util::Round(-0.1));
  EXPECT_EQ(0.0, swri_math_util::Round(0.45));
  EXPECT_EQ(90.0, swri_math_util::Round(90.1));
  EXPECT_EQ(90.0, swri_math_util::Round(89.7));
  EXPECT_EQ(-10.0, swri_math_util::Round(-9.9));
  EXPECT_EQ(-10.0, swri_math_util::Round(-10.1));
}

TEST(MathUtilTests, ToNearest)
{
  EXPECT_EQ(0.0, swri_math_util::ToNearest(0.1, 1));
  EXPECT_EQ(0.0, swri_math_util::ToNearest(-0.1, 1));
  EXPECT_EQ(0.0, swri_math_util::ToNearest(0.45, 1));
  EXPECT_EQ(90.0, swri_math_util::ToNearest(90.1, 1));
  EXPECT_EQ(90.0, swri_math_util::ToNearest(89.7, 1));
  EXPECT_EQ(-10.0, swri_math_util::ToNearest(-9.9, 1));
  EXPECT_EQ(-10.0, swri_math_util::ToNearest(-10.1, 1));
  EXPECT_EQ(4, swri_math_util::ToNearest(4, 2));
  EXPECT_EQ(-4, swri_math_util::ToNearest(-4, 2));

  EXPECT_EQ(90.0, swri_math_util::ToNearest(85.4, 90));
  EXPECT_EQ(270.0, swri_math_util::ToNearest(301.4, 90));
  EXPECT_EQ(0.0, swri_math_util::ToNearest(35.4, 90));
  EXPECT_EQ(0.0, swri_math_util::ToNearest(-35.3, 90));
  EXPECT_EQ(-270.0, swri_math_util::ToNearest(-301.4, 90));

  EXPECT_EQ(0.0, swri_math_util::ToNearest(85.4, 0));
  EXPECT_EQ(0.0, swri_math_util::ToNearest(301.4, 0));
  EXPECT_EQ(0.0, swri_math_util::ToNearest(35.4, 0));
  EXPECT_EQ(0.0, swri_math_util::ToNearest(-35.3, 0));
  EXPECT_EQ(0.0, swri_math_util::ToNearest(-301.4, 0));

  EXPECT_EQ(0, swri_math_util::ToNearest(-0.45, swri_math_util::_half_pi));
  EXPECT_EQ(0, swri_math_util::ToNearest(0.45, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_half_pi, swri_math_util::ToNearest(1.2, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_half_pi, swri_math_util::ToNearest(1.6, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::ToNearest(2.4, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::ToNearest(swri_math_util::_pi, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_2pi, swri_math_util::ToNearest(6.1, swri_math_util::_half_pi));
}

TEST(MathUtilTests, UpToNearest)
{
  EXPECT_EQ(1.0, swri_math_util::UpToNearest(0.1, 1));
  EXPECT_EQ(0.0, swri_math_util::UpToNearest(-0.1, 1));
  EXPECT_EQ(1.0, swri_math_util::UpToNearest(0.45, 1));
  EXPECT_EQ(91.0, swri_math_util::UpToNearest(90.1, 1));
  EXPECT_EQ(90.0, swri_math_util::UpToNearest(89.7, 1));
  EXPECT_EQ(-9.0, swri_math_util::UpToNearest(-9.9, 1));
  EXPECT_EQ(-10.0, swri_math_util::UpToNearest(-10.1, 1));
  EXPECT_EQ(4, swri_math_util::UpToNearest(4, 2));
  EXPECT_EQ(-4, swri_math_util::UpToNearest(-4, 2));

  EXPECT_EQ(90.0, swri_math_util::UpToNearest(85.4, 90));
  EXPECT_EQ(360.0, swri_math_util::UpToNearest(301.4, 90));
  EXPECT_EQ(90.0, swri_math_util::UpToNearest(35.4, 90));
  EXPECT_EQ(0.0, swri_math_util::UpToNearest(-35.3, 90));
  EXPECT_EQ(-270.0, swri_math_util::UpToNearest(-301.4, 90));

  EXPECT_EQ(0.0, swri_math_util::UpToNearest(85.4, 0));
  EXPECT_EQ(0.0, swri_math_util::UpToNearest(301.4, 0));
  EXPECT_EQ(0.0, swri_math_util::UpToNearest(35.4, 0));
  EXPECT_EQ(0.0, swri_math_util::UpToNearest(-35.3, 0));
  EXPECT_EQ(0.0, swri_math_util::UpToNearest(-301.4, 0));

  EXPECT_EQ(0, swri_math_util::UpToNearest(-0.45, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_half_pi, swri_math_util::UpToNearest(0.45, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_half_pi, swri_math_util::UpToNearest(1.2, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::UpToNearest(1.6, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::UpToNearest(2.4, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::UpToNearest(swri_math_util::_pi, swri_math_util::_half_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_2pi, swri_math_util::UpToNearest(6.1, swri_math_util::_half_pi));
}

TEST(MathUtilTests, IsNear)
{
   EXPECT_TRUE(swri_math_util::IsNear(1, 1, 0));
   EXPECT_TRUE(swri_math_util::IsNear(3535.353, 3535.353, 0));
   EXPECT_TRUE(swri_math_util::IsNear(10.1, 10, 0.1));
   EXPECT_TRUE(swri_math_util::IsNear(10, 10.1, 0.1));
   EXPECT_TRUE(swri_math_util::IsNear(-10.1, -10, 0.1));
   EXPECT_TRUE(swri_math_util::IsNear(-10, -10.1, 0.1));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
