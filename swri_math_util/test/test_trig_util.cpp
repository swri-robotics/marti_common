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
#include <swri_math_util/trig_util.h>

TEST(TrigUtilTests, WrapRadians)
{
  // Test values wrapped between [-pi, pi]
  EXPECT_FLOAT_EQ(0, swri_math_util::WrapRadians(0, 0));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::WrapRadians(swri_math_util::_pi, 0));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::WrapRadians(swri_math_util::_pi * 3.0, 0));
  EXPECT_FLOAT_EQ(swri_math_util::_pi * -0.5, swri_math_util::WrapRadians(swri_math_util::_pi * 1.5, 0));
  EXPECT_FLOAT_EQ(swri_math_util::_pi * 0.5, swri_math_util::WrapRadians(swri_math_util::_pi * -1.5, 0));

  // Test values wrapped between [0, 2pi]
  EXPECT_FLOAT_EQ(0, swri_math_util::WrapRadians(0, swri_math_util::_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::WrapRadians(swri_math_util::_pi, swri_math_util::_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::WrapRadians(swri_math_util::_pi * 3.0, swri_math_util::_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::WrapRadians(-swri_math_util::_pi, swri_math_util::_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi, swri_math_util::WrapRadians(swri_math_util::_pi * -3.0, swri_math_util::_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi * 0.5, swri_math_util::WrapRadians(swri_math_util::_pi * 2.5, swri_math_util::_pi));
  EXPECT_FLOAT_EQ(swri_math_util::_pi * 1.5, swri_math_util::WrapRadians(swri_math_util::_pi * 3.5, swri_math_util::_pi));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
