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

#include <swri_string_util/string_util.h>

TEST(StringUtilTests, ToDouble_0)
{
  double value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble(".0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("+.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("+0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("+0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("+0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("+0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("+0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("-.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("-0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("-0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("-0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("-0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("-0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   .0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   +0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   +.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   +0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   +0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   +0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   +0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   +0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   -0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   -.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   -0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   -0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   -0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   -0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToDouble("   -0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
}

TEST(StringUtilTests, ToDouble_1)
{
  double value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1.", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1.0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1.00", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("+1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("+1.", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("+1.0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("+1.00", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("+1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("+1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("+1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("-1", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("-1.", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("-1.0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("-1.00", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("-1.0e0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("-1.0e-0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("-1.0e+0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   1.", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   1.0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   1.00", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   +1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   +1.", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   +1.0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   +1.00", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   +1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   +1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   +1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   -1", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   -1.", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   -1.0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   -1.00", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   -1.0e0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   -1.0e-0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   -1.0e+0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   10.0e-1", value));
  EXPECT_FLOAT_EQ(1, value);

    value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   100.0e-2", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   0.1e1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   0.1e+1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("   0.01e+2", value));
  EXPECT_FLOAT_EQ(1, value);
}

TEST(StringUtilTests, ToDouble)
{
  double value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("0.123456789", value));
  EXPECT_FLOAT_EQ(0.123456789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("123456789.", value));
  EXPECT_FLOAT_EQ(123456789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("12345678.9", value));
  EXPECT_FLOAT_EQ(12345678.9, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("123456789.00", value));
  EXPECT_FLOAT_EQ(123456789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("12345.6789", value));
  EXPECT_FLOAT_EQ(12345.6789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1e308", value));
  EXPECT_FLOAT_EQ(1e308, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToDouble("1e-307", value));
  EXPECT_FLOAT_EQ(1e-307, value);
}

TEST(StringUtilTests, ToDoubleInvalid)
{
  double value = 0;

  EXPECT_FALSE(swri_string_util::ToDouble("a", value));
  EXPECT_FALSE(swri_string_util::ToDouble("a0", value));
  EXPECT_FALSE(swri_string_util::ToDouble("0a", value));
  EXPECT_FALSE(swri_string_util::ToDouble("0 ", value));
  EXPECT_FALSE(swri_string_util::ToDouble("1e309", value));
  EXPECT_FALSE(swri_string_util::ToDouble("1e-308", value));
}

TEST(StringUtilTests, ToFloat_0)
{
  float value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat(".0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("+.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("+0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("+0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("+0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("+0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("+0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("-.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("-0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("-0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("-0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("-0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("-0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   .0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   +0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   +.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   +0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   +0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   +0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   +0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   +0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   -0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   -.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   -0.0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   -0.00", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   -0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   -0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);

  value = -1;
  EXPECT_TRUE(swri_string_util::ToFloat("   -0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
}

TEST(StringUtilTests, ToFloat_1)
{
  float value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1.", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1.0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1.00", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("+1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("+1.", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("+1.0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("+1.00", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("+1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("+1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("+1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("-1", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("-1.", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("-1.0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("-1.00", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("-1.0e0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("-1.0e-0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("-1.0e+0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   1.", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   1.0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   1.00", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   +1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   +1.", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   +1.0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   +1.00", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   +1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   +1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   +1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   -1", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   -1.", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   -1.0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   -1.00", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   -1.0e0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   -1.0e-0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   -1.0e+0", value));
  EXPECT_FLOAT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   10.0e-1", value));
  EXPECT_FLOAT_EQ(1, value);

    value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   100.0e-2", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   0.1e1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   0.1e+1", value));
  EXPECT_FLOAT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("   0.01e+2", value));
  EXPECT_FLOAT_EQ(1, value);
}

TEST(StringUtilTests, ToFloat)
{
  float value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("0.123456789", value));
  EXPECT_FLOAT_EQ(0.123456789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("123456789.", value));
  EXPECT_FLOAT_EQ(123456789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("12345678.9", value));
  EXPECT_FLOAT_EQ(12345678.9, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("123456789.00", value));
  EXPECT_FLOAT_EQ(123456789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("12345.6789", value));
  EXPECT_FLOAT_EQ(12345.6789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1e38", value));
  EXPECT_FLOAT_EQ(1e38, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToFloat("1e-37", value));
  EXPECT_FLOAT_EQ(1e-37, value);
}

TEST(StringUtilTests, ToFloatInvalid)
{
  float value = 0;

  EXPECT_FALSE(swri_string_util::ToFloat("", value));
  EXPECT_FALSE(swri_string_util::ToFloat(" ", value));
  EXPECT_FALSE(swri_string_util::ToFloat("a", value));
  EXPECT_FALSE(swri_string_util::ToFloat("a0", value));
  EXPECT_FALSE(swri_string_util::ToFloat("0a", value));
  EXPECT_FALSE(swri_string_util::ToFloat("0 ", value));
  EXPECT_FALSE(swri_string_util::ToFloat("1e39", value));
  EXPECT_FALSE(swri_string_util::ToFloat("1e-38", value));
}

TEST(StringUtilTests, ToInt32Base10)
{
  int32_t value = 1;
  EXPECT_TRUE(swri_string_util::ToInt32("0", value));
  EXPECT_EQ(0, value);

  EXPECT_TRUE(swri_string_util::ToInt32("00", value));
  EXPECT_EQ(0, value);

  value = 1;
  EXPECT_TRUE(swri_string_util::ToInt32("-0", value));
  EXPECT_EQ(0, value);

  value = 1;
  EXPECT_TRUE(swri_string_util::ToInt32("+0", value));
  EXPECT_EQ(0, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("1", value));
  EXPECT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("+1", value));
  EXPECT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("-1", value));
  EXPECT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("12345678", value));
  EXPECT_EQ(12345678, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("012345678", value));
  EXPECT_EQ(12345678, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("-123456789", value));
  EXPECT_EQ(-123456789, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("2147483647", value));
  EXPECT_EQ(2147483647, value);
}

TEST(StringUtilTests, ToInt32Base10Invalid)
{
  int32_t value = 0;

  EXPECT_FALSE(swri_string_util::ToInt32("", value));
  EXPECT_FALSE(swri_string_util::ToInt32(" ", value));
  EXPECT_FALSE(swri_string_util::ToInt32("a", value));
  EXPECT_FALSE(swri_string_util::ToInt32("a0", value));
  EXPECT_FALSE(swri_string_util::ToInt32("0a", value));
  EXPECT_FALSE(swri_string_util::ToInt32("0 ", value));
  EXPECT_FALSE(swri_string_util::ToInt32("1e39", value));
  EXPECT_FALSE(swri_string_util::ToInt32("1e-38", value));
  EXPECT_FALSE(swri_string_util::ToInt32("1e1", value));
  EXPECT_FALSE(swri_string_util::ToInt32("1e0", value));
  EXPECT_FALSE(swri_string_util::ToInt32(".", value));
  EXPECT_FALSE(swri_string_util::ToInt32("0.0", value));
  EXPECT_FALSE(swri_string_util::ToInt32("0.", value));
  EXPECT_FALSE(swri_string_util::ToInt32("2147483648", value));
}

TEST(StringUtilTests, ToInt32Base16)
{
  int32_t value = 1;
  EXPECT_TRUE(swri_string_util::ToInt32("0", value, 16));
  EXPECT_EQ(0, value);

  EXPECT_TRUE(swri_string_util::ToInt32("00", value, 16));
  EXPECT_EQ(0, value);

  value = 1;
  EXPECT_TRUE(swri_string_util::ToInt32("-0", value, 16));
  EXPECT_EQ(0, value);

  value = 1;
  EXPECT_TRUE(swri_string_util::ToInt32("+0", value, 16));
  EXPECT_EQ(0, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("1", value, 16));
  EXPECT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("+1", value, 16));
  EXPECT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("-1", value, 16));
  EXPECT_EQ(-1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("12345678", value, 16));
  EXPECT_EQ(0x12345678, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("012345678", value, 16));
  EXPECT_EQ(0x12345678, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("-12345678", value, 16));
  EXPECT_EQ(-0x12345678, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToInt32("aff", value, 16));
  EXPECT_EQ(0xaff, value);
}

TEST(StringUtilTests, ToInt32Base16Invalid)
{
  int32_t value = 0;

  EXPECT_FALSE(swri_string_util::ToInt32("", value, 16));
  EXPECT_FALSE(swri_string_util::ToInt32(" ", value, 16));
  EXPECT_FALSE(swri_string_util::ToInt32("g", value, 16));
  EXPECT_FALSE(swri_string_util::ToInt32("g0", value, 16));
  EXPECT_FALSE(swri_string_util::ToInt32("0g", value, 16));
  EXPECT_FALSE(swri_string_util::ToInt32("0 ", value, 16));
  EXPECT_FALSE(swri_string_util::ToInt32(".", value, 16));
  EXPECT_FALSE(swri_string_util::ToInt32("0.0", value, 16));
  EXPECT_FALSE(swri_string_util::ToInt32("0.", value, 16));
}

TEST(StringUtilTests, ToUInt32Base10)
{
  uint32_t value = 1;
  EXPECT_TRUE(swri_string_util::ToUInt32("0", value));
  EXPECT_EQ(0, value);

  EXPECT_TRUE(swri_string_util::ToUInt32("00", value));
  EXPECT_EQ(0, value);

  value = 1;
  EXPECT_TRUE(swri_string_util::ToUInt32("-0", value));
  EXPECT_EQ(0, value);

  value = 1;
  EXPECT_TRUE(swri_string_util::ToUInt32("+0", value));
  EXPECT_EQ(0, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToUInt32("1", value));
  EXPECT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToUInt32("+1", value));
  EXPECT_EQ(1, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToUInt32("12345678", value));
  EXPECT_EQ(12345678, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToUInt32("012345678", value));
  EXPECT_EQ(12345678, value);

  value = 0;
  EXPECT_TRUE(swri_string_util::ToUInt32("4294967295", value));
  EXPECT_EQ(4294967295, value);
}

TEST(StringUtilTests, ToUInt32Base10Invalid)
{
  uint32_t value = 0;

  EXPECT_FALSE(swri_string_util::ToUInt32("", value));
  EXPECT_FALSE(swri_string_util::ToUInt32(" ", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("a", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("a0", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("0a", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("0 ", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("1e39", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("1e-38", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("1e1", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("1e0", value));
  EXPECT_FALSE(swri_string_util::ToUInt32(".", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("0.0", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("0.", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("4294967296", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("-1", value));
  EXPECT_FALSE(swri_string_util::ToUInt32("-123456789", value));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
