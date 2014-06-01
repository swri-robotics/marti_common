// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <gtest/gtest.h>

#include <string_util/string_util.h>

TEST(StringUtilTests, ToDouble_0)
{
  double value = -1;
  EXPECT_TRUE(string_util::ToDouble("0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble(".0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("+.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("+0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("+0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("+0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("+0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("+0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("-.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("-0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("-0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("-0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("-0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("-0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   .0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   +0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   +.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   +0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   +0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   +0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   +0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   +0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   -0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   -.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   -0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   -0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   -0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   -0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToDouble("   -0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
}

TEST(StringUtilTests, ToDouble_1)
{
  double value = 0;
  EXPECT_TRUE(string_util::ToDouble("1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("1.", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("1.0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("1.00", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("+1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("+1.", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("+1.0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("+1.00", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("+1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("+1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("+1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("-1", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("-1.", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("-1.0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("-1.00", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("-1.0e0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("-1.0e-0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("-1.0e+0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   1.", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   1.0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   1.00", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   +1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   +1.", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   +1.0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   +1.00", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   +1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   +1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   +1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   -1", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   -1.", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   -1.0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   -1.00", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   -1.0e0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   -1.0e-0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   -1.0e+0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   10.0e-1", value));
  EXPECT_FLOAT_EQ(1, value);
  
    value = 0;
  EXPECT_TRUE(string_util::ToDouble("   100.0e-2", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   0.1e1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   0.1e+1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("   0.01e+2", value));
  EXPECT_FLOAT_EQ(1, value);
}

TEST(StringUtilTests, ToDouble)
{
  double value = 0;
  EXPECT_TRUE(string_util::ToDouble("0.123456789", value));
  EXPECT_FLOAT_EQ(0.123456789, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("123456789.", value));
  EXPECT_FLOAT_EQ(123456789, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("12345678.9", value));
  EXPECT_FLOAT_EQ(12345678.9, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("123456789.00", value));
  EXPECT_FLOAT_EQ(123456789, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("12345.6789", value));
  EXPECT_FLOAT_EQ(12345.6789, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("1e308", value));
  EXPECT_FLOAT_EQ(1e308, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToDouble("1e-307", value));
  EXPECT_FLOAT_EQ(1e-307, value);
}

TEST(StringUtilTests, ToDoubleInvalid)
{
  double value = 0;
  
  EXPECT_FALSE(string_util::ToDouble("a", value));
  EXPECT_FALSE(string_util::ToDouble("a0", value));
  EXPECT_FALSE(string_util::ToDouble("0a", value));
  EXPECT_FALSE(string_util::ToDouble("0 ", value));
  EXPECT_FALSE(string_util::ToDouble("1e309", value));
  EXPECT_FALSE(string_util::ToDouble("1e-308", value));
}

TEST(StringUtilTests, ToFloat_0)
{
  float value = -1;
  EXPECT_TRUE(string_util::ToFloat("0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat(".0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("+.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("+0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("+0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("+0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("+0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("+0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("-.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("-0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("-0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("-0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("-0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("-0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   .0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   +0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   +.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   +0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   +0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   +0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   +0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   +0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   -0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   -.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   -0.0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   -0.00", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   -0.0e0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   -0.0e-0", value));
  EXPECT_FLOAT_EQ(0, value);
  
  value = -1;
  EXPECT_TRUE(string_util::ToFloat("   -0.0e+0", value));
  EXPECT_FLOAT_EQ(0, value);
}

TEST(StringUtilTests, ToFloat_1)
{
  float value = 0;
  EXPECT_TRUE(string_util::ToFloat("1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("1.", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("1.0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("1.00", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("+1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("+1.", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("+1.0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("+1.00", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("+1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("+1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("+1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("-1", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("-1.", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("-1.0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("-1.00", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("-1.0e0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("-1.0e-0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("-1.0e+0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   1.", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   1.0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   1.00", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   +1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   +1.", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   +1.0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   +1.00", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   +1.0e0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   +1.0e-0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   +1.0e+0", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   -1", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   -1.", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   -1.0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   -1.00", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   -1.0e0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   -1.0e-0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   -1.0e+0", value));
  EXPECT_FLOAT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   10.0e-1", value));
  EXPECT_FLOAT_EQ(1, value);
  
    value = 0;
  EXPECT_TRUE(string_util::ToFloat("   100.0e-2", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   0.1e1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   0.1e+1", value));
  EXPECT_FLOAT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("   0.01e+2", value));
  EXPECT_FLOAT_EQ(1, value);
}

TEST(StringUtilTests, ToFloat)
{
  float value = 0;
  EXPECT_TRUE(string_util::ToFloat("0.123456789", value));
  EXPECT_FLOAT_EQ(0.123456789, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("123456789.", value));
  EXPECT_FLOAT_EQ(123456789, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("12345678.9", value));
  EXPECT_FLOAT_EQ(12345678.9, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("123456789.00", value));
  EXPECT_FLOAT_EQ(123456789, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("12345.6789", value));
  EXPECT_FLOAT_EQ(12345.6789, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("1e38", value));
  EXPECT_FLOAT_EQ(1e38, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToFloat("1e-37", value));
  EXPECT_FLOAT_EQ(1e-37, value);
}

TEST(StringUtilTests, ToFloatInvalid)
{
  float value = 0;
  
  EXPECT_FALSE(string_util::ToFloat("", value));
  EXPECT_FALSE(string_util::ToFloat(" ", value));
  EXPECT_FALSE(string_util::ToFloat("a", value));
  EXPECT_FALSE(string_util::ToFloat("a0", value));
  EXPECT_FALSE(string_util::ToFloat("0a", value));
  EXPECT_FALSE(string_util::ToFloat("0 ", value));
  EXPECT_FALSE(string_util::ToFloat("1e39", value));
  EXPECT_FALSE(string_util::ToFloat("1e-38", value));
}

TEST(StringUtilTests, ToInt32Base10)
{
  int32_t value = 1;
  EXPECT_TRUE(string_util::ToInt32("0", value));
  EXPECT_EQ(0, value);
  
  value = 1;
  EXPECT_TRUE(string_util::ToInt32("-0", value));
  EXPECT_EQ(0, value);
  
  value = 1;
  EXPECT_TRUE(string_util::ToInt32("+0", value));
  EXPECT_EQ(0, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToInt32("1", value));
  EXPECT_EQ(1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToInt32("+1", value));
  EXPECT_EQ(1, value);
 
  value = 0;
  EXPECT_TRUE(string_util::ToInt32("-1", value));
  EXPECT_EQ(-1, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToInt32("12345678", value));
  EXPECT_EQ(12345678, value);
  
  value = 0;
  EXPECT_TRUE(string_util::ToInt32("-123456789", value));
  EXPECT_EQ(-123456789, value);
}

TEST(StringUtilTests, ToInt32Base10Invalid)
{
  int32_t value = 0;
  
  EXPECT_FALSE(string_util::ToInt32("", value));
  EXPECT_FALSE(string_util::ToInt32(" ", value));
  EXPECT_FALSE(string_util::ToInt32("a", value));
  EXPECT_FALSE(string_util::ToInt32("a0", value));
  EXPECT_FALSE(string_util::ToInt32("0a", value));
  EXPECT_FALSE(string_util::ToInt32("0 ", value));
  EXPECT_FALSE(string_util::ToInt32("1e39", value));
  EXPECT_FALSE(string_util::ToInt32("1e-38", value));
  EXPECT_FALSE(string_util::ToInt32("1e1", value));
  EXPECT_FALSE(string_util::ToInt32("1e0", value));
  EXPECT_FALSE(string_util::ToInt32(".", value));
  EXPECT_FALSE(string_util::ToInt32("0.0", value));
  EXPECT_FALSE(string_util::ToInt32("0.", value));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
