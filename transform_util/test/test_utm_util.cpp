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

#include <transform_util/utm_util.h>

TEST(UtmUtilTests, GetZone)
{
  EXPECT_EQ(11, transform_util::GetZone(-118.408056));  // LAX
  EXPECT_EQ(17, transform_util::GetZone( -80.290556));  // MIA
  EXPECT_EQ(30, transform_util::GetZone(  -0.461389));  // LHR
  EXPECT_EQ(37, transform_util::GetZone(  37.414722));  // SVO
  EXPECT_EQ(54, transform_util::GetZone( 139.781111));  // HND
  EXPECT_EQ( 4, transform_util::GetZone(  -157.9225));  // HNL
}

TEST(UtmUtilTests, GetBand)
{
  EXPECT_EQ('S', transform_util::GetBand( 33.9425));    // LAX
  EXPECT_EQ('R', transform_util::GetBand( 25.793333));  // MIA
  EXPECT_EQ('U', transform_util::GetBand( 51.4775));    // LHR
  EXPECT_EQ('U', transform_util::GetBand( 55.972778));  // SVO
  EXPECT_EQ('S', transform_util::GetBand( 35.553333));  // HND
  EXPECT_EQ('Q', transform_util::GetBand( 21.318611));  // HNL
  EXPECT_EQ('F', transform_util::GetBand(-54.843333));  // USH

  EXPECT_EQ('Z', transform_util::GetBand(84.5));
  EXPECT_EQ('Z', transform_util::GetBand(-80.5));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
