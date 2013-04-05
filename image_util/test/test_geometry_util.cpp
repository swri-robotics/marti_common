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

#include <image_util/geometry_util.h>

TEST(ImageUtilTests, Intersects)
{
  image_util::BoundingBox b1(-2, -2, 4, 4);
  image_util::BoundingBox b2(-1, -1, 2, 2);
  image_util::BoundingBox b3(-4, -4, 2.5, 2.5);
  image_util::BoundingBox b4(1.5, 1.5, 6, 6);
  image_util::BoundingBox b5(4, 4, 6, 6);

  EXPECT_TRUE(image_util::Intersects(b1, b2));
  EXPECT_TRUE(image_util::Intersects(b1, b3));
  EXPECT_TRUE(image_util::Intersects(b1, b4));
  EXPECT_FALSE(image_util::Intersects(b1, b5));

  EXPECT_TRUE(image_util::Intersects(b2, b1));
  EXPECT_FALSE(image_util::Intersects(b2, b3));
  EXPECT_FALSE(image_util::Intersects(b2, b4));
  EXPECT_FALSE(image_util::Intersects(b2, b5));

  EXPECT_TRUE(image_util::Intersects(b3, b1));
  EXPECT_FALSE(image_util::Intersects(b3, b2));
  EXPECT_FALSE(image_util::Intersects(b3, b4));
  EXPECT_FALSE(image_util::Intersects(b3, b5));

  EXPECT_TRUE(image_util::Intersects(b4, b1));
  EXPECT_FALSE(image_util::Intersects(b4, b2));
  EXPECT_FALSE(image_util::Intersects(b4, b3));
  EXPECT_TRUE(image_util::Intersects(b4, b5));

  EXPECT_FALSE(image_util::Intersects(b5, b1));
  EXPECT_FALSE(image_util::Intersects(b5, b2));
  EXPECT_FALSE(image_util::Intersects(b5, b3));
  EXPECT_TRUE(image_util::Intersects(b5, b4));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
