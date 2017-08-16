// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#include <swri_geometry_util/intersection.h>

TEST(IntersectionTests, Intersects)
{
  cv::Vec2d c;
  ASSERT_TRUE(swri_geometry_util::LineIntersection(
      cv::Vec2d(0, 5),
      cv::Vec2d(10, 5),
      cv::Vec2d(5, 0),
      cv::Vec2d(5, 10),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(5.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineIntersection(
      cv::Vec2d(0, 6),
      cv::Vec2d(10, 6),
      cv::Vec2d(5, 0),
      cv::Vec2d(5, 10),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(6.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineIntersection(
      cv::Vec2d(10, 6),
      cv::Vec2d(0, 6),
      cv::Vec2d(5, 10),
      cv::Vec2d(5, 0),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(6.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineIntersection(
      cv::Vec2d(5, 10),
      cv::Vec2d(5, 0),
      cv::Vec2d(10, 6),
      cv::Vec2d(0, 6),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(6.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineIntersection(
      cv::Vec2d(10, 10),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 10),
      cv::Vec2d(10, 0),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(5.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineIntersection(
      cv::Vec2d(-10, -10),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, -10),
      cv::Vec2d(-10, 0),
      c));

  EXPECT_FLOAT_EQ(-5.0, c[0]);
  EXPECT_FLOAT_EQ(-5.0, c[1]);
}

TEST(IntersectionTests, Parallel)
{
  cv::Vec2d c;
  EXPECT_FALSE(swri_geometry_util::LineIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 0),
      c));

  EXPECT_FALSE(swri_geometry_util::LineIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      c));

  EXPECT_FALSE(swri_geometry_util::LineIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(25, 0),
      cv::Vec2d(15, 0),
      c));

  EXPECT_FALSE(swri_geometry_util::LineIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(0, -1),
      cv::Vec2d(10, -1),
      c));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
