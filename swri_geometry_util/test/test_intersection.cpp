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

  ASSERT_TRUE(swri_geometry_util::LineIntersection(
      cv::Vec2d(0, 5),
      cv::Vec2d(-10, 5),
      cv::Vec2d(5, 0),
      cv::Vec2d(5, -10),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(5.0, c[1]);
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

TEST(IntersectionTests, SegmentsIntersect)
{
  cv::Vec2d c;
  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 5),
      cv::Vec2d(10, 5),
      cv::Vec2d(5, 0),
      cv::Vec2d(5, 10),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(5.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 6),
      cv::Vec2d(10, 6),
      cv::Vec2d(5, 0),
      cv::Vec2d(5, 10),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(6.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 6),
      cv::Vec2d(0, 6),
      cv::Vec2d(5, 10),
      cv::Vec2d(5, 0),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(6.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(5, 10),
      cv::Vec2d(5, 0),
      cv::Vec2d(10, 6),
      cv::Vec2d(0, 6),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(6.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 10),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 10),
      cv::Vec2d(10, 0),
      c));

  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(5.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(-10, -10),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, -10),
      cv::Vec2d(-10, 0),
      c));

  EXPECT_FLOAT_EQ(-5.0, c[0]);
  EXPECT_FLOAT_EQ(-5.0, c[1]);


  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(2, 2),
      c));

  EXPECT_FLOAT_EQ(1.0, c[0]);
  EXPECT_FLOAT_EQ(1.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(2, 0),
      c));

  EXPECT_FLOAT_EQ(2.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(3, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      c));
  EXPECT_FLOAT_EQ(2.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(3, 0),
      c));
  EXPECT_FLOAT_EQ(2.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(3, 0),
      cv::Vec2d(0, 0),
      c));
  EXPECT_FLOAT_EQ(2.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(2, 0),
      cv::Vec2d(0, 0),
      c));
  EXPECT_FLOAT_EQ(2.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(0, 2),
      cv::Vec2d(0, 0),
      c));

  EXPECT_FLOAT_EQ(0.0, c[0]);
  EXPECT_FLOAT_EQ(2.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(1, 1),
      cv::Vec2d(0, 0),
      c));

  EXPECT_FLOAT_EQ(1.0, c[0]);
  EXPECT_FLOAT_EQ(1.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(200, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(33.3, 0),
      cv::Vec2d(10, 10),
      c));
  EXPECT_FLOAT_EQ(33.3, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 0),
      c));
  EXPECT_FLOAT_EQ(0.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(1, 1),
      cv::Vec2d(1, 1),
      c));
  EXPECT_FLOAT_EQ(1.0, c[0]);
  EXPECT_FLOAT_EQ(1.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(1, 1),
      cv::Vec2d(1, 1),
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      c));
  EXPECT_FLOAT_EQ(1.0, c[0]);
  EXPECT_FLOAT_EQ(1.0, c[1]);

  ASSERT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(1065.8687582537791058712173253,-1053.2999883795632740657310933),
      cv::Vec2d(1065.5686875431010776082985103,-1051.8000590902413478033849970),
      cv::Vec2d(1066.1686875431009866588283330,-1053.0000590902413932781200856),
      cv::Vec2d(1065.5686875431010776082985103,-1051.8000590902413478033849970),
      c));
  EXPECT_FLOAT_EQ(1065.5686875431010776082985103, c[0]);
  EXPECT_FLOAT_EQ(-1051.8000590902413478033849970, c[1]);
}

TEST(IntersectionTests, SegmentsDontIntersect)
{
  cv::Vec2d c;
  ASSERT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 5),
      cv::Vec2d(-10, 5),
      cv::Vec2d(5, 0),
      cv::Vec2d(5, -10),
      c));

  ASSERT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(2, 1),
      cv::Vec2d(5, 2),
      c));

  ASSERT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(-5, -5),
      c));

  ASSERT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(0.999, 0.999),
      cv::Vec2d(0, 0),
      c));

  ASSERT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(200, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(33.3, 0.0000001),
      cv::Vec2d(10, 10),
      c));

  ASSERT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(1.00001, 1.00001),
      cv::Vec2d(1.00001, 1.00001),
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      c));

  ASSERT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 2),
      cv::Vec2d(2, 0),
      cv::Vec2d(1.00001, 1.00001),
      cv::Vec2d(1.00001, 1.00001),
      c));
}

TEST(IntersectionTests, ParallelSegments)
{
  cv::Vec2d c;
  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      c));
  EXPECT_FLOAT_EQ(0.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      c));
  EXPECT_FLOAT_EQ(0.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      c));
  EXPECT_FLOAT_EQ(10.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      c));
  EXPECT_FLOAT_EQ(10.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(20, 0),
      c));
  EXPECT_FLOAT_EQ(10.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(5, 0),
      cv::Vec2d(10, 0),
      c));
  EXPECT_FLOAT_EQ(10.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(5, 0),
      c));
  EXPECT_FLOAT_EQ(10.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(5, 0),
      cv::Vec2d(10, 0),
      c));
  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(5, 0),
      c));
  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(5, 0),
      cv::Vec2d(6, 0),
      c));
  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(6, 0),
      cv::Vec2d(5, 0),
      c));
  EXPECT_FLOAT_EQ(5.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(5, 0),
      cv::Vec2d(6, 0),
      c));
  EXPECT_FLOAT_EQ(6.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_TRUE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 0),
      cv::Vec2d(6, 0),
      cv::Vec2d(5, 0),
      c));
  EXPECT_FLOAT_EQ(6.0, c[0]);
  EXPECT_FLOAT_EQ(0.0, c[1]);

  EXPECT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(0, 1),
      cv::Vec2d(10, 1),
      c));

  EXPECT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(0, .00001),
      cv::Vec2d(10, .00001),
      c));

  EXPECT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(11, 0),
      cv::Vec2d(20, 0),
      c));

  EXPECT_FALSE(swri_geometry_util::LineSegmentIntersection(
      cv::Vec2d(0, 0),
      cv::Vec2d(10, 0),
      cv::Vec2d(10.00001, 0),
      cv::Vec2d(20, 0),
      c));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
