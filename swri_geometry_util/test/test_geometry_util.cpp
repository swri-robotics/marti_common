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

#include <swri_geometry_util/geometry_util.h>

TEST(IntersectionTests, ClosestPointToLinesInvalid)
{
  tf::Vector3 point;
  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(1, 0, 0),
    tf::Vector3(1, 0, 0),
    tf::Vector3(0, 1, 0),
    tf::Vector3(1, 0, 1),
    point));
 
  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(1, 0, 1),
    tf::Vector3(0, 1, 0),
    tf::Vector3(0, 0, 1),
    tf::Vector3(0, 0, 1),
    point));

  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(20, 0, 0),
    tf::Vector3(30, 0, 0),
    point));

  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(30, 0, 0),
    tf::Vector3(10, 0, 0),
    point));

  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(20, 10, 10),
    tf::Vector3(30, 10, 10),
    point));


  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(30, 10, 10),
    tf::Vector3(10, 10, 10),
    point));
}

TEST(IntersectionTests, ClosestPointToLines)
{
  tf::Vector3 point;
  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(0, 0, 0),
    tf::Vector3(0, 10, 0),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 0);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(0, 5, 0),
    tf::Vector3(0, 10, 0),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 0);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(5, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(0, 5, 0),
    tf::Vector3(0, 10, 0),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 0);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(0, -5, 0),
    tf::Vector3(0, 10, 0),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 0);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(0, 0, 20),
    tf::Vector3(0, 10, 20),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 10);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf::Vector3(0, 0, 0),
    tf::Vector3(10, 0, 0),
    tf::Vector3(0, 10, 20),
    tf::Vector3(0, 0, 20),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
