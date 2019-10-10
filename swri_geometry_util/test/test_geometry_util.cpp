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

TEST(GeometryUtilTests, ClosestPointToLinesInvalid)
{
  tf2::Vector3 point;
  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(1, 0, 0),
    tf2::Vector3(1, 0, 0),
    tf2::Vector3(0, 1, 0),
    tf2::Vector3(1, 0, 1),
    point));
 
  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(1, 0, 1),
    tf2::Vector3(0, 1, 0),
    tf2::Vector3(0, 0, 1),
    tf2::Vector3(0, 0, 1),
    point));

  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(20, 0, 0),
    tf2::Vector3(30, 0, 0),
    point));

  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(30, 0, 0),
    tf2::Vector3(10, 0, 0),
    point));

  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(20, 10, 10),
    tf2::Vector3(30, 10, 10),
    point));


  ASSERT_FALSE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(30, 10, 10),
    tf2::Vector3(10, 10, 10),
    point));
}

TEST(GeometryUtilTests, ClosestPointToLines)
{
  tf2::Vector3 point;
  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(0, 10, 0),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 0);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(0, 5, 0),
    tf2::Vector3(0, 10, 0),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 0);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(5, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(0, 5, 0),
    tf2::Vector3(0, 10, 0),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 0);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(0, -5, 0),
    tf2::Vector3(0, 10, 0),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 0);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(0, 0, 20),
    tf2::Vector3(0, 10, 20),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 10);

  ASSERT_TRUE(swri_geometry_util::ClosestPointToLines(
    tf2::Vector3(0, 0, 0),
    tf2::Vector3(10, 0, 0),
    tf2::Vector3(0, 10, 20),
    tf2::Vector3(0, 0, 20),
    point));
  EXPECT_FLOAT_EQ(point.x(), 0);
  EXPECT_FLOAT_EQ(point.y(), 0);
  EXPECT_FLOAT_EQ(point.z(), 10);
}

TEST(GeometryUtilTests, ProjectPointToLineSegment3D)
{
  tf2::Vector3 p1(0, 0, 0);
  tf2::Vector3 p2(10, 0, 0);
  tf2::Vector3 p3(2, 0, 0);
  tf2::Vector3 p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(p3.x(), p4.x());
  EXPECT_FLOAT_EQ(p3.y(), p4.y());
  EXPECT_FLOAT_EQ(p3.z(), p4.z());

  p4 = swri_geometry_util::ProjectToLineSegment(p2, p1, p3);
  EXPECT_FLOAT_EQ(p3.x(), p4.x());
  EXPECT_FLOAT_EQ(p3.y(), p4.y());
  EXPECT_FLOAT_EQ(p3.z(), p4.z());

  p3 = tf2::Vector3(0, 0, 0);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(p3.x(), p4.x());
  EXPECT_FLOAT_EQ(p3.y(), p4.y());
  EXPECT_FLOAT_EQ(p3.z(), p4.z());

  p3 = tf2::Vector3(0, 1, 0);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(0, p4.x());
  EXPECT_FLOAT_EQ(0, p4.y());
  EXPECT_FLOAT_EQ(0, p4.z());

  p3 = tf2::Vector3(5, -1, 0);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(5, p4.x());
  EXPECT_FLOAT_EQ(0, p4.y());
  EXPECT_FLOAT_EQ(0, p4.z());

  p3 = tf2::Vector3(5, -1, 10);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(5, p4.x());
  EXPECT_FLOAT_EQ(0, p4.y());
  EXPECT_FLOAT_EQ(0, p4.z());

  p3 = tf2::Vector3(-5, -1, 10);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(0, p4.x());
  EXPECT_FLOAT_EQ(0, p4.y());
  EXPECT_FLOAT_EQ(0, p4.z());

  p3 = tf2::Vector3(15, -1, 10);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(10, p4.x());
  EXPECT_FLOAT_EQ(0, p4.y());
  EXPECT_FLOAT_EQ(0, p4.z());

  p1 = tf2::Vector3(0, 0, 0);
  p2 = tf2::Vector3(10, 10, 10);
  p3 = tf2::Vector3(1, 1, 1);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(1, p4.x());
  EXPECT_FLOAT_EQ(1, p4.y());
  EXPECT_FLOAT_EQ(1, p4.z());

  p2 = tf2::Vector3(10, 10, 0);
  p3 = tf2::Vector3(0, 10, 0);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(5, p4.x());
  EXPECT_FLOAT_EQ(5, p4.y());
  EXPECT_FLOAT_EQ(0, p4.z());

  p4 = swri_geometry_util::ProjectToLineSegment(p2, p1, p3);
  EXPECT_FLOAT_EQ(5, p4.x());
  EXPECT_FLOAT_EQ(5, p4.y());
  EXPECT_FLOAT_EQ(0, p4.z());

  p3 = tf2::Vector3(-100, 10, 0);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(0, p4.x());
  EXPECT_FLOAT_EQ(0, p4.y());
  EXPECT_FLOAT_EQ(0, p4.z());
}

TEST(GeometryUtilTests, ProjectPointToLineSegment2D)
{
  cv::Vec2d p1(0, 0);
  cv::Vec2d p2(10, 0);
  cv::Vec2d p3(2, 0);
  cv::Vec2d p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(p3[0], p4[0]);
  EXPECT_FLOAT_EQ(p3[1], p4[1]);

  p4 = swri_geometry_util::ProjectToLineSegment(p2, p1, p3);
  EXPECT_FLOAT_EQ(p3[0], p4[0]);
  EXPECT_FLOAT_EQ(p3[1], p4[1]);

  p3 = cv::Vec2d(0, 0);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(p3[0], p4[0]);
  EXPECT_FLOAT_EQ(p3[1], p4[1]);

  p3 = cv::Vec2d(0, 1);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(0, p4[0]);
  EXPECT_FLOAT_EQ(0, p4[1]);

  p3 = cv::Vec2d(5, -1);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(5, p4[0]);
  EXPECT_FLOAT_EQ(0, p4[1]);

  p3 = cv::Vec2d(-5, -1);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(0, p4[0]);
  EXPECT_FLOAT_EQ(0, p4[1]);

  p3 = cv::Vec2d(15, -1);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(10, p4[0]);
  EXPECT_FLOAT_EQ(0, p4[1]);

  p1 = cv::Vec2d(0, 0);
  p2 = cv::Vec2d(10, 10);
  p3 = cv::Vec2d(1, 1);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(1, p4[0]);
  EXPECT_FLOAT_EQ(1, p4[1]);

  p2 = cv::Vec2d(10, 10);
  p3 = cv::Vec2d(0, 10);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(5, p4[0]);
  EXPECT_FLOAT_EQ(5, p4[1]);

  p4 = swri_geometry_util::ProjectToLineSegment(p2, p1, p3);
  EXPECT_FLOAT_EQ(5, p4[0]);
  EXPECT_FLOAT_EQ(5, p4[1]);

  p3 = cv::Vec2d(-100, 10);
  p4 = swri_geometry_util::ProjectToLineSegment(p1, p2, p3);
  EXPECT_FLOAT_EQ(0, p4[0]);
  EXPECT_FLOAT_EQ(0, p4[1]);
}

TEST(GeometryUtilTests, DistanceFromLineSegment3D)
{
  tf2::Vector3 p1(0, 0, 0);
  tf2::Vector3 p2(10, 0, 0);
  tf2::Vector3 p3(2, 0, 0);
  EXPECT_FLOAT_EQ(0, swri_geometry_util::DistanceFromLineSegment(p1, p2, p3));
  EXPECT_FLOAT_EQ(0, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = tf2::Vector3(0, 0, 0);
  EXPECT_FLOAT_EQ(0, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = tf2::Vector3(0, 1, 0);
  EXPECT_FLOAT_EQ(1, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = tf2::Vector3(5, -1, 0);
  EXPECT_FLOAT_EQ(1, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = tf2::Vector3(5, -1, 10);
  EXPECT_FLOAT_EQ(std::sqrt(101), swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = tf2::Vector3(-5, -1, 10);
  EXPECT_FLOAT_EQ(std::sqrt(126), swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = tf2::Vector3(15, -1, 10);
  EXPECT_FLOAT_EQ(std::sqrt(126), swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p1 = tf2::Vector3(0, 0, 0);
  p2 = tf2::Vector3(10, 10, 10);
  p3 = tf2::Vector3(1, 1, 1);
  EXPECT_FLOAT_EQ(0, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p2 = tf2::Vector3(10, 10, 0);
  p3 = tf2::Vector3(0, 10, 0);
  EXPECT_FLOAT_EQ(std::sqrt(50), swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));
  EXPECT_FLOAT_EQ(std::sqrt(50), swri_geometry_util::DistanceFromLineSegment(p1, p2, p3));

  p3 = tf2::Vector3(-100, 10, 0);
  EXPECT_FLOAT_EQ(std::sqrt(10100), swri_geometry_util::DistanceFromLineSegment(p1, p2, p3));
}

TEST(GeometryUtilTests, DistanceFromLineSegment2D)
{
  cv::Vec2d p1(0, 0);
  cv::Vec2d p2(10, 0);
  cv::Vec2d p3(2, 0);
  EXPECT_FLOAT_EQ(0, swri_geometry_util::DistanceFromLineSegment(p1, p2, p3));
  EXPECT_FLOAT_EQ(0, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = cv::Vec2d(0, 0);
  EXPECT_FLOAT_EQ(0, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = cv::Vec2d(0, 1);
  EXPECT_FLOAT_EQ(1, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p3 = cv::Vec2d(5, -1);
  EXPECT_FLOAT_EQ(1, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p1 = cv::Vec2d(0, 0);
  p2 = cv::Vec2d(10, 10);
  p3 = cv::Vec2d(1, 1);
  EXPECT_FLOAT_EQ(0, swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));

  p2 = cv::Vec2d(10, 10);
  p3 = cv::Vec2d(0, 10);
  EXPECT_FLOAT_EQ(std::sqrt(50), swri_geometry_util::DistanceFromLineSegment(p2, p1, p3));
  EXPECT_FLOAT_EQ(std::sqrt(50), swri_geometry_util::DistanceFromLineSegment(p1, p2, p3));

  p3 = cv::Vec2d(-100, 10);
  EXPECT_FLOAT_EQ(std::sqrt(10100), swri_geometry_util::DistanceFromLineSegment(p1, p2, p3));
}


TEST(GeometryUtilTests, PointInPolygon)
{
  std::vector<cv::Vec2d> polygon1;
  polygon1.push_back(cv::Vec2d(1, 1));
  polygon1.push_back(cv::Vec2d(1, -1));
  polygon1.push_back(cv::Vec2d(-1, -1));
  polygon1.push_back(cv::Vec2d(-1, 1));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon1, cv::Vec2d(0, 0)));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon1, cv::Vec2d(0.99, 0.99)));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon1, cv::Vec2d(-0.99, 0.99)));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon1, cv::Vec2d(-0.99, -0.99)));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon1, cv::Vec2d(0.99, -0.99)));
  EXPECT_FALSE(swri_geometry_util::PointInPolygon(polygon1, cv::Vec2d(2, 0)));
  EXPECT_FALSE(swri_geometry_util::PointInPolygon(polygon1, cv::Vec2d(-2, 0)));
  EXPECT_FALSE(swri_geometry_util::PointInPolygon(polygon1, cv::Vec2d(2, -2)));

  std::vector<cv::Vec2d> polygon2;
  polygon2.push_back(cv::Vec2d(1, 1));
  polygon2.push_back(cv::Vec2d(1, -1));
  polygon2.push_back(cv::Vec2d(-1, -1));
  polygon2.push_back(cv::Vec2d(-1, 1));
  polygon2.push_back(cv::Vec2d(1, 1));
  polygon2.push_back(cv::Vec2d(1, 1));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon2, cv::Vec2d(0, 0)));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon2, cv::Vec2d(0.99, 0.99)));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon2, cv::Vec2d(-0.99, 0.99)));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon2, cv::Vec2d(-0.99, -0.99)));
  EXPECT_TRUE(swri_geometry_util::PointInPolygon(polygon2, cv::Vec2d(0.99, -0.99)));
  EXPECT_FALSE(swri_geometry_util::PointInPolygon(polygon2, cv::Vec2d(2, 0)));
  EXPECT_FALSE(swri_geometry_util::PointInPolygon(polygon2, cv::Vec2d(-2, 0)));
  EXPECT_FALSE(swri_geometry_util::PointInPolygon(polygon2, cv::Vec2d(2, -2)));
}

TEST(GeometryUtilTests, DistanceFromPolygon)
{
  std::vector<cv::Vec2d> polygon1;
  polygon1.push_back(cv::Vec2d(1, 1));
  polygon1.push_back(cv::Vec2d(1, -1));
  polygon1.push_back(cv::Vec2d(-1, -1));
  polygon1.push_back(cv::Vec2d(-1, 1));
  EXPECT_FLOAT_EQ(1.0, swri_geometry_util::DistanceFromPolygon(polygon1, cv::Vec2d(0, 0)));
  EXPECT_FLOAT_EQ(0.01, swri_geometry_util::DistanceFromPolygon(polygon1, cv::Vec2d(0.99, 0.99)));
  EXPECT_FLOAT_EQ(0.01, swri_geometry_util::DistanceFromPolygon(polygon1, cv::Vec2d(-0.99, 0.99)));
  EXPECT_FLOAT_EQ(0.01, swri_geometry_util::DistanceFromPolygon(polygon1, cv::Vec2d(-0.99, -0.99)));
  EXPECT_FLOAT_EQ(0.01, swri_geometry_util::DistanceFromPolygon(polygon1, cv::Vec2d(0.99, -0.99)));
  EXPECT_FLOAT_EQ(1, swri_geometry_util::DistanceFromPolygon(polygon1, cv::Vec2d(2, 0)));
  EXPECT_FLOAT_EQ(1,swri_geometry_util::DistanceFromPolygon(polygon1, cv::Vec2d(-2, 0)));
  EXPECT_FLOAT_EQ(std::sqrt(2),swri_geometry_util::DistanceFromPolygon(polygon1, cv::Vec2d(2, -2)));

  std::vector<cv::Vec2d> polygon2;
  polygon2.push_back(cv::Vec2d(1, 1));
  polygon2.push_back(cv::Vec2d(1, -1));
  polygon2.push_back(cv::Vec2d(-1, -1));
  polygon2.push_back(cv::Vec2d(-1, 1));
  polygon2.push_back(cv::Vec2d(1, 1));
  polygon2.push_back(cv::Vec2d(1, 1));
  EXPECT_FLOAT_EQ(1.0, swri_geometry_util::DistanceFromPolygon(polygon2, cv::Vec2d(0, 0)));
  EXPECT_FLOAT_EQ(0.01, swri_geometry_util::DistanceFromPolygon(polygon2, cv::Vec2d(0.99, 0.99)));
  EXPECT_FLOAT_EQ(0.01, swri_geometry_util::DistanceFromPolygon(polygon2, cv::Vec2d(-0.99, 0.99)));
  EXPECT_FLOAT_EQ(0.01, swri_geometry_util::DistanceFromPolygon(polygon2, cv::Vec2d(-0.99, -0.99)));
  EXPECT_FLOAT_EQ(0.01, swri_geometry_util::DistanceFromPolygon(polygon2, cv::Vec2d(0.99, -0.99)));
  EXPECT_FLOAT_EQ(1, swri_geometry_util::DistanceFromPolygon(polygon2, cv::Vec2d(2, 0)));
  EXPECT_FLOAT_EQ(1,swri_geometry_util::DistanceFromPolygon(polygon2, cv::Vec2d(-2, 0)));
  EXPECT_FLOAT_EQ(std::sqrt(2),swri_geometry_util::DistanceFromPolygon(polygon2, cv::Vec2d(2, -2)));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
