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

#include <ros/ros.h>

#include <swri_transform_util/georeference.h>

TEST(GeoreferenceTests, Load)
{
  std::string filename;
  ASSERT_TRUE(ros::param::get("geo_file", filename));

  swri_transform_util::GeoReference georeference(filename);
  ASSERT_TRUE(georeference.Load());

  EXPECT_EQ(std::string("wgs84"), georeference.Datum());
  EXPECT_EQ(std::string("utm"), georeference.Projection());
  EXPECT_EQ(29184, georeference.Width());
  EXPECT_EQ(15872, georeference.Height());
  EXPECT_EQ(512, georeference.TileSize());
  EXPECT_EQ("jpg", georeference.Extension());
}

TEST(GeoreferenceTests, LoadExtension)
{
  std::string filename;
  ASSERT_TRUE(ros::param::get("geo_file_extension", filename));

  swri_transform_util::GeoReference georeference(filename);
  ASSERT_TRUE(georeference.Load());

  EXPECT_EQ(std::string("wgs84"), georeference.Datum());
  EXPECT_EQ(std::string("utm"), georeference.Projection());
  EXPECT_EQ(29184, georeference.Width());
  EXPECT_EQ(15872, georeference.Height());
  EXPECT_EQ(512, georeference.TileSize());
  EXPECT_EQ("png", georeference.Extension());
}

// TODO(malban): Test coordinate/pixel transforms.

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Initialize the ROS core parameters can be loaded from the launch file
  ros::init(argc, argv, "test_georeference");

  return RUN_ALL_TESTS();
}
