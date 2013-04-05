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

#include <ros/ros.h>

#include <transform_util/georeference.h>

TEST(GeoreferenceTests, Load)
{
  std::string filename;
  ASSERT_TRUE(ros::param::get("geo_file", filename));

  transform_util::GeoReference georeference(filename);
  ASSERT_TRUE(georeference.Load());

  EXPECT_EQ(std::string("wgs84"), georeference.Datum());
  EXPECT_EQ(std::string("utm"), georeference.Projection());
  EXPECT_EQ(29184, georeference.Width());
  EXPECT_EQ(15872, georeference.Height());
  EXPECT_EQ(512, georeference.TileSize());
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
