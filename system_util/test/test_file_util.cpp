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

#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem/path.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include <system_util/file_util.h>

TEST(FileUtilTests, Uncomplete)
{
  std::string path1;
  ASSERT_TRUE(ros::param::get("path1", path1));

  std::string path2;
  ASSERT_TRUE(ros::param::get("path2", path2));

  std::string path3;
  ASSERT_TRUE(ros::param::get("path3", path3));

  EXPECT_EQ(boost::filesystem::path("./"), system_util::NaiveUncomplete(path1, path1));
  EXPECT_EQ(boost::filesystem::path("src"), system_util::NaiveUncomplete(path2, path1));
  EXPECT_EQ(boost::filesystem::path("include/system_util"), system_util::NaiveUncomplete(path3, path1));
  EXPECT_EQ(boost::filesystem::path("../"), system_util::NaiveUncomplete(path1, path2));
  EXPECT_EQ(boost::filesystem::path("../../"), system_util::NaiveUncomplete(path1, path3));
  EXPECT_EQ(boost::filesystem::path(""), system_util::NaiveUncomplete(boost::filesystem::path(""), path1));
  EXPECT_EQ(boost::filesystem::path(""), system_util::NaiveUncomplete(path1, boost::filesystem::path("")));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_file_util");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
