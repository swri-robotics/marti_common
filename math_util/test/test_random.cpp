// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#include <boost/random/mersenne_twister.hpp>
#include <gtest/gtest.h>
#include <math_util/random.h>

#include <ros/ros.h>

TEST(RandomTests, GetUniformRandomSample)
{
  boost::random::mt19937 gen;

  std::vector<int32_t> sample;
  math_util::GetUniformRandomSample<boost::random::mt19937>(gen, 0, 100, 10, sample);
  
  EXPECT_EQ(10, sample.size());
  
  math_util::GetUniformRandomSample<boost::random::mt19937>(gen, 0, 100, 90, sample);
  EXPECT_EQ(90, sample.size());
}

TEST(RandomTests, RandomGenerator)
{
  math_util::RandomGenerator gen;
  
  std::vector<int32_t> sample;
  gen.GetUniformRandomSample(0, 100, 10, sample);
  
  EXPECT_EQ(10, sample.size());
  
  gen.GetUniformRandomSample(0, 100, 90, sample);
  EXPECT_EQ(90, sample.size());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
