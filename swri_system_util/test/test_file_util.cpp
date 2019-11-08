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

#include <boost/filesystem/path.hpp>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_prefix.hpp>

#include <swri_system_util/file_util.h>

TEST(FileUtilTests, Uncomplete)
{
  std::string path1 = ament_index_cpp::get_package_prefix("swri_system_util");
  std::string path2 = path1 + "/src";
  std::string path3 = path1 + "/include/swri_system_util";

  EXPECT_EQ(boost::filesystem::path("./"), swri_system_util::NaiveUncomplete(path1, path1));
  EXPECT_EQ(boost::filesystem::path("src"), swri_system_util::NaiveUncomplete(path2, path1));
  EXPECT_EQ(boost::filesystem::path("include/swri_system_util"), swri_system_util::NaiveUncomplete(path3, path1));
  EXPECT_EQ(boost::filesystem::path("../"), swri_system_util::NaiveUncomplete(path1, path2));
  EXPECT_EQ(boost::filesystem::path("../../"), swri_system_util::NaiveUncomplete(path1, path3));
  EXPECT_EQ(boost::filesystem::path(""), swri_system_util::NaiveUncomplete(boost::filesystem::path(""), path1));
  EXPECT_EQ(boost::filesystem::path(""), swri_system_util::NaiveUncomplete(path1, boost::filesystem::path("")));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
