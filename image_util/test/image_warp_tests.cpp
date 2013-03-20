// *****************************************************************************
//
// Copyright (C) 2011 All Right Reserved, Southwest Research Institute® (SwRI®)
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
/*
 * image_warp_tests.cpp
 *
 *  Created on: Jul 25, 2012
 *      Author: kkozak
 */


// GTEST Library
#include <gtest/gtest.h>

// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>

// Boost Libraries
#include <boost/filesystem.hpp>

#include <image_util/image_normalization.h>
#include <image_util/motion_estimation.h>
#include <image_util/image_warp_util.h> // The library to test

/**
 * @brief      A utility function that loads, rectifies and normalizes an image
 *
 * @param calibration_file
 * @param normalization_image
 * @param image_file
 *
 * @return
 */
cv::Mat rectify_image(const std::string& calibration_file,
                            const std::string& normalization_image,
                            const cv::Mat& image)
{
  std::string camera_name;
  sensor_msgs::CameraInfo camera_info;
  cv::Mat norm_image;

  // Read single camera calibration file
  bool success = camera_calibration_parsers::readCalibration(calibration_file,
                                                             camera_name,
                                                             camera_info);
  EXPECT_TRUE(success);

  if(!success)
  {
    ROS_ERROR("  Failed to load camera calibration: %s",
              calibration_file.c_str());
  }

  ROS_INFO("Loading normalization image: %s",
           normalization_image.c_str());
  // Read normalization image
  cv::Mat temp = cv::imread(normalization_image, cv::IMREAD_GRAYSCALE);
  if(temp.empty())
  {
    ROS_ERROR("  Failed to load normalization image: %s",
              normalization_image.c_str());
  }
  else
  {
    temp.convertTo(norm_image,
                   CV_32FC1,
                   1.0 / 255.0,
                   0.0);
  }


  //cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);

  // Normalize the illumination of the frame image.
  cv::Mat normalized_image;
  image_util::normalize_illumination(norm_image,
                                        image,
                                        normalized_image);


  cv::Mat rectified_image;

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);
  model.rectifyImage(normalized_image,
                     rectified_image,
                     CV_INTER_LANCZOS4);

  return rectified_image;
}

/**
 * @brief      A comparator that returns true if the last number in the first
 *             filename is less than the last number in the second filename. The
 *             purpose of this function is to allow us to put filenames in order
 *             that have numbering like foo1.jpg, foo2.jpg... foo10.jpg, whereas
 *             standard ordering would put foo10.jpg after foo1.jpg and before
 *             foo2.jpg
 *
 * @param[in]  p1   The boost path variable associated with the first filename
 * @param[in]  p2   The boost path variable associated with the second filename
 *
 * @retval     Returns true if the last number in the first filename is less
 *             than the last number in the second filename.  If there are no
 *             numbers in the name, then the result is simply the direct string
 *             comparison (p1.string() < p2.string())
 */
bool cmp_names(boost::filesystem::path p1,
               boost::filesystem::path p2)
{
  int num1;
  int num2;
  std::string digits1;
  std::string digits2;
  std::string s1 = p1.string();
  std::string s2 = p2.string();
  std::string::reverse_iterator ri1 = s1.rbegin();
  bool success = false;
  while(ri1 != s1.rend())
  {
    if(std::isdigit(*ri1))
    {
      success = true;
      while(std::isdigit(*ri1))
      {
        digits1.push_back(*(ri1++));
      }
      break;
    }
    ri1++;
  }
  if(!success)
  {
    return (s1 < s2);
  }
  else
  {
    std::reverse(digits1.begin(), digits1.end());
    num1 = std::atoi(digits1.c_str());
  }

  std::string::reverse_iterator ri2 = s2.rbegin();
  success = false;
  while(ri2 != s2.rend())
  {
    if(std::isdigit(*ri2))
    {
      success = true;
      while(std::isdigit(*ri2))
      {
        digits2.push_back(*(ri2++));
      }
      break;
    }
    ri2++;
  }
  if(!success)
  {
    return (s1 < s2);
  }
  else
  {
    std::reverse(digits2.begin(), digits2.end());
    num2 = std::atoi(digits2.c_str());
  }


  return (num1 < num2);


}

////////////////////////////////////////////////////////////////////////////////
//
// Test1
//
////////////////////////////////////////////////////////////////////////////////
TEST(ImageWarpTests, Test1)
{

  bool display_images = false;

  std::string calibration_file("");
  std::string normalization_image("");
  std::string image_dir;


  // Read in parameters
  EXPECT_TRUE(ros::param::get("calibration_file", calibration_file));
  EXPECT_TRUE(ros::param::get("normalization_image", normalization_image));
  EXPECT_TRUE(ros::param::get("image_warp_test_image_dir", image_dir));
  if(!image_dir.empty())
  {
    // Add a trailing slash if not already there
    if(image_dir[image_dir.size()-1] != '/')
    {
      image_dir += "/";
    }
  }
  std::string first_frame_file = image_dir + "frame1.png";
  std::string second_frame_file = image_dir + "frame3.png";

  cv::Mat im1 = cv::imread(first_frame_file, cv::IMREAD_GRAYSCALE);
  cv::Mat im2 = cv::imread(second_frame_file, cv::IMREAD_GRAYSCALE);


  if(!calibration_file.empty() || !normalization_image.empty())
  {
    im1 =  rectify_image(calibration_file,
                         normalization_image,
                         im1);

    im2 =  rectify_image(calibration_file,
                         normalization_image,
                         im2);
  }
  else
  {
    ROS_ERROR("Calibration file and/or normalization file not specified, using"
              "unrectified/unnormalized images");
  }

  // The images are now prepared and can be used in the image_warp_util

  if(display_images)
  {
    cv::namedWindow("Image 1");
    cv::imshow("Image 1", im1);
    cv::namedWindow("Image 2");
    cv::imshow("Image 2", im2);
    cv::waitKey(10000);
  }

  image_util::PitchAndRollEstimator warper;

  bool success = warper.LoadImages(im1, im2);
  EXPECT_TRUE(success);
  if(!success)
  {
    ROS_ERROR("Image load failed. Exiting");
    return;
  }

  double nominal_pitch = 0.0;
  double nominal_roll = 0.0;
  cv::Mat R = warper.EstimateNominalAngle(nominal_pitch,
                                          nominal_roll,
                                          true);

  cv::destroyAllWindows();
  cv::waitKey(10);

}


////////////////////////////////////////////////////////////////////////////////
//
// Test2
//
////////////////////////////////////////////////////////////////////////////////
TEST(ImageWarpTests, Test2)
{
  std::string image_dir;


  // Read in parameters
  EXPECT_TRUE(ros::param::get("image_sequence_dir", image_dir));

  boost::filesystem::path image_path(image_dir);

  bool success = boost::filesystem::exists(image_path);
  EXPECT_TRUE(success);
  if(!success)
  {
    return;
  }

  success = boost::filesystem::is_directory(image_path);
  EXPECT_TRUE(success);
  if(!success)
  {
    return;
  }

  std::vector<boost::filesystem::path> paths_vec;
  std::copy(boost::filesystem::directory_iterator(image_path),
            boost::filesystem::directory_iterator(),
            std::back_inserter(paths_vec));

  std::sort(paths_vec.begin(), paths_vec.end(), cmp_names);

  std::vector<boost::filesystem::path> file_vec;
  for(int32_t i = 0; i < (int)paths_vec.size(); ++i)
  {
    if(boost::filesystem::is_regular_file(paths_vec[i]))
    {
      if (paths_vec[i].extension() == ".jpg" ||
          paths_vec[i].extension() == ".png")
      {
        file_vec.push_back(paths_vec[i]);
      }
    }
  }

  int32_t vect_len = file_vec.size();
  EXPECT_TRUE(vect_len > 0);
  if(vect_len <= 0)
  {
    return;
  }

  for(int32_t i = 0; i < (int)file_vec.size(); ++i)
  {
    ROS_ERROR("%s", file_vec[i].c_str());
  }

  std::vector<double> pitches;
  std::vector<double> rolls;
  for(int32_t i = 0; i < (int)file_vec.size()-1; i += 4)
  {
    ROS_ERROR("Processing images:\n%s\n%s",
              file_vec[i].c_str(),
              file_vec[i+1].c_str());
    std::string first_frame(file_vec[i].string());
    std::string second_frame(file_vec[i+1].string());

    cv::Mat im1 = cv::imread(first_frame, cv::IMREAD_GRAYSCALE);
    cv::Mat im2 = cv::imread(second_frame, cv::IMREAD_GRAYSCALE);

    image_util::PitchAndRollEstimator warper;
    if(!warper.LoadImages(im1, im2))
    {
      ROS_ERROR("Image load failed. Skipping");
      continue;
    }

    double nominal_pitch = 0.0;
    double nominal_roll = 0.0;
    cv::Mat R = warper.EstimateNominalAngle(nominal_pitch,
                                            nominal_roll,
                                            true);

    pitches.push_back(nominal_pitch);
    rolls.push_back(nominal_roll);
  }

  double pitch_sum = 0.0;
  double roll_sum = 0.0;
  for(int32_t i = 0; i < (int)pitches.size(); ++i)
  {
    ROS_ERROR("pitch = %g, roll = %g",
              pitches[i] * 180.0 / 3.14159,
              rolls[i] * 180.0 / 3.14159);
    pitch_sum += pitches[i];
    roll_sum += rolls[i];
  }

  std::sort(pitches.begin(), pitches.end());
  std::sort(rolls.begin(), rolls.end());
  int N = pitches.size();
  int mid_idx = N/2;
  double median_pitch = pitches[mid_idx];
  double median_roll = rolls[mid_idx];

  double mean_pitch = pitch_sum / ((double)pitches.size());
  double mean_roll = roll_sum / ((double)pitches.size());
  ROS_ERROR("Mean pitch = %g, Mean roll = %g, "
            "median pitch = %g, median roll = %g",
            mean_pitch * 180.0 / 3.14159,
            mean_roll * 180.0 / 3.14159,
            median_pitch * 180.0 / 3.14159,
            median_roll * 180.0 / 3.14159);

}


TEST(ImageWarpTests, Test3)
{
  image_util::PitchAndRollEstimatorQueue prq;
  prq.SetBufferSize(3);
  double pitch;
  double roll;
  EXPECT_FALSE(prq.GetMeanPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 0.0);
  EXPECT_FLOAT_EQ(roll, 0.0);
  EXPECT_FALSE(prq.GetMedianPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 0.0);
  EXPECT_FLOAT_EQ(roll, 0.0);

  prq.LoadNewData(1.0, 0.0);

  EXPECT_TRUE(prq.GetMeanPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 1.0);
  EXPECT_FLOAT_EQ(roll, 0.0);
  EXPECT_TRUE(prq.GetMedianPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 1.0);
  EXPECT_FLOAT_EQ(roll, 0.0);

  prq.LoadNewData(2.0, -1.0);
  EXPECT_TRUE(prq.GetMeanPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 1.5);
  EXPECT_FLOAT_EQ(roll, -0.5);
  EXPECT_TRUE(prq.GetMedianPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 1.5);
  EXPECT_FLOAT_EQ(roll, -0.5);

  prq.LoadNewData(10.0, 4.0);
  EXPECT_TRUE(prq.GetMeanPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 13.0/3.0);
  EXPECT_FLOAT_EQ(roll, 1.0);
  EXPECT_TRUE(prq.GetMedianPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 2.0);
  EXPECT_FLOAT_EQ(roll, 0.0);


  prq.LoadNewData(10.0, -10.0);
  EXPECT_TRUE(prq.GetMeanPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 22.0/3.0);
  EXPECT_FLOAT_EQ(roll, -7.0/3.0);
  EXPECT_TRUE(prq.GetMedianPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 10.0);
  EXPECT_FLOAT_EQ(roll, -1.0);

  prq.Clear();
  EXPECT_FALSE(prq.GetMeanPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 0.0);
  EXPECT_FLOAT_EQ(roll, 0.0);
  EXPECT_FALSE(prq.GetMedianPitchAndRoll(pitch, roll));
  EXPECT_FLOAT_EQ(pitch, 0.0);
  EXPECT_FLOAT_EQ(roll, 0.0);

}

// Run the tests
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "image_warp_util_tests");

  return RUN_ALL_TESTS();
}

