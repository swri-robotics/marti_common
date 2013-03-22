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
 * image_warp_util.cpp
 *
 *  Created on: Jul 25, 2012
 *      Author: kkozak
 */
 
#include <image_util/image_warp_util.h>

namespace image_util
{

  cv::Mat WarpImage(const cv::Mat& image, double roll, double pitch)
  {
    cv::Mat warped;
    
    // Initialize the camera matrix:
    cv::Mat K = cv::Mat::eye(cv::Size(3,3), CV_32F);
    K.at<float>(0,2) = static_cast<double>(image.cols - 1) / 2.0;
    K.at<float>(1,2) = static_cast<double>(image.rows - 1) / 2.0;
    
    cv::Mat T = cv::Mat::zeros(cv::Size(3,1), CV_32F);
    
    cv::Mat R = GetR(pitch, roll);
    
    cv::detail::PlaneWarper warper;
    warper.warp(image, K, R, T, cv::INTER_LANCZOS4, 0, warped);
    
    return warped;
  }
  
  cv::Mat GetR(double pitch, double roll, double yaw)
  {
    cv::Mat R1 = cv::Mat::eye(cv::Size(3,3), CV_32F);
    cv::Mat R2 = cv::Mat::eye(cv::Size(3,3), CV_32F);
    cv::Mat R3 = cv::Mat::eye(cv::Size(3,3), CV_32F);

    // do pitch first:
    R1.at<float>(0,0) = std::cos(pitch);
    R1.at<float>(0,2) = -std::sin(pitch);
    R1.at<float>(2,0) = std::sin(pitch);
    R1.at<float>(2,2) = std::cos(pitch);

    // Then roll
    R2.at<float>(1,1) = std::cos(roll);
    R2.at<float>(1,2) = std::sin(roll);
    R2.at<float>(2,1) = -std::sin(roll);
    R2.at<float>(2,2) = std::cos(roll);

    // Finally yaw
    R3.at<float>(0,0) = std::cos(yaw);
    R3.at<float>(0,1) = std::sin(yaw);
    R3.at<float>(1,0) = -std::sin(yaw);
    R3.at<float>(1,1) = std::cos(yaw);


    cv::Mat R = R3*R2*R1;

    return R;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // ImageWarpUtil()
  //
  //////////////////////////////////////////////////////////////////////////////
  PitchAndRollEstimator::PitchAndRollEstimator(const cv::Mat& im1,
                                               const cv::Mat& im2)
  {
    LoadImages(im1, im2);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // ImageWarpUtil()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool PitchAndRollEstimator::LoadImages(const cv::Mat& im1,
                                         const cv::Mat& im2)
  {
    kp1_.clear();
    kp2_.clear();

    // Set the images;
    im1_ = im1;
    im2_ = im2;

    // Get the keypoints and descriptors
    GetKeypoints(im1_,
                 kp1_,
                 descriptors1_,
                 200,
                 500);

    GetKeypoints(im2_,
                 kp2_,
                 descriptors2_,
                 200,
                 500);

    // Match the keypoints
    bool success = ComputeGeometricMatches();

    // If not successful clear out the variables
    if(!success)
    {
      ROS_ERROR("Loaded images are unsuitable for computing warp parameters");
      im1_.release();
      im2_.release();
      kp1_.clear();
      kp2_.clear();
      descriptors1_.release();
      descriptors2_.release();
    }

    // Initialize the camera matrix:
    K_ = cv::Mat::eye(cv::Size(3,3), CV_32F);
    K_.at<float>(0,2) = static_cast<double>(im1_.cols - 1) / 2.0;
    K_.at<float>(1,2) = static_cast<double>(im1_.rows - 1) / 2.0;

    T_ = cv::Mat::zeros(cv::Size(3,1), CV_32F);

    return success;
  }



  //////////////////////////////////////////////////////////////////////////////
  //
  // EstimateNominalAngle()
  //
  //////////////////////////////////////////////////////////////////////////////
  cv::Mat PitchAndRollEstimator::EstimateNominalAngle(double& nominal_pitch,
                                              double& nominal_roll,
                                              bool show_image_diff)
  {

    if(kp1_matched_.empty() || kp2_matched_.empty())
    {
      return cv::Mat();
    }

    ros::WallTime T1 = ros::WallTime::now();
    cv::Mat T_rigid = EstimateNominalAngle(kp1_matched_,
                                           kp2_matched_,
                                           cv::Size(im1_.cols, im1_.rows),
                                           nominal_pitch,
                                           nominal_roll);

    ros::WallTime T2 = ros::WallTime::now();

    ROS_ERROR("Estimate Nominal Angle time = %g", (T2 - T1).toSec());
    cv::Mat R = GetR(nominal_pitch, nominal_roll);

    if(show_image_diff)
    {
      // Do the warping and transformation and show the results
      cv::Mat warped_im1;
      cv::Mat warped_im2;

      warper_.warp(im1_, K_, R, T_, cv::INTER_LANCZOS4, 0, warped_im1);
      warper_.warp(im2_, K_, R, T_, cv::INTER_LANCZOS4, 0, warped_im2);

      cv::Mat temp_im;
      cv::warpAffine(warped_im1,
                     temp_im,
                     T_rigid,
                     cv::Size(warped_im1.cols, warped_im1.rows));

      cv::Mat sub = warped_im2 - temp_im;

      cv::namedWindow("Warped Subtraction");
      cv::imshow("Warped Subtraction", sub);

      // Now compare the result to the unwarped, transformed result:

      cv::warpAffine(im1_,
                     temp_im,
                     T_rigid,
                     cv::Size(im1_.cols, im1_.rows));

      cv::Mat sub2 = im2_ - temp_im;
      cv::namedWindow("Subtraction");
      cv::imshow("Subtraction", sub2);

      cv::namedWindow("im2_");
      cv::imshow("im2_", im2_);
      cv::waitKey(0);

    }

    return R;

  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // EstimateNominalAngle()
  //
  //////////////////////////////////////////////////////////////////////////////
  cv::Mat PitchAndRollEstimator::EstimateNominalAngle(const cv::Mat& points1,
                                                      const cv::Mat& points2,
                                                      const cv::Size& image_size,
                                                      double& nominal_pitch,
                                                      double& nominal_roll)
  {
    // Max number of iterations per angle, per scale
    const int32_t max_iterations = 5;
    const int32_t num_octaves = 5;

    double pitch_range = 0.02 * 3.14159/180.0;
    double min_pitch = -std::abs(pitch_range / 2.0);
    double max_pitch = std::abs(pitch_range / 2.0);

    double roll_range = 0.02*3.14159/180.0;
    double min_roll = -std::abs(roll_range / 2.0);
    double max_roll = std::abs(roll_range / 2.0);

    cv::Mat T_rigid_final = cv::Mat();
    for(int32_t octave_idx = 0; octave_idx < num_octaves; ++octave_idx)
    {
      double dp = (max_pitch - min_pitch) /
          static_cast<double>(max_iterations - 1);
      double dr = (max_roll - min_roll) /
          static_cast<double>(max_iterations - 1);

      double min_diff = 1e20;
      nominal_pitch = 0.0;
      nominal_roll = 0.0;


      for(int32_t pitch_idx = 0; pitch_idx < max_iterations; ++pitch_idx)
      {
        double cur_pitch = min_pitch + dp*pitch_idx;
        for(int32_t roll_idx = 0; roll_idx < max_iterations; ++roll_idx)
        {
          double cur_roll = min_roll + dr*roll_idx;

          cv::Mat kp1_warped;
          WarpPoints(cur_pitch,
                     cur_roll,
                     image_size,
                     points1,
                     kp1_warped);

          cv::Mat kp2_warped;
          WarpPoints(cur_pitch,
                     cur_roll,
                     image_size,
                     points2,
                     kp2_warped);

          cv::Mat T_rigid;
          cv::Mat T_affine;
          double rms_error;
          bool success = EstimateTransforms(kp1_warped,
                                            kp2_warped,
                                            T_affine,
                                            T_rigid,
                                            rms_error);
          if(!success)
          {
            continue;
          }

          double cur_diff = rms_error;

          if(cur_diff < min_diff)
          {

            min_diff = cur_diff;

            nominal_pitch = cur_pitch;
            nominal_roll = cur_roll;
            T_rigid_final = T_rigid;
          }


        }
      }


      min_pitch = nominal_pitch - std::abs(dp*2/3);
      max_pitch = nominal_pitch + std::abs(dp*2/3);


      min_roll = nominal_roll - std::abs(dr*2/3);
      max_roll = nominal_roll + std::abs(dr*2/3);

    }
    ROS_ERROR("Final pitch and roll: (%g, %g)",
              nominal_pitch*180.0/3.14159,
              nominal_roll*180.0/3.14159);

    cv::Mat R = GetR(nominal_pitch,
                     nominal_roll);


    return T_rigid_final;
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // WarpPoints()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimator::WarpPoints(double pitch,
                                         double roll,
                                         const cv::Size& image_size,
                                         const cv::Mat& pts_in,
                                         cv::Mat& pts_out)
  {

    // Initialize the camera matrix:
    cv::Mat K = cv::Mat::eye(cv::Size(3,3), CV_32F);
    K.at<float>(0,2) = static_cast<double>(image_size.width - 1) / 2.0;
    K.at<float>(1,2) = static_cast<double>(image_size.height - 1) / 2.0;

    cv::detail::PlaneWarper warper;

    cv::Mat T = cv::Mat::zeros(cv::Size(3,1), CV_32F);
    cv::Mat R = GetR(pitch, roll);
    pts_in.copyTo(pts_out);
    for(int32_t i = 0; i < pts_in.rows; ++i)
    {
      cv::Point2f pt;
      pt.x = pts_in.at<cv::Vec2f>(i,0).val[0];
      pt.y = pts_in.at<cv::Vec2f>(i,0).val[1];
      cv::Point2f pt2 = warper.warpPoint(pt, K, R, T);
      pts_out.at<cv::Vec2f>(i,0).val[0] = pt2.x + K.at<float>(0,2);
      pts_out.at<cv::Vec2f>(i,0).val[1] = pt2.y + K.at<float>(1,2);
    }
  }

  void PitchAndRollEstimator::WarpPoints(
      double pitch,
      double roll,
      const cv::Size& image_size,
      const std::vector<cv::KeyPoint>& pts_in,
      std::vector<cv::KeyPoint>& pts_out)
  {
    pts_out = pts_in;

    // Initialize the camera matrix:
    cv::Mat K = cv::Mat::eye(cv::Size(3,3), CV_32F);
    K.at<float>(0,2) = static_cast<double>(image_size.width - 1) / 2.0;
    K.at<float>(1,2) = static_cast<double>(image_size.height - 1) / 2.0;

    cv::detail::PlaneWarper warper;

    cv::Mat T = cv::Mat::zeros(cv::Size(3,1), CV_32F);
    cv::Mat R = GetR(pitch, roll);

    for(int32_t i = 0; i < (int)pts_in.size(); ++i)
    {
      pts_out[i].pt = warper.warpPoint(pts_in[i].pt, K, R, T);
      pts_out[i].pt.x += K.at<float>(0,2);
      pts_out[i].pt.y += K.at<float>(1,2);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // GetKeypoints()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool PitchAndRollEstimator::GetKeypoints(const cv::Mat& image,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::Mat& descriptors,
                                    int32_t min_keypoints,
                                    int32_t max_keypoints)
  {
    double auto_hessian = 1000.0;
    double min_hessian = 50;
    double max_hessian = 6000.0;

    const int32_t max_iterations = 10;
    int32_t cur_iter = 0;

    // Detect SURF keypoints in the frame.  Tune the hessian threshold to keep
    // the number of points between the specified number of points and the
    // specified bounds of the hessian threshold.
    while (auto_hessian < max_hessian &&
           auto_hessian > min_hessian &&
           cur_iter++ < max_iterations)
    {

      cv::SurfFeatureDetector detector(auto_hessian);
      keypoints.clear();
      detector.detect(image, keypoints);
      if ((int)keypoints.size() >= min_keypoints
          && (int)keypoints.size() <= max_keypoints)
      {
        break;
      }
      else if ((int)keypoints.size() < min_keypoints)
      {
        auto_hessian *= .90;
        ROS_ERROR("   Not enough features: %d < %d.  Changing hessian "
                  "threshold to %lf",
                  (int)keypoints.size(),
                  min_keypoints,
                  auto_hessian);
      }
      else
      {
        auto_hessian *= 1.2;
        ROS_ERROR("   To many features: %d > %d.  Changing hessian threshold "
                  "to %lf",
                  (int)keypoints.size(),
                  max_keypoints,
                  auto_hessian);
      }
    }

    // Extract SURF descriptors for each keypoint.
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(image,
                      keypoints,
                      descriptors);

    return (cur_iter <= max_iterations);
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // ComputeGeometricMatches()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool PitchAndRollEstimator::ComputeGeometricMatches()
  {
    if(im1_.empty() || im2_.empty())
    {
      ROS_ERROR("No images defined");
      return false;
    }

    // perform the matching first:
    // Compute the matching features between this frame and the previous one.
    std::vector<cv::DMatch> matches;
    cv::BruteForceMatcher<cv::L2<float> > matcher;
    matcher.match(descriptors1_,
                  descriptors2_,
                  matches);


    cv::Mat points1;
    cv::Mat points2;
    ConvertMatches(kp1_,
                   kp2_,
                   matches,
                   points1,
                   points2);

    // Compute the fundamental matrix which describes the camera motion
    // between the frames using a RANSAC process and get the set of inlier
    // matches which agree with the motion.
    cv::Mat fundamental_matrix;
    cv::Mat fund_inliers1;
    cv::Mat fund_inliers2;
    try
    {
      GetFundamentalInliers(points1,
                            points2,
                            fundamental_matrix,
                            fund_inliers1,
                            fund_inliers2);
    }
    catch(std::exception& e)
    {
      ROS_ERROR("Caught an exception when computing fundamental inliers:"
                " %s",e.what());
      return false;
    }

    ROS_INFO("Found %d fundamental inliers.", fund_inliers1.rows);

    cv::Mat T_affine;
    cv::Mat T_rigid;
    cv::Mat inliers1;
    cv::Mat inliers2;
    double rms_error;
    T_affine =  computeLooseRigid2DAffine(fund_inliers1,
                                          fund_inliers2,
                                          inliers1,
                                          inliers2,
                                          T_rigid,
                                          rms_error);

    if(T_affine.empty())
    {
      ROS_ERROR("Failed to comput loose 2D rigid transform.");
      return false;
    }

    kp1_matched_ = inliers1;
    kp2_matched_ = inliers2;

    return true;
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // EstimateTransforms()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool PitchAndRollEstimator::EstimateTransforms(cv::Mat& pts1,
                                         cv::Mat& pts2,
                                         cv::Mat& T_affine,
                                         cv::Mat& T_rigid,
                                         double& rms_error)
  {

    cv::Mat inliers1;
    cv::Mat inliers2;
    T_affine =  computeLooseRigid2DAffine(pts1,
                                          pts2,
                                          inliers1,
                                          inliers2,
                                          T_rigid,
                                          rms_error);

    if(T_affine.empty())
    {
      return false;
    }
    return true;

    return false;
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // WarpPoints()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimator::WarpPoints(double pitch,
                                 double roll,
                                 const cv::Mat& pts_in,
                                 cv::Mat& pts_out)
  {
    if(im1_.empty() || im2_.empty())
    {
      ROS_ERROR("Object not initialized. Pitch and roll not computed.  Perhaps"
                "call static implementation instead");
      return;
    }

    WarpPoints(pitch,
               roll,
               cv::Size(im1_.cols, im1_.rows),
               pts_in,
               pts_out);
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // WarpPoints()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimator::WarpAffinePoints(const cv::Mat& T,
                                 const cv::Mat& pts_in,
                                 cv::Mat& pts_out)
  {
    // Create augmented keypoint matrix:
    cv::Mat aug_mat(cv::Size(3,pts_in.rows), CV_32F);

    for(int32_t i = 0; i < pts_in.rows; ++i)
    {
      aug_mat.at<float>(i,0) = pts_in.at<cv::Vec2f>(0,i)[0];
      aug_mat.at<float>(i,1) = pts_in.at<cv::Vec2f>(0,i)[1];
      aug_mat.at<float>(i,2) = 1.0;
    }

    cv::Mat T_temp = T.t();
    cv::Mat pts_out_temp = aug_mat*T_temp;

    pts_out.release();
    pts_out.create(cv::Size(1,pts_in.rows), CV_32FC2);
    // Convert points back to proper form:
    for(int32_t i = 0; i < pts_in.rows; ++i)
    {
      pts_out.at<cv::Vec2f>(0,i)[0] = pts_out_temp.at<float>(i,0);
      pts_out.at<cv::Vec2f>(0,i)[1] = pts_out_temp.at<float>(i,1);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////   PitchAndRollEstimatorQueue   ////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////
  //
  // PitchAndRollEstimatorQueue()
  //
  //////////////////////////////////////////////////////////////////////////////
  PitchAndRollEstimatorQueue::PitchAndRollEstimatorQueue()
  {
    SetBufferSize();
    ComputeStats();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // SetBufferSize()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::SetBufferSize(int32_t buff_size)
  {
    pitches_.set_capacity(buff_size);
    rolls_.set_capacity(buff_size);
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // WarpPoints()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::WarpPoints(const cv::Mat& points_in,
                                              cv::Mat& points_out,
                                              const cv::Size& image_size,
                                              bool use_median)
  {
    double pitch;
    double roll;
    if(use_median)
    {
      GetMedianPitchAndRoll(pitch, roll);
    }
    else
    {
      GetMeanPitchAndRoll(pitch, roll);
    }

    PitchAndRollEstimator::WarpPoints(pitch,
                                      roll,
                                      image_size,
                                      points_in,
                                      points_out);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // Clear()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::Clear()
  {
    pitches_.clear();
    rolls_.clear();
    ComputeStats();
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // GenerateNewEstimate()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::GenerateNewEstimate(
      const cv::Mat& points1,
      const cv::Mat& points2,
      const cv::Size& image_size)
  {

    double pitch = 0.0;
    double roll = 0.0;
    cv::Mat T = PitchAndRollEstimator::EstimateNominalAngle(points1,
                                                            points2,
                                                            image_size,
                                                            pitch,
                                                            roll);

    if(!T.empty())
    {
      LoadNewData(pitch, roll);
      ComputeStats();
    }
  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // GenerateNewEstimate()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::GenerateNewEstimate(const cv::Mat& im1,
                                                       const cv::Mat& im2)
  {
    PitchAndRollEstimator est(im1, im2);
    double pitch = 0.0;
    double roll = 0.0;
    cv::Mat T = est.EstimateNominalAngle(pitch, roll);

    if(!T.empty())
    {
      LoadNewData(pitch, roll);
      ComputeStats();
    }

  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // GetMeanPitchAndRoll()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool PitchAndRollEstimatorQueue::GetMeanPitchAndRoll(double& pitch,
                                                       double& roll)
  {
    pitch = mean_pitch_;
    roll = mean_roll_;

    return pitches_.size() > 0;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // GetMedianPitchAndRoll()
  //
  //////////////////////////////////////////////////////////////////////////////
  bool PitchAndRollEstimatorQueue::GetMedianPitchAndRoll(double& pitch,
                                                         double& roll)
  {
    pitch = median_pitch_;
    roll = median_roll_;

    return pitches_.size() > 0;

  }


  //////////////////////////////////////////////////////////////////////////////
  //
  // LoadNewData()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::LoadNewData(double new_pitch,
                                               double new_roll)
  {
    pitches_.push_back(new_pitch);
    rolls_.push_back(new_roll);
    ComputeStats();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // ComputeStats()
  //
  //////////////////////////////////////////////////////////////////////////////
  void PitchAndRollEstimatorQueue::ComputeStats()
  {
    mean_pitch_ = 0.0;
    mean_roll_ = 0.0;
    median_pitch_ = 0.0;
    median_roll_ = 0.0;

    if(pitches_.empty())
    {
      return;
    }

    std::vector<double> temp_pitch;
    std::vector<double> temp_roll;
    temp_pitch.assign(pitches_.begin(), pitches_.end());
    temp_roll.assign(rolls_.begin(), rolls_.end());

    std::sort(temp_pitch.begin(), temp_pitch.end());
    std::sort(temp_roll.begin(), temp_roll.end());

    double pitch_sum = 0.0;
    double roll_sum = 0.0;

    for(int32_t i = 0; i < (int32_t)temp_pitch.size(); ++i)
    {
      pitch_sum += temp_pitch[i];
      roll_sum += temp_roll[i];
    }

    double N = static_cast<double>(temp_pitch.size());
    mean_pitch_ = pitch_sum / N;
    mean_roll_ = roll_sum / N;

    int32_t mid_idx = static_cast<int32_t>(temp_pitch.size() - 1) / 2;
    if(temp_pitch.size() % 2 == 0)
    {

      median_pitch_ = (temp_pitch[mid_idx] + temp_pitch[mid_idx + 1]) / 2.0;
      median_roll_ = (temp_roll[mid_idx] + temp_roll[mid_idx + 1]) / 2.0;
    }
    else
    {
      median_pitch_ = temp_pitch[mid_idx];
      median_roll_ = temp_roll[mid_idx];
    }

  }

}


