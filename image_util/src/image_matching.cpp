// *****************************************************************************
//
// Copyright (C) 2012 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#include <image_util/image_matching.h>

#include <algorithm>
#include <vector>

#include <QPolygonF>
#include <QPointF>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ros/ros.h>

namespace image_util
{
  void GetFundamentalInliers(const cv::Mat points1,
                             const cv::Mat points2,
                             cv::Mat& fundamental_matrix,
                             cv::Mat& inliers1,
                             cv::Mat& inliers2,
                             double max_distance,
                             double confidence)
  {
    std::vector<uint32_t> indices;
    GetFundamentalInliers(
      points1, points2, 
      fundamental_matrix, 
      inliers1, inliers2,
      indices,
      max_distance, 
      confidence);
  }

  void GetFundamentalInliers(const cv::Mat points1,
                             const cv::Mat points2,
                             cv::Mat& fundamental_matrix,
                             cv::Mat& inliers1,
                             cv::Mat& inliers2,
                             std::vector<uint32_t>& indices,
                             double max_distance,
                             double confidence)
  {
    std::vector<uchar> status;
    fundamental_matrix = cv::findFundamentalMat(
      points1,
      points2,
      status,
      CV_FM_RANSAC,
      max_distance,
      confidence);

    int inliers = 0;
    for (unsigned int i = 0; i < status.size(); i++)
    {
      if (status[i])
      {
        inliers++;
      }
    }

    indices.resize(inliers);

    if (inliers > 0)
    {
      inliers1 = cv::Mat(cv::Size(1, inliers), CV_32FC2);
      inliers2 = cv::Mat(cv::Size(1, inliers), CV_32FC2);

      int index = 0;
      for (unsigned int i = 0; i < status.size(); i++)
      {
        if (status[i])
        {
          inliers1.at<cv::Vec2f>(0, index) = points1.at<cv::Vec2f>(0, i);
          inliers2.at<cv::Vec2f>(0, index) = points2.at<cv::Vec2f>(0, i);
          indices[index] = i;
          index++;
        }
      }
    }
  }

  void ConvertMatches(const std::vector<cv::KeyPoint>& kp1,
                      const std::vector<cv::KeyPoint>& kp2,
                      const std::vector<cv::DMatch>& matches,
                      cv::Mat& kp1_out,
                      cv::Mat& kp2_out)
  {
    kp1_out.release();
    kp2_out.release();
    kp1_out.create(cv::Size(1, matches.size()), CV_32FC2);
    kp2_out.create(cv::Size(1, matches.size()), CV_32FC2);

    for (unsigned int i = 0; i < matches.size(); i++)
    {
      kp1_out.at<cv::Vec2f>(0, i) = kp1[matches[i].queryIdx].pt;
      kp2_out.at<cv::Vec2f>(0, i) = kp2[matches[i].trainIdx].pt;
    }

  }
}
