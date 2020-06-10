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

#include <swri_image_util/image_matching.h>

#include <algorithm>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace swri_image_util
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
      cv::FM_RANSAC,
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
