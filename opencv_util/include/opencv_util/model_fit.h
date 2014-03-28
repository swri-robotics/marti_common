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

#ifndef OPENCV_UTIL_MODEL_FIT_H_
#define OPENCV_UTIL_MODEL_FIT_H_

#include <vector>

#include <opencv2/core/core.hpp>

#include <math_util/random.h>

namespace opencv_util
{
  cv::Mat FindRigidTransform2d(
    const cv::Mat& points1, 
    const cv::Mat& points2,
    cv::Mat& inliers1,
    cv::Mat& inliers2,
    std::vector<uint32_t> &good_points,
    double max_error = 1.0,
    double confidence = 0.9,
    int32_t max_iterations = 1000,
    math_util::RandomGeneratorPtr rng = math_util::RandomGeneratorPtr());
}

#endif  // OPENCV_UTIL_MODEL_FIT_H_
