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

#ifndef OPENCV_UTIL_MODELS_H_
#define OPENCV_UTIL_MODELS_H_

#include <vector>

#include <opencv2/core/core.hpp>

namespace swri_opencv_util
{

  class Correspondence2d
  {
  public:
    typedef cv::Mat T;         // An Nx4 float matrix
    typedef cv::Mat M;
    
    explicit Correspondence2d(const T& data) : data_(data) {}
    
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const = 0;
    int32_t GetInlierCount(const M& model, double max_error);
    void GetInliers(const M& model, double max_error, std::vector<uint32_t>& indices);
    int32_t Size() const { return data_.rows; }
    virtual std::string GetModelString(M& model) const { return ""; }
    
    static void CopyTo(const M& src, M& dst)
    {
      src.copyTo(dst);
    }
    
  protected:
    virtual void CalculateNorms(const M& model, cv::Mat& norms);
    
    const T& data_;
    
    // Buffer matrices to avoid repeated memory allocations.
    cv::Mat norms__;
    cv::Mat predicted__;
    cv::Mat delta__;
    cv::Mat delta_squared__;
    cv::Mat thresholded__;
  };

  class Homography : public Correspondence2d
  {
  public:
    enum { MIN_SIZE = 4 };
    
    explicit Homography(const T& data) : Correspondence2d(data) {}
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;
    bool ValidData() const 
    { 
      return data_.cols == 4 && data_.rows >= MIN_SIZE && data_.type() == CV_32F; 
    }
    
  protected:
    virtual void CalculateNorms(const M& model, cv::Mat& norms);
  };

  class AffineTransform2d : public Correspondence2d
  {
  public:
    enum { MIN_SIZE = 3 };
    
    explicit AffineTransform2d(const T& data) : Correspondence2d(data) {}
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;
    bool ValidData() const 
    { 
      return data_.cols == 4 && data_.rows >= MIN_SIZE && data_.type() == CV_32F; 
    }
  };

  class RigidTransform2d : public Correspondence2d
  {
  public:
    enum { MIN_SIZE = 2 };
    
    explicit RigidTransform2d(const T& data) : Correspondence2d(data) {}
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;
    bool ValidData() const 
    { 
      return data_.cols == 4 && data_.rows >= MIN_SIZE && data_.type() == CV_32F; 
    }
  };
  
  class Translation2d : public Correspondence2d
  {
  public:
    enum { MIN_SIZE = 1 };
    
    explicit Translation2d(const T& data) : Correspondence2d(data) {}
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;
    bool ValidData() const 
    { 
      return data_.cols == 4 && data_.rows >= MIN_SIZE && data_.type() == CV_32F; 
    }
  };
  
  bool Valid2dPointCorrespondences(
    const cv::Mat& points1, 
    const cv::Mat& points2);
  
  bool ZipCorrespondences(
    const cv::Mat& points1,
    const cv::Mat& points2,
    cv::Mat& correspondences);
}

#endif  // OPENCV_UTIL_MODELS_H_
