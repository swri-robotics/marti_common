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

  bool Valid3dPointCorrespondences(
    const cv::Mat& points1,
    const cv::Mat& points2);

  bool ZipCorrespondences(
    const cv::Mat& points1,
    const cv::Mat& points2,
    cv::Mat& correspondences);

  template <class Model>
  class Fit3d
  {
  public:
    typedef cv::Mat T;         // An Nx3 float matrix
    typedef Model M;           // A geometric model

    explicit Fit3d(const T& data) : data_(data) {}

    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const = 0;
    int32_t GetInlierCount(const M& model, double max_error)
    {
      CalculateNorms(model, norms__);

      cv::compare(norms__, cv::Scalar(max_error), thresholded__, cv::CMP_LT);

      return cv::countNonZero(thresholded__);
    }

    void GetInliers(const M& model, double max_error, std::vector<uint32_t>& indices)
    {
      CalculateNorms(model, norms__);

      indices.clear();
      indices.reserve(norms__.rows);
      double threshold = max_error;
      for (int i = 0; i < norms__.rows; i++)
      {
        if (norms__.at<float>(i) < threshold)
        {
          indices.push_back(i);
        }
      }
    }

    int32_t Size() const { return data_.rows; }
    virtual std::string GetModelString(M& model) const { return ""; }

    static void CopyTo(const M& src, M& dst)
    {
      src.copyTo(dst);
    }

  protected:
    virtual void CalculateNorms(const M& model, cv::Mat& norms) = 0;

    const T& data_;

    // Buffer matrices to avoid repeated memory allocations.
    cv::Mat norms__;
    cv::Mat delta__;
    cv::Mat thresholded__;
  };

  struct PlaneModel
  {
    PlaneModel() : x(0), y(0), z(0), i(0), j(0), k(0) {}
    void copyTo(PlaneModel& dst) const
    {
      dst = *this;
    }

    float x, y, z, i, j, k;
  };

  class PlaneFit : public Fit3d<PlaneModel>
  {
  public:
    enum { MIN_SIZE = 3 };

    PlaneFit(const T& data, float min_angle = 0.2) :
        Fit3d<PlaneModel>(data),
        min_angle_(min_angle) {}
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;
    bool ValidData() const
    {
      return data_.cols == 1 && data_.rows >= MIN_SIZE && data_.type() == CV_32FC3;
    }

  protected:
    virtual void CalculateNorms(const M& model, cv::Mat& norms);

    float min_angle_;
  };

  class PerpendicularPlaneWithPointFit : public PlaneFit
  {
  public:
    enum { MIN_SIZE = 2 };

    PerpendicularPlaneWithPointFit(const T& data,
                                   const cv::Vec3f& point_on_plane = cv::Vec3f(0,0,0),
                                   const cv::Vec3f& perp_axis = cv::Vec3f(0,0,1),
                                   float max_axis_angle = 0.5,
                                   float min_angle = 0.2) :
        PlaneFit(data, min_angle),
        point_(point_on_plane),
        perp_axis_(perp_axis),
        max_axis_angle_(max_axis_angle)
    {}

    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;

  protected:

    cv::Vec3f point_;
    cv::Vec3f perp_axis_;
    float max_axis_angle_;
  };

  struct LineModel3d
  {
    LineModel3d() : x(0), y(0), z(0), i(0), j(0), k(0) {}
    void copyTo(LineModel3d& dst) const
    {
      dst = *this;
    }

    float x, y, z, i, j, k;
  };

  class LineFit3d : public Fit3d<LineModel3d>
  {
  public:
    enum { MIN_SIZE = 2 };

    LineFit3d(const T& data) : Fit3d<LineModel3d>(data) {}
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;
    bool ValidData() const
    {
      return data_.cols == 1 && data_.rows >= MIN_SIZE && data_.type() == CV_32FC3;
    }

  protected:
    virtual void CalculateNorms(const M& model, cv::Mat& norms);

    cv::Mat temp1__;
    cv::Mat temp2__;
    cv::Mat x0_p_dot_n__;
    cv::Mat x0_p__;
  };

  class OrthoLineFit3d : public LineFit3d
  {
  public:
    OrthoLineFit3d(const T& data, const LineModel3d& ortho, float angle_tolerance = 0.09) :
      LineFit3d(data), ortho_(ortho), angle_tolerance_(angle_tolerance) {}
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;

  protected:
    LineModel3d ortho_;
    float angle_tolerance_;
  };

  struct CrossModel3d
  {
    CrossModel3d() : x(0), y(0), z(0), i1(0), j1(0), k1(0), i2(0), j2(0), k2(0) {}
    void copyTo(CrossModel3d& dst) const
    {
      dst = *this;
    }

    float x, y, z, i1, j1, k1, i2, j2, k2;
  };

  class CrossFit3d : public Fit3d<CrossModel3d>
  {
  public:
    enum { MIN_SIZE = 3 };

    CrossFit3d(const T& data, float min_angle = 0.2) :
        Fit3d<CrossModel3d>(data),
        min_angle_(min_angle) {}
    virtual bool GetModel(const std::vector<int32_t>& indices, M& model, double max_error) const;
    bool ValidData() const
    {
      return data_.cols == 1 && data_.rows >= MIN_SIZE && data_.type() == CV_32FC3;
    }

  protected:
    virtual void CalculateNorms(const M& model, cv::Mat& norms);

    float min_angle_;
    cv::Mat temp1__;
    cv::Mat temp2__;
    cv::Mat temp3__;
    cv::Mat x0_p_dot_n__;
    cv::Mat x0_p__;
  };

}

#endif  // OPENCV_UTIL_MODELS_H_
