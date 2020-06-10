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

#include <swri_opencv_util/show.h>

#include <map>
#include <string>

#include <boost/serialization/singleton.hpp>
#include <boost/thread/mutex.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace swri_opencv_util
{
  class CvWindows
  {
  public:
    ~CvWindows() {}

    void RegisterWindow(const std::string& name)
    {
      boost::unique_lock<boost::mutex> lock(mutex_);

      if (windows_.empty())
      {
        cv::startWindowThread();
      }

      if (windows_.count(name) == 0)
      {
        windows_[name] = name;

        cv::namedWindow(name.c_str(), cv::WINDOW_NORMAL);
      }
    }

#if (BOOST_VERSION / 100 % 1000) >= 65 && (BOOST_VERSION / 100 % 1000) < 69
    friend class boost::serialization::singleton<CvWindows>;
#else
    friend class boost::serialization::detail::singleton_wrapper<CvWindows>;
#endif
  private:
    CvWindows() {}
    boost::mutex mutex_;
    std::map<std::string, std::string> windows_;
  };
  typedef boost::serialization::singleton<CvWindows> CvWindowsSingleton;

  void ShowScaled(
      const std::string& name,
      const cv::Mat& mat,
      const cv::Mat& mask,
      double a,
      double b)
  {
    if (mat.empty())
    {
      return;
    }

    CvWindowsSingleton::get_mutable_instance().RegisterWindow(name);

    cv::Mat scaled;

    // Autoscale if a is negative
    if(a < 0.0)
    {
      double min, max;
      cv::minMaxLoc(mat, &min, &max, 0, 0, mask);

      if(mat.type() == CV_8UC1)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8U, a, b);
      }
      else if(mat.type() == CV_32FC1)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8U, a, b);
        if (!mask.empty())
        {
          cv::Mat color;
          cv::cvtColor(scaled, color, cv::COLOR_GRAY2BGR);
          color.setTo(cv::Scalar(0.0,0.0,255.0), mask == 0);
          scaled = color;
        }
      }
      else if(mat.type() == CV_32FC3)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8UC3, a, b);
      }
      else if(mat.type() == CV_8UC3)
      {
        a = 255.0 / std::max(max - min, DBL_EPSILON);
        b = -min * a;
        mat.convertTo(scaled, CV_8UC3, a, b);
      }
    }
    else
    {
      mat.convertTo(scaled, CV_8U, a, b);
    }

    cv::imshow(name, scaled);
  }
}

