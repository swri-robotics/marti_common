// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#include <opencv_util/show.h>

#include <map>
#include <string>

#include <boost/serialization/singleton.hpp>
#include <boost/thread/mutex.hpp>

#include <opencv2/highgui/highgui.hpp>

namespace opencv_util
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
        cvStartWindowThread();
      }

      if (windows_.count(name) == 0)
      {
        windows_[name] = name;

        cvNamedWindow(name.c_str(), CV_WINDOW_NORMAL);
      }
    }

    friend class boost::serialization::detail::singleton_wrapper<CvWindows>;
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
      }
      else if(mat.type() == CV_32FC3)
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

