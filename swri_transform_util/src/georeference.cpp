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

#include <swri_transform_util/georeference.h>

// C++ standard libraries
#include <fstream>

#include <boost/filesystem.hpp>

// ROS libraries
#include <ros/ros.h>
#include <swri_yaml_util/yaml_util.h>

namespace swri_transform_util
{
  GeoReference::GeoReference(const std::string& path) :
    loaded_(false),
    path_(path),
    image_path_(""),
    width_(0),
    height_(0),
    tile_size_(0),
    extension_("jpg"),
    datum_(""),
    projection_(""),
    transform_(2, 3, CV_64F),
    pixels_(1, 1, CV_32SC2),
    coordinates_(1, 1, CV_64FC2),
    x_offset_(0),
    y_offset_(0)
  {
    // Initialize transform to identity
    transform_.at<double>(0, 0) = 1;
    transform_.at<double>(0, 1) = 0;
    transform_.at<double>(0, 2) = 0;
    transform_.at<double>(1, 0) = 0;
    transform_.at<double>(1, 1) = 1;
    transform_.at<double>(1, 2) = 0;
  }

  GeoReference::GeoReference(const GeoReference& geo) :
    loaded_(geo.loaded_),
    path_(geo.path_),
    image_path_(geo.image_path_),
    width_(geo.width_),
    height_(geo.height_),
    tile_size_(geo.tile_size_),
    extension_(geo.extension_),
    datum_(geo.datum_),
    projection_(geo.projection_),
    transform_(geo.transform_)
  {
  }

  GeoReference::~GeoReference()
  {
  }

  bool GeoReference::Load()
  {
    YAML::Node doc;
    if (!swri_yaml_util::LoadFile(path_, doc))
    {
      ROS_ERROR("Failed to load file: %s", path_.c_str());
      return false;
    }

    try
    {
      if (!swri_yaml_util::FindValue(doc, "image_path"))
      {
        ROS_ERROR("Georeference missing image_path.");
        return false;
      }
      doc["image_path"] >> image_path_;

      boost::filesystem::path imagePath(image_path_);

      // If we have an absolute path, we don't need to change it
      // Otherwise, we want to create a relative path from the .geo file
      // location
      if (imagePath.is_complete() == false)
      {
        boost::filesystem::path geoPath(path_);
        image_path_ = (geoPath.parent_path() / imagePath.relative_path()).normalize().string();

        ROS_INFO("georeference: Image path is %s", image_path_.c_str());
      }

      if (!swri_yaml_util::FindValue(doc, "image_width"))
      {
        ROS_ERROR("Georeference missing image_width.");
        return false;
      }
      doc["image_width"] >> width_;

      if (!swri_yaml_util::FindValue(doc, "image_height"))
      {
        ROS_ERROR("Georeference missing image_height.");
        return false;
      }
      doc["image_height"] >> height_;
      
      if (!swri_yaml_util::FindValue(doc, "tile_size"))
      {
        ROS_ERROR("Georeference missing tile_size.");
        return false;
      }
      doc["tile_size"] >> tile_size_;

      if (swri_yaml_util::FindValue(doc, "extension"))
      {
          doc["extension"] >> extension_;
      }

      if (!swri_yaml_util::FindValue(doc, "datum"))
      {
        ROS_ERROR("Georeference missing datum.");
        return false;
      }
      doc["datum"] >> datum_;

      if (!swri_yaml_util::FindValue(doc, "projection"))
      {
        ROS_ERROR("Georeference missing projection.");
        return false;
      }
      doc["projection"] >> projection_;

      // Parse in the tiepoints
      if (!swri_yaml_util::FindValue(doc, "tiepoints"))
      {
        ROS_ERROR("Georeference missing tiepoints.");
        return false;
      }
      pixels_ = cv::Mat(1, doc["tiepoints"].size(), CV_32SC2);
      coordinates_ = cv::Mat(1, doc["tiepoints"].size(), CV_64FC2);
      ROS_INFO("georeference: Found %d tiepoints", (int32_t)(doc["tiepoints"].size()));
      for (size_t i = 0; i < doc["tiepoints"].size(); i++)
      {
		if (!swri_yaml_util::FindValue(doc["tiepoints"][i], "point"))
		{
		  ROS_ERROR("Georeference tiepoint %zu missing point.", i);
		  return false;
        }

        if (doc["tiepoints"][i]["point"].size() != 4)
        {
		  ROS_ERROR("Georeference tiepoint %zu size != 4.", i);
		  return false;
        }

        // Parse pixel column value into the pixel list
        doc["tiepoints"][i]["point"][0] >> pixels_.at<cv::Vec2i>(0, i)[0];

        // Parse pixel row value into the pixel list
        doc["tiepoints"][i]["point"][1] >> pixels_.at<cv::Vec2i>(0, i)[1];

        // Parse the x coordinate into the coordinate list
        doc["tiepoints"][i]["point"][2] >> coordinates_.at<cv::Vec2d>(0, i)[0];

        // Parse the y coordinate into the coordinate list
        doc["tiepoints"][i]["point"][3] >> coordinates_.at<cv::Vec2d>(0, i)[1];
      }

      if (doc["tiepoints"].size() > 2)
      {
        GetTransform();
        if (transform_.empty())
        {
          ROS_ERROR("Failed to calculate georeference transform.");
          return false;
        }
      }
      else if (doc["tiepoints"].size() == 1)
      {
        // Parse in the X scale
        doc["pixel_scale"][0] >> transform_.at<float>(0, 0);

        // Parse in the Y scale
        doc["pixel_scale"][1] >> transform_.at<float>(1, 1);

        transform_.at<float>(0, 2) = coordinates_.at<cv::Vec2d>(0, 1)[0] -
            pixels_.at<cv::Vec2i>(0, 1)[0] * transform_.at<double>(0, 0);

        transform_.at<float>(1, 2) = coordinates_.at<cv::Vec2d>(0, 1)[1] -
            pixels_.at<cv::Vec2i>(0, 1)[1] * transform_.at<double>(1, 1);
      }
      else
      {
        ROS_ERROR("georeference: At least 3 tiepoints required.");
        return false;
      }

      Print();
    }
    catch (const YAML::ParserException& e)
    {
      ROS_ERROR("%s", e.what());
      return false;
    }
    catch (const YAML::Exception& e)
    {
      ROS_ERROR("%s", e.what());
      return false;
    }

    loaded_ = true;
    return true;
  }

  void GeoReference::GetTransform()
  {
    // Copy pixels into float 32 matrix
    cv::Mat src(1, pixels_.cols, CV_32FC2);
    for (int i = 0; i < pixels_.cols; i++)
    {
      src.at<cv::Vec2f>(0, i)[0] = static_cast<float>(pixels_.at<cv::Vec2i>(0, i)[0]);
      src.at<cv::Vec2f>(0, i)[1] = static_cast<float>(pixels_.at<cv::Vec2i>(0, i)[1]);
    }

    // Offset coordinates to a new origin to avoid loss of precision
    x_offset_ = coordinates_.at<cv::Vec2d>(0, 0)[0];
    y_offset_ = coordinates_.at<cv::Vec2d>(0, 0)[1];

    // Copy coordinates into float 32 matrix
    cv::Mat dst(1, pixels_.cols, CV_32FC2);
    for (int i = 0; i < pixels_.cols; i++)
    {
      dst.at<cv::Vec2f>(0, i)[0] = coordinates_.at<cv::Vec2d>(0, i)[0] - x_offset_;
      dst.at<cv::Vec2f>(0, i)[1] = coordinates_.at<cv::Vec2d>(0, i)[1] - y_offset_;
    }

    transform_ = cv::estimateRigidTransform(src, dst, true);
    inverse_transform_ = cv::estimateRigidTransform(dst, src, true);
  }

  void GeoReference::GetCoordinate(
    int x_pixel, int y_pixel,
    double& x_coordinate, double& y_coordinate) const
  {
    cv::Mat src(1, 1, CV_32FC2);
    cv::Mat dst(1, 1, CV_32FC2);

    src.at<cv::Vec2f>(0, 0)[0] = x_pixel;
    src.at<cv::Vec2f>(0, 0)[1] = y_pixel;

    cv::transform(src, dst, transform_);

    x_coordinate = dst.at<cv::Vec2f>(0, 0)[0] + x_offset_;
    y_coordinate = dst.at<cv::Vec2f>(0, 0)[1] + y_offset_;
  }

  void GeoReference::GetPixel(
    double x_coordinate, double y_coordinate,
    int& x_pixel, int& y_pixel) const
  {
    cv::Mat src(1, 1, CV_32FC2);
    cv::Mat dst(1, 1, CV_32FC2);

    src.at<cv::Vec2f>(0, 0)[0] = x_coordinate - x_offset_;
    src.at<cv::Vec2f>(0, 0)[1] = y_coordinate - y_offset_;

    cv::transform(src, dst, inverse_transform_);

    x_pixel = static_cast<int>(dst.at<cv::Vec2f>(0, 0)[0]);
    y_pixel = static_cast<int>(dst.at<cv::Vec2f>(0, 0)[1]);
  }

  void GeoReference::Print()
  {
    ROS_INFO("georeference:  path = %s", path_.c_str());
    ROS_INFO("georeference:  image = %s", image_path_.c_str());
    ROS_INFO("georeference:  width = %d", width_);
    ROS_INFO("georeference:  height = %d", height_);
    ROS_INFO("georeference:  tile_size = %d", tile_size_);
    ROS_INFO("georeference:  extension = %s", extension_.c_str());
    ROS_INFO("georeference:  datum = %s", datum_.c_str());
    ROS_INFO("georeference:  projection = %s", projection_.c_str());

    ROS_INFO("georeference:  tiepoints");
    for (int i = 0; i < pixels_.cols; i++)
    {
      ROS_INFO("georeference:     [%d, %d, %lf, %lf]",
          pixels_.at<cv::Vec2i>(0, i)[0],
          pixels_.at<cv::Vec2i>(0, i)[1],
          coordinates_.at<cv::Vec2d>(0, i)[0],
          coordinates_.at<cv::Vec2d>(0, i)[1]);
    }

    ROS_INFO("georeference:  transform: %8lf, %8lf, %8lf",
        transform_.at<double>(0, 0),
        transform_.at<double>(0, 1),
        transform_.at<double>(0, 2) + x_offset_);

    ROS_INFO("georeference:             %8lf, %8lf, %8lf",
        transform_.at<double>(1, 0),
        transform_.at<double>(1, 1),
        transform_.at<double>(1, 2) + y_offset_);

    ROS_INFO("georeference:             %8lf, %8lf, %8lf", 0.0, 0.0, 1.0);
  }
}

