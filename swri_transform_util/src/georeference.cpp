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
#include <filesystem>
#include <fstream>

#include <opencv2/calib3d.hpp>

// ROS libraries
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

namespace swri_transform_util
{
  GeoReference::GeoReference(const std::string& path,
                             const rclcpp::Logger logger) :
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
      y_offset_(0),
      logger_(logger)
  {
    // Initialize transform to identity
    transform_.at<double>(0, 0) = 1;
    transform_.at<double>(0, 1) = 0;
    transform_.at<double>(0, 2) = 0;
    transform_.at<double>(1, 0) = 0;
    transform_.at<double>(1, 1) = 1;
    transform_.at<double>(1, 2) = 0;
  }

  GeoReference::GeoReference(const GeoReference& geo,
                             const rclcpp::Logger logger) :
      loaded_(geo.loaded_),
      path_(geo.path_),
      image_path_(geo.image_path_),
      width_(geo.width_),
      height_(geo.height_),
      tile_size_(geo.tile_size_),
      extension_(geo.extension_),
      datum_(geo.datum_),
      projection_(geo.projection_),
      transform_(geo.transform_),
      logger_(logger)
  {
  }

  bool GeoReference::Load()
  {
    YAML::Node doc = YAML::LoadFile(path_);
    if (!doc)
    {
      RCLCPP_ERROR(logger_, "Failed to load file: %s", path_.c_str());
      return false;
    }

    try
    {
      if (!doc["image_path"])
      {
        RCLCPP_ERROR(logger_, "Georeference missing image_path.");
        return false;
      }
      image_path_ = doc["image_path"].as<std::string>();

      std::filesystem::path imagePath(image_path_);

      // If we have an absolute path, we don't need to change it
      // Otherwise, we want to create a relative path from the .geo file
      // location
      if (imagePath.is_absolute() == false)
      {
        std::filesystem::path geoPath(path_);
        image_path_ = (geoPath.parent_path() / imagePath.relative_path()).lexically_normal().string();

        RCLCPP_INFO(logger_, "georeference: Image path is %s", image_path_.c_str());
      }

      if (!doc["image_width"])
      {
        RCLCPP_ERROR(logger_, "Georeference missing image_width.");
        return false;
      }
      width_ = doc["image_width"].as<uint32_t>();

      if (!doc["image_height"])
      {
        RCLCPP_ERROR(logger_, "Georeference missing image_height.");
        return false;
      }
      height_ = doc["image_height"].as<uint32_t>();

      if (!doc["tile_size"])
      {
        RCLCPP_ERROR(logger_, "Georeference missing tile_size.");
        return false;
      }
      tile_size_ = doc["tile_size"].as<uint32_t>();

      if (doc["extension"])
      {
        extension_ = doc["extension"].as<std::string>();
      }

      if (!doc["datum"])
      {
        RCLCPP_ERROR(logger_, "Georeference missing datum.");
        return false;
      }
      datum_ = doc["datum"].as<std::string>();

      if (!doc["projection"])
      {
        RCLCPP_ERROR(logger_, "Georeference missing projection.");
        return false;
      }
      projection_ = doc["projection"].as<std::string>();

      // Parse in the tiepoints
      if (!doc["tiepoints"])
      {
        RCLCPP_ERROR(logger_, "Georeference missing tiepoints.");
        return false;
      }
      pixels_ = cv::Mat(1, doc["tiepoints"].size(), CV_32SC2);
      coordinates_ = cv::Mat(1, doc["tiepoints"].size(), CV_64FC2);
      RCLCPP_INFO(logger_, "georeference: Found %d tiepoints", (int32_t) (doc["tiepoints"].size()));
      for (size_t i = 0; i < doc["tiepoints"].size(); i++)
      {
        if (!doc["tiepoints"][i]["point"])
        {
          RCLCPP_ERROR(logger_, "Georeference tiepoint %zu missing point.", i);
          return false;
        }

        if (doc["tiepoints"][i]["point"].size() != 4)
        {
          RCLCPP_ERROR(logger_, "Georeference tiepoint %zu size != 4.", i);
          return false;
        }

        // Parse pixel column value into the pixel list
        pixels_.at<cv::Vec2i>(0, i)[0] = doc["tiepoints"][i]["point"][0].as<int32_t>();

        // Parse pixel row value into the pixel list
        pixels_.at<cv::Vec2i>(0, i)[1] = doc["tiepoints"][i]["point"][1].as<int32_t>();

        // Parse the x coordinate into the coordinate list
        coordinates_.at<cv::Vec2d>(0, i)[0] = doc["tiepoints"][i]["point"][2].as<double>();

        // Parse the y coordinate into the coordinate list
        coordinates_.at<cv::Vec2d>(0, i)[1] = doc["tiepoints"][i]["point"][3].as<double>();
      }

      if (doc["tiepoints"].size() > 2)
      {
        GetTransform();
        if (transform_.empty())
        {
          RCLCPP_ERROR(logger_, "Failed to calculate georeference transform.");
          return false;
        }
      }
      else if (doc["tiepoints"].size() == 1)
      {
        // Parse in the X scale
        transform_.at<float>(0, 0) = doc["pixel_scale"][0].as<float>();

        // Parse in the Y scale
        transform_.at<float>(1, 1) = doc["pixel_scale"][1].as<float>();

        transform_.at<float>(0, 2) = coordinates_.at<cv::Vec2d>(0, 1)[0] -
                                     pixels_.at<cv::Vec2i>(0, 1)[0] * transform_.at<double>(0, 0);

        transform_.at<float>(1, 2) = coordinates_.at<cv::Vec2d>(0, 1)[1] -
                                     pixels_.at<cv::Vec2i>(0, 1)[1] * transform_.at<double>(1, 1);
      }
      else
      {
        RCLCPP_ERROR(logger_, "georeference: At least 3 tiepoints required.");
        return false;
      }

      Print();
    }
    catch (const YAML::ParserException& e)
    {
      RCLCPP_ERROR(logger_, "%s", e.what());
      return false;
    }
    catch (const YAML::Exception& e)
    {
      RCLCPP_ERROR(logger_, "%s", e.what());
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

    transform_ = cv::estimateAffine2D(src, dst);
    inverse_transform_ = cv::estimateAffine2D(dst, src);
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
    RCLCPP_INFO(logger_, "georeference:  path = %s", path_.c_str());
    RCLCPP_INFO(logger_, "georeference:  image = %s", image_path_.c_str());
    RCLCPP_INFO(logger_, "georeference:  width = %d", width_);
    RCLCPP_INFO(logger_, "georeference:  height = %d", height_);
    RCLCPP_INFO(logger_, "georeference:  tile_size = %d", tile_size_);
    RCLCPP_INFO(logger_, "georeference:  extension = %s", extension_.c_str());
    RCLCPP_INFO(logger_, "georeference:  datum = %s", datum_.c_str());
    RCLCPP_INFO(logger_, "georeference:  projection = %s", projection_.c_str());

    RCLCPP_INFO(logger_, "georeference:  tiepoints");
    for (int i = 0; i < pixels_.cols; i++)
    {
      RCLCPP_INFO(logger_, "georeference:     [%d, %d, %lf, %lf]",
                  pixels_.at<cv::Vec2i>(0, i)[0],
                  pixels_.at<cv::Vec2i>(0, i)[1],
                  coordinates_.at<cv::Vec2d>(0, i)[0],
                  coordinates_.at<cv::Vec2d>(0, i)[1]);
    }

    RCLCPP_INFO(logger_, "georeference:  transform: %8lf, %8lf, %8lf",
                transform_.at<double>(0, 0),
                transform_.at<double>(0, 1),
                transform_.at<double>(0, 2) + x_offset_);

    RCLCPP_INFO(logger_, "georeference:             %8lf, %8lf, %8lf",
                transform_.at<double>(1, 0),
                transform_.at<double>(1, 1),
                transform_.at<double>(1, 2) + y_offset_);

    RCLCPP_INFO(logger_, "georeference:             %8lf, %8lf, %8lf", 0.0, 0.0, 1.0);
  }
}
