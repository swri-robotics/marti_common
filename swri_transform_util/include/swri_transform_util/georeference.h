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

#ifndef TRANSFORM_UTIL_GEOREFERENCE_H_
#define TRANSFORM_UTIL_GEOREFERENCE_H_

#include <string>

#include <yaml-cpp/yaml.h>

#include <rclcpp/logger.hpp>

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

namespace swri_transform_util
{
  class GeoReference
  {
  public:
    explicit GeoReference(const std::string& path,
        rclcpp::Logger logger = rclcpp::get_logger("swri_transform_util::GeoReference"));
    GeoReference(const GeoReference& geo,
                 rclcpp::Logger logger = rclcpp::get_logger("swri_transform_util::GeoReference"));
    ~GeoReference() = default;

    bool Load();
    void Print();

    std::string GeoPath() const { return path_; }
    std::string Path() const { return image_path_; }
    unsigned int Width() const { return width_; }
    unsigned int Height() const { return height_; }
    unsigned int TileSize() const { return tile_size_; }
    std::string Extension() const { return extension_; }

    std::string Datum() const { return datum_; }
    std::string Projection() const { return projection_; }

    void GetCoordinate(int x_pixel, int y_pixel, double& x_coordinate, double& y_coordinate) const;
    void GetPixel(double x_coordinate, double y_coordinate, int& x_pixel, int& y_pixel) const;

  private:
    void GetTransform();

    bool loaded_;

    // Image properties
    std::string path_;
    std::string image_path_;
    unsigned int width_;
    unsigned int height_;
    unsigned int tile_size_;
    std::string extension_;

    // Coordinate system
    std::string datum_;
    std::string projection_;

    // Affine transform from pixel space
    cv::Mat transform_;
    cv::Mat inverse_transform_;

    // Tiepoints
    cv::Mat pixels_;
    cv::Mat coordinates_;

    double x_offset_;
    double y_offset_;

    rclcpp::Logger logger_;
  };
}

#endif  // TRANSFORM_UTIL_GEOREFERENCE_H_

