// *****************************************************************************
//
// Copyright (C) 2011 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#ifndef TRANSFORM_UTIL_GEOREFERENCE_H_
#define TRANSFORM_UTIL_GEOREFERENCE_H_

#include <string>

#include <yaml-cpp/yaml.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace transform_util
{
  class GeoReference
  {
  public:
    GeoReference(const std::string& path);
    GeoReference(const GeoReference& geo);
    ~GeoReference();

    bool Load();
    void Print();

    std::string GeoPath() const { return path_; }
    std::string Path() const { return image_path_; }
    unsigned int Width() const { return width_; }
    unsigned int Height() const { return height_; }
    unsigned int TileSize() const { return tile_size_; }

    std::string Datum() const { return datum_; }
    std::string Projection() const { return projection_; }

    void GetCoordinate(int x_pixel, int y_pixel, double& x_coordinate, double& y_coordinate) const;
    void GetPixel(double x_coordinate, double y_coordinate, int& x_pixel, int& y_pixel) const;

  private:
    bool loaded_;
  
    // Image properties
    std::string path_;
    std::string image_path_;
    unsigned int width_;
    unsigned int height_;
    unsigned int tile_size_;

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

    void GetTransform();
  };
}

#endif  // TRANSFORM_UTIL_GEOREFERENCE_H_

