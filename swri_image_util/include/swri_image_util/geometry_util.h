// *****************************************************************************
//
// Copyright (c) 2012, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef SWRI_IMAGE_UTIL__GEOMETRY_UTIL_H_
#define SWRI_IMAGE_UTIL__GEOMETRY_UTIL_H_

#include <vector>

#include <opencv2/core/core.hpp>

#include <tf2/transform_datatypes.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace swri_image_util
{
typedef cv::Rect_<double> BoundingBox;

/**
   * Calculate the overlapping area of a rectangle an a rigidly transformed
   * version of itself.
   *
   * @param[in]  rect             The rectangle.
   * @param[in]  rigid_transform  The rigid transform.
   *
   * @returns The area of intersection of the two rectangles.
   */
double GetOverlappingArea(
  const cv::Rect & rect,
  const cv::Mat & rigid_transform);

/**
   * Determine if two aligned rectangles intersect one another.
   *
   * @param[in]  box1  The first rectangle.
   * @param[in]  box2  The second rectangle.
   *
   * @returns True if box1 intersects with box2.  False otherwise.
   */
bool Intersects(const BoundingBox & box1, const BoundingBox & box2);

/**
   * Projects a 3D ellipsoid to an ellipse on the XY-plane.
   *
   * @param[in]  ellipsoid  The ellipsoid represented as a 3x3 float matrix.
   *
   * @returns The ellipse as a 2x2 float matrix if successful.  An empty matrix
   *          otherwise.
   */
cv::Mat ProjectEllipsoid(const cv::Mat & ellipsiod);

/**
   * Gets a list of points on the perimeter of an ellipse.
   *
   * The Z coordinate of every returned point is 0, whatever the center's Z is.
   * Use GetEllipsePointsWithZ() to get the perimeter in the plane of the
   * center instead.
   *
   * @param[in]  ellipse     The ellipse represented as a 2x2 float matrix.
   * @param[in]  center      The center of the ellipse.
   * @param[in]  scale       A scale factor.
   * @param[in]  num_points  The number of points to use.
   *
   * @returns A list of points on the perimeter of the ellipse if successful.
   *          An empty list otherwise.
   */
std::vector<tf2::Vector3> GetEllipsePoints(
  const cv::Mat & ellipse,
  const tf2::Vector3 & center,
  double scale,
  int32_t num_points);

/**
   * Gets a list of points on the perimeter of an ellipse, in the plane of the
   * ellipse's center.
   *
   * This is GetEllipsePoints() except that the Z coordinate of each returned
   * point is the center's Z rather than 0.
   *
   * @param[in]  ellipse     The ellipse represented as a 2x2 float matrix.
   * @param[in]  center      The center of the ellipse.
   * @param[in]  scale       A scale factor.
   * @param[in]  num_points  The number of points to use.
   *
   * @returns A list of points on the perimeter of the ellipse if successful.
   *          An empty list otherwise.
   */
std::vector<tf2::Vector3> GetEllipsePointsWithZ(
  const cv::Mat & ellipse,
  const tf2::Vector3 & center,
  double scale,
  int32_t num_points);
}  // namespace swri_image_util

#endif  // SWRI_IMAGE_UTIL__GEOMETRY_UTIL_H_
