// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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

#include <geometry_util/cubic_spline.h>

#include <lapackpp.h>

namespace geometry_util
{

  bool CubicSplineInterpolation(
    const std::vector<cv::Vec2d>& points,
    double delta,
    std::vector<std::vector<cv::Vec2d> >& splines)
  {
    if (delta <= 0)
    {
      return false;
    }

    size_t num_points = points.size();

    splines.clear();
    splines.resize(num_points - 1);

    // Accumulated distance along linear path.
    std::vector<double> s = std::vector<double>(num_points);

    // Distance between consecutive points on path.
    std::vector<double> ds = std::vector<double>(num_points);

    s[0] = 0;
    for (size_t i = 1; i < num_points; i++)
    {
      double dx = points[i][0] - points[i - 1][0];
      double dy = points[i][1] - points[i - 1][1];

      ds[i - 1] = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      s[i] = s[i - 1] + ds[i - 1];
    }

    // Normalize s to be [0:1]
    double totalDistance = s[num_points - 1];
    for (size_t i = 0; i < num_points; i++)
    {
      s[i] /= totalDistance;
    }

    LaGenMatDouble u(num_points, 2);
    LaGenMatDouble A(num_points, num_points);
    LaGenMatDouble z(num_points, 2);

    // Set up the z0 equation.
    u(0, 0) = 0;
    u(0, 1) = 0;
    A(0, 0) = 1;
    for (size_t i = 1; i < num_points; i++)
    {
      A(0, i) = 0;
    }

    // Set up the z1 to zn-1 equations.
    for (size_t i = 1; i < num_points - 1; i++)
    {
      double hi = s[i + 1] - s[i];
      double himl = s[i] - s[i - 1];

      u(i, 0) = 6.0 * ((points[i + 1][0] - points[i][0]) / hi - (points[i][0]
          - points[i - 1][0]) / himl);
      u(i, 1) = 6.0 * ((points[i + 1][1] - points[i][1]) / hi - (points[i][1]
          - points[i - 1][1]) / himl);

      for (size_t j = 0; j < i - 1; j++)
      {
        A(i, j) = 0;
      }
      A(i, i - 1) = himl;
      A(i, i) = 2 * (himl + hi);
      A(i, i + 1) = hi;
      for (int32_t j = i + 2; j < num_points; j++)
      {
        A(i, j) = 0;
      }
    }

    // Set up the zn equation
    u(num_points - 1, 0) = 0;
    u(num_points - 1, 1) = 0;
    for (size_t j = 0; j < num_points - 1; j++)
    {
      A(num_points - 1, j) = 0;
    }
    A(num_points - 1, num_points - 1) = 1;

    // Solve Az = u
    LaLinearSolve(A, z, u);

    for (size_t i = 0; i < num_points - 1; i++)
    {
      std::vector<cv::Vec2d>& spline = splines[i];

      spline.push_back(points[i]);

      double hi = s[i + 1] - s[i];
      double act_dist = ds[i];
      int32_t num_to_add = static_cast<int32_t> (act_dist / delta);
      double normalizedDelta = hi / (num_to_add + 1);

      // Add the additional points between waypoints
      for (int32_t j = 1; j <= num_to_add; j++)
      {
        // Calculate the (x,y) value of the interpolated point
        double si = s[i] + j * normalizedDelta;
        double xx = (z(i + 1, 0) * pow(si - s[i], 3) + z(i, 0) * pow(s[i + 1]
            - si, 3)) / (6.0 * hi) + (points[i + 1][0] / hi - hi * z(i + 1, 0)
            / 6.0) * (si - s[i]) + (points[i][0] / hi - hi * z(i, 0) / 6.0)
            * (s[i + 1] - si);
        double yy = (z(i + 1, 1) * pow(si - s[i], 3) + z(i, 1) * pow(s[i + 1]
            - si, 3)) / (6.0 * hi) + (points[i + 1][1] / hi - hi * z(i + 1, 1)
            / 6.0) * (si - s[i]) + (points[i][1] / hi - hi * z(i, 1) / 6.0)
            * (s[i + 1] - si);

        // Add the point to the interpolated array
        spline.push_back(cv::Vec2d(xx, yy));
      }

      spline.push_back(points[i + 1]);
    }

    return true;
  }
}
