// *****************************************************************************
//
// Copyright (c) 2016, Southwest Research Institute® (SwRI®)
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
#include <swri_route_util/util.h>
#include <swri_route_util/route.h>
#include <swri_route_util/route_point.h>

#include <swri_transform_util/frames.h>
#include <swri_transform_util/transform_util.h>

namespace mnm = marti_nav_msgs;
namespace stu = swri_transform_util;

namespace swri_route_util
{
void transform(Route &route,
               const stu::Transform &transform,
               const std::string &target_frame)
{
  for (auto &point : route.points) {
    point.setPosition(transform*point.position());
    point.setOrientation(transform*point.orientation());
  }
  route.header.frame_id = target_frame;
}

void projectToXY(Route &route)
{
  for (auto &point : route.points) {
    point.position().setZ(0.0);
    // todo(exjohnson): if orientation is valid, project it to a
    // rotation around the Z axis only.
  }
}

void fillOrientations(Route &route,
    const tf2::Vector3 &up,
    rclcpp::Logger logger)
{
  // We can't estimate any orientations for 0 or 1 points.
  if (route.points.size() < 2) {
    return;
  }

  std::vector<size_t> degenerate_orientations;

  for (size_t i = 0; i < route.points.size(); ++i) {
    // We're going to estimate the orientation by the center difference using
    // the vector from the previous point to the next point in the route. This
    // assumes that the points are evenly spaced, but it is a reasonable
    // estimate even if they are not.
    tf2::Vector3 v_forward;
    if (i == 0) {
      // For the first point, we use the forward difference
      v_forward = route.points[i+1].position() - route.points[i+0].position();
    } else if (i+1 == route.points.size()) {
      // For the last point, we use the backward difference
      v_forward = route.points[i+0].position() - route.points[i-1].position();
    } else {
      v_forward = route.points[i+1].position() - route.points[i-1].position();
    }

    v_forward.normalize();

    // Y = Z x X
    tf2::Vector3 v_left = up.cross(v_forward);
    // Since Z and X are not necessarily orthogonal, we need to normalize this
    // to get a unit vector.  This is where we'll have problems if our
    // v_forward happens to be really closely aligned with the up
    // axis.  We ignore that.
    v_left.normalize();

    if (std::isnan(v_left.x()))
    {
      // This should catch issues with repeated route points, and co-linear
      // v_forward and up vectors (though the latter should never happen)
      degenerate_orientations.push_back(i);
    }


    // We now have left unit vector that is perpendicular to the
    // forward unit vector, so we can find our actual up vector, which
    // should be in the plane spanned by v_forward and the user
    // provided up direction.
    tf2::Vector3 v_up = v_forward.cross(v_left);
    // We shouldn't need to normalize v_up, but it's good practice
    // since I don't know if the Matrix3x3 handles errors well.
    v_up.normalize();

    // Don't understand why Matrix3x3 doesn't have a constructor for 3
    // vectors.
    tf2::Matrix3x3 rotation(
      v_forward.x(), v_left.x(), v_up.x(),
      v_forward.y(), v_left.y(), v_up.y(),
      v_forward.z(), v_left.z(), v_up.z());

    // Finally we can extract the orientation as a quaternion from the
    // matrix.
    tf2::Quaternion orientation;
    rotation.getRotation(orientation);
    route.points[i].setOrientation(orientation);

    // There is probably a simpler, more elegant way to do this.
  }

  // If there are any degenerate orientations, assign the same orientation as
  // the nearest neighbor. There is probably a better way to do this.
  for (size_t i = 0; i < degenerate_orientations.size(); ++i)
  {
    size_t d_idx = degenerate_orientations[i];
    bool repaired_orientation = false;
    for (size_t j = 1; j < route.points.size(); ++j)
    {
      size_t up_idx = d_idx + j;
      int64_t down_idx = (int64_t)d_idx - (int64_t)j;
      if (up_idx < route.points.size())
      {
        if (std::find(degenerate_orientations.begin(), degenerate_orientations.end(), up_idx) == degenerate_orientations.end())
        {
          // Found a neighboring point with valid orientation.
          route.points[d_idx].setOrientation(route.points[up_idx].orientation());
          repaired_orientation = true;
          break;
        }
      }

      if (down_idx >= 0)
      {
        if (std::find(degenerate_orientations.begin(), degenerate_orientations.end(), down_idx) == degenerate_orientations.end())
        {
          // Found a neighboring point with valid orientation.
          route.points[d_idx].setOrientation(route.points[down_idx].orientation());
          repaired_orientation = true;
          break;
        }
      }
    }

    if (!repaired_orientation)
    {
      RCLCPP_ERROR(logger, "fillOrientations was unable to repair an invalid "
                   "orientation. The route may be malformed.");
    }
    else
    {
      RCLCPP_WARN(logger, "fillOrientations found and repaired an invalid "
                  "orientation. Note that the source route may contain "
                  "repeated points.");
    }
  }
}

// This is a private helper function that finds the nearest distance
// between a point p and the line segment defined by p0 ad p1.  It
// also returns the distance along the segment to the point that is
// nearest to p.  If extrapolate_start/extrapolate_end are true, the
// calculation considers the line segment to extend infinitely in the
// corresponding directions.
static
void nearestDistanceToLineSegment(
  double &min_distance_from_line,
  double &min_distance_on_line,
  const tf2::Vector3 &p0,
  const tf2::Vector3 &p1,
  const tf2::Vector3 &p,
  bool extrapolate_start,
  bool extrapolate_end)
{
  tf2::Vector3 v = p1 - p0;
  const double v_len_sq = v.dot(v);

  // s will be the normalized distance along v that is closest to the
  // desired point.
  double s = 0.0;
  if (v_len_sq > 1e-6) {
    s = v.dot(p - p0) / v_len_sq;
  } else {
    // The two points are too close to define a reasonable line
    // segment, so just pick p1 as the closest point.
    s = 1.0;
  }

  // If we don't allow extrapolation and the nearest point is beyond
  // the line segment boundaries, we need to clamp to the boundary.
  if (!extrapolate_start && s < 0.0) {
    s = 0.0;
  } else if (!extrapolate_end && s > 1.0) {
    s = 1.0;
  }

  tf2::Vector3 x_nearest = p0 + s*v;

  min_distance_from_line = x_nearest.distance(p);
  min_distance_on_line = s*std::sqrt(v_len_sq);
}

bool projectOntoRoute(mnm::msg::RoutePosition &position,
                      const Route &route,
                      const tf2::Vector3 &point,
                      bool extrapolate_before_start,
                      bool extrapolate_past_end)
{
  if (route.points.size() == 0) {
    // We can't do anything with this.
    return false;
  }

  if (route.points.size() == 1) {
    // We can't do much with this.
    position.route_id = route.guid();
    position.id = route.points[0].id();
    position.distance = 0.0;
    return true;
  }

  // First we find the nearest point on the route, without allowing
  // extrapolation.
  double min_distance_from_line = std::numeric_limits<double>::infinity();
  double min_distance_on_line = std::numeric_limits<double>::infinity();
  size_t min_segment_index = 0;

  for (size_t i = 0; i+1 < route.points.size(); ++i) {
    double distance_from_line;
    double distance_on_line;

    nearestDistanceToLineSegment(distance_from_line,
                                 distance_on_line,
                                 route.points[i+0].position(),
                                 route.points[i+1].position(),
                                 point,
                                 false, false);

    if (distance_from_line <= min_distance_from_line) {
      min_segment_index = i;
      min_distance_on_line = distance_on_line;
      min_distance_from_line = distance_from_line;
    }
  }

  // If the nearest segment is the first or last segment, we redo
  // the search while allowing the segment to be extrapolated
  // backward or forward, respectively.  This allows graceful
  // operation if the vehicle is past the boundary of the route.

  if (extrapolate_before_start && min_segment_index == 0) {
    size_t i = 0;
    nearestDistanceToLineSegment(min_distance_from_line,
                                 min_distance_on_line,
                                 route.points[i+0].position(),
                                 route.points[i+1].position(),
                                 point,
                                 true, false);
  } else if (min_segment_index + 2 == route.points.size()) {
    // The end of the route is a special case.  If we go past the end,
    // we want to return a position with the id of the last point and
    // the distance past it.  This annoying complicates things in a
    // number of places, but makes it easy to check if a point is past
    // the end of a route.
    size_t i = min_segment_index;
    nearestDistanceToLineSegment(min_distance_from_line,
                                 min_distance_on_line,
                                 route.points[i+0].position(),
                                 route.points[i+1].position(),
                                 point,
                                 false, true);

    double last_length = (route.points[i+1].position() - route.points[i+0].position()).length();
    if (min_distance_on_line > last_length) {
      min_segment_index++;
      min_distance_on_line -= last_length;
    }

    if (!extrapolate_past_end) {
      min_distance_on_line = 0.0;
    }
  }

  position.route_id = route.guid();
  position.id = route.points[min_segment_index].id();
  position.distance = min_distance_on_line;
  return true;
}

bool projectOntoRouteWindow(
  mnm::msg::RoutePosition &position,
  const Route &route,
  const tf2::Vector3 &point,
  const mnm::msg::RoutePosition &window_start,
  const mnm::msg::RoutePosition &window_end)
{
  if (route.points.size() < 2) {
    // We can't do anything with this.
    return false;
  }

  // First we normalize the window boundaries.
  mnm::msg::RoutePosition start;
  if (!normalizeRoutePosition(start, route, window_start)) {
    return false;
  }
  mnm::msg::RoutePosition end;
  if (!normalizeRoutePosition(end, route, window_end)) {
    return false;
  }

  // Handle the special case where the start and end points are
  // identical.
  if (start.id == end.id && start.distance == end.distance) {
    position.route_id = route.guid();
    position = start;
    return true;
  }

  // Find the indices of the start and end points.  Since we have
  // normalized positions, we know they exist in the route.
  size_t start_index;
  route.findPointId(start_index, start.id);
  size_t end_index;
  route.findPointId(end_index, end.id);

  // Fix the ordering so that start comes before end.
  if ((end_index < start_index) ||
      (start_index == end_index && end.distance < start.distance)) {
    std::swap(end, start);
    std::swap(start_index, end_index);
  }

  // If either of the points are past the end of the route, we want to
  // back them up to the previous segment to reduce the number of
  // special cases we have to handle.
  if (start_index+1 == route.points.size()) {
    start_index -= 1;
    start.id = route.points[start_index].id();
    start.distance += (route.points[start_index+1].position() -
                       route.points[start_index+0].position()).length();
  }
  if (end_index+1 == route.points.size()) {
    end_index -= 1;
    end.id = route.points[end_index].id();
    end.distance += (route.points[end_index+1].position() -
                     route.points[end_index+0].position()).length();
  }

  // Although it causes a little duplication, it's easier over all to
  // explicitly handle the special case where the window is over a
  // single segment.
  if (start_index == end_index) {
    double distance_from_line;
    double distance_on_line;

    nearestDistanceToLineSegment(distance_from_line,
                                 distance_on_line,
                                 route.points[start_index+0].position(),
                                 route.points[start_index+1].position(),
                                 point,
                                 true, true);

    if (distance_on_line < start.distance) {
      distance_on_line = start.distance;
    } else if (distance_on_line > end.distance) {
      distance_on_line = end.distance;
    }

    mnm::msg::RoutePosition denormal_position;
    denormal_position.id = start.id;
    denormal_position.distance = distance_on_line;
    if (!normalizeRoutePosition(position, route, denormal_position)) {
      return false;
    }

    position.route_id = route.guid();
    return true;
  }

  // Find the nearest point on the route, without allowing
  // extrapolation.
  double min_distance_from_line = std::numeric_limits<double>::infinity();
  double min_distance_on_line = std::numeric_limits<double>::infinity();
  size_t min_segment_index = 0;

  for (size_t i = start_index; i <= end_index; ++i) {
    double distance_from_line;
    double distance_on_line;

    nearestDistanceToLineSegment(distance_from_line,
                                 distance_on_line,
                                 route.points[i+0].position(),
                                 route.points[i+1].position(),
                                 point,
                                 false, false);

    if (distance_from_line <= min_distance_from_line) {
      min_segment_index = i;
      min_distance_on_line = distance_on_line;
      min_distance_from_line = distance_from_line;
    }
  }

  // We have identified the closest segment.  We need to clamp it
  // to the window boundaries.
  if (min_segment_index == start_index) {
    nearestDistanceToLineSegment(min_distance_from_line,
                                 min_distance_on_line,
                                 route.points[min_segment_index+0].position(),
                                 route.points[min_segment_index+1].position(),
                                 point,
                                 true, false);
    if (min_distance_on_line < start.distance) {
      min_distance_on_line = start.distance;
    }
  } else if (min_segment_index == end_index) {
    nearestDistanceToLineSegment(min_distance_from_line,
                                 min_distance_on_line,
                                 route.points[min_segment_index+0].position(),
                                 route.points[min_segment_index+1].position(),
                                 point,
                                 false, true);
    if (min_distance_on_line > end.distance) {
      min_distance_on_line = end.distance;
    }
  }

  mnm::msg::RoutePosition denormal_position;
  denormal_position.id = route.points[min_segment_index].id();
  denormal_position.distance = min_distance_on_line;
  if (!normalizeRoutePosition(position, route, denormal_position)) {
    return false;
  }

  position.route_id = route.guid();
  return true;
}

static
void interpolateRouteSegment(
  RoutePoint &dst,
  const RoutePoint &p0,
  const RoutePoint &p1,
  double distance)
{
  double len = (p0.position()-p1.position()).length();

  double s;
  if (len > 1e-6) {
    s = distance / len;
  } else {
    // This is a degenerate case where the points are too close
    // together to define a numerically stable route point.
    if (distance < 0) {
      // If the distance is negative, the interpolated value will be
      // the first route point.
      s = 0.0;
    } else if (distance > 1) {
      // If the distance is positive, the interpolated value will be
      // the second route point.
      s = 1.0;
    } else {
      // Otherwise just take the center of the two points.
      s = 0.5;
    }
  }

  dst.setPosition((1.0-s)*p0.position() + s*p1.position());
  dst.setOrientation(p0.orientation().slerp(p1.orientation(), s));

  // Interpolate other known properties here.
}

bool normalizeRoutePosition(mnm::msg::RoutePosition &normalized_position,
                            const Route &route,
                            const mnm::msg::RoutePosition &position)
{
  size_t index;
  if (!route.findPointId(index, position.id)) {
    return false;
  }

  double distance = position.distance;
  while (distance < 0.0) {
    // We can't go past the start of the route.
    if (index == 0) {
      break;
    }

    // The distance is still negative, so we can't be on this
    // segment.  Move to the preceding segment.
    distance += (route.points[index-0].position() -
                 route.points[index-1].position()).length();
    index--;
  }

  while (distance > 0.0) {
    // We can't go past the end of the route.
    if (index+1 == route.points.size()) {
      break;
    }

    double segment_length = (route.points[index+0].position() -
                             route.points[index+1].position()).length();
    if (distance > segment_length) {
      // The distance is greater than this segment length, so we're
      // not on this segment.  Move to the following segment.
      distance -= segment_length;
      index++;
    } else {
      // The distance is within the length of this segment, so the
      // point is on this segment.
      break;
    }
  }

  normalized_position.route_id = position.route_id;
  normalized_position.distance = distance;
  normalized_position.id = route.points[index].id();
  return true;
}


bool interpolateRoutePosition(RoutePoint &dst,
                              const Route &route,
                              const mnm::msg::RoutePosition &position,
                              bool allow_extrapolation)
{
  mnm::msg::RoutePosition norm_position;
  if (!normalizeRoutePosition(norm_position, route, position)) {
    return false;
  }

  // Since we have a normalized position, we know it exists in the route.
  size_t index;
  route.findPointId(index, norm_position.id);

  // Special case when the point is before the start of the route.
  if (index == 0 && norm_position.distance < 0.0) {
    if (!allow_extrapolation) {
      return false;
    }

    if (route.points.size() < 2) {
      // This route point is before the start of the route and we
      // don't have enough information to extrapolate.
      return false;
    }

    interpolateRouteSegment(dst,
                            route.points[0],
                            route.points[1],
                            norm_position.distance);
    return true;
  }

  // Special case when the point is after the end of the route.
  if (index+1 == route.points.size() && norm_position.distance > 0.0) {
    if (!allow_extrapolation) {
      return false;
    }
    if (route.points.size() < 2) {
      // This route point is after the end of the route and we don't
      // have enough information to extrapolate.
      return false;
    }

    const RoutePoint &p0 = route.points[index-1];
    const RoutePoint &p1 = route.points[index-0];
    double extra_dist = (p1.position() - p0.position()).length();

    interpolateRouteSegment(dst,
                            p0,
                            p1,
                            norm_position.distance + extra_dist);
    return true;
  }

  interpolateRouteSegment(dst,
                          route.points[index+0],
                          route.points[index+1],
                          norm_position.distance);
  return true;
}

bool routeDistance(
  double &distance,
  const mnm::msg::RoutePosition &start,
  const mnm::msg::RoutePosition &end,
  const Route &route)
{
  size_t start_index;
  if (!route.findPointId(start_index, start.id)) {
    return false;
  }

  size_t end_index;
  if (!route.findPointId(end_index, end.id)) {
    return false;
  }

  size_t min_index = std::min(start_index, end_index);
  size_t max_index = std::max(start_index, end_index);

  double d = 0.0;
  if (route.header.frame_id == stu::_wgs84_frame) {
    for (size_t i = min_index; i < max_index; i++) {
      d += stu::GreatCircleDistance(route.points[i+1].position(), route.points[i].position());
    }
  } else {
    for (size_t i = min_index; i < max_index; i++) {
      d += (route.points[i+1].position() - route.points[i].position()).length();
    }
  }

  if (end_index < start_index) {
    d = -d;
  }

  distance = d + end.distance - start.distance;
  return true;
}

bool routeDistances(
  std::vector<double> &distances,
  const mnm::msg::RoutePosition &start,
  const std::vector<mnm::msg::RoutePosition> &ends,
  const Route &route)
{
  size_t start_index;
  if (!route.findPointId(start_index, start.id)) {
    // Without a start index, we can't calculate anything.
    return false;
  }

  // Find the indices for every point in the route, and the minimum
  // and maximum indices that we need to work over (the ROI).
  size_t min_index = start_index;
  size_t max_index = start_index;
  std::vector<int> indices;
  indices.resize(ends.size());
  for (size_t i = 0; i < ends.size(); ++i) {
    size_t pt_index;
    if (route.findPointId(pt_index, ends[i].id)) {
      indices[i] = pt_index;
      min_index = std::min(min_index, pt_index);
      max_index = std::max(max_index, pt_index);
    } else {
      indices[i] = -1;
    }
  }

  // This is the index of the start point in the ROI.
  const size_t roi_start_index = start_index - min_index;

  // We calculate the arc length of each point in the ROI relative to
  // the start point.  This vector covers the ROI (so it corresponds
  // from min_index to max_index)
  std::vector<double> arc_lengths;
  arc_lengths.resize(max_index-min_index+1);

  arc_lengths[roi_start_index] = 0.0;
  if (route.header.frame_id == stu::_wgs84_frame) {
    // Calculate the lengths before the start point.
    for (size_t rev_i = 1; rev_i <= roi_start_index; ++rev_i) {
      const size_t i = roi_start_index - rev_i;
      const tf2::Vector3 pt1 = route.points[min_index+i].position();
      const tf2::Vector3 pt2 = route.points[min_index+i+1].position();
      arc_lengths[i] = arc_lengths[i+1] - stu::GreatCircleDistance(pt1, pt2);
    }
    // Calculate the lengths after the start point.
    for (size_t i = roi_start_index+1; i < arc_lengths.size(); ++i) {
      const tf2::Vector3 pt1 = route.points[min_index+i].position();
      const tf2::Vector3 pt2 = route.points[min_index+i-1].position();
      arc_lengths[i] = arc_lengths[i-1] + stu::GreatCircleDistance(pt1, pt2);
    }
  } else {
    // Assume Euclidean coordinates.
    // Calculate the lengths before the start point.
    for (size_t rev_i = 1; rev_i <= roi_start_index; ++rev_i) {
      const size_t i = roi_start_index - rev_i;
      const tf2::Vector3 pt1 = route.points[min_index+i].position();
      const tf2::Vector3 pt2 = route.points[min_index+i+1].position();
      arc_lengths[i] = arc_lengths[i+1] - (pt2-pt1).length();
    }
    // Calculate the lengths after the start point.
    for (size_t i = roi_start_index+1; i < arc_lengths.size(); ++i) {
      const tf2::Vector3 pt1 = route.points[min_index+i].position();
      const tf2::Vector3 pt2 = route.points[min_index+i-1].position();
      arc_lengths[i] = arc_lengths[i-1] + (pt2-pt1).length();
    }
  }

  // Now we can calculate the distances.
  distances.resize(ends.size());
  for (size_t i = 0; i < distances.size(); ++i) {
    if (indices[i] < 0) {
      distances[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }

    const size_t cache_index = indices[i]-min_index;
    distances[i] = arc_lengths[cache_index] + ends[i].distance - start.distance;
  }

  return true;
}

bool extractSubroute(
  Route &sub_route,
  const Route &route,
  const marti_nav_msgs::msg::RoutePosition &start,
  const marti_nav_msgs::msg::RoutePosition &end)
{
  sub_route.header = route.header;
  sub_route.properties_ = route.properties_;
  sub_route.guid_ = route.guid_;
  sub_route.name_ = route.name_;

  mnm::msg::RoutePosition norm_start;
  if (!normalizeRoutePosition(norm_start, route, start)) {
    return false;
  }

  mnm::msg::RoutePosition norm_end;
  if (!normalizeRoutePosition(norm_end, route, end)) {
    return false;
  }

  // Since we have a normalized position, we know it exists in the route.
  size_t start_index;
  route.findPointId(start_index, norm_start.id);

  size_t end_index;
  route.findPointId(end_index, norm_end.id);

  // If the end distance is after the id point, round up to the next
  // point.
  if (norm_end.distance > 0.0) {
    end_index += 1;
  }

  // Increment the end_index so that we can iterate from [start, end),
  // and make sure we stay within the array bounds.
  end_index++;
  end_index = std::min(end_index, route.points.size());

  if (end_index <= start_index)
  {
    sub_route.points.clear();
    sub_route.rebuildPointIndex();
    return true;
  }

  sub_route.points.reserve(end_index - start_index);
  for (size_t i = start_index; i < end_index; i++) {
    sub_route.points.push_back(route.points[i]);
  }
  sub_route.rebuildPointIndex();

  return true;
}
}  // namespace swri_route_util
