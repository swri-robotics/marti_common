// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-62987
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

#ifndef TRANSFORM_UTIL_TRANSFORM_UTIL_H_
#define TRANSFORM_UTIL_TRANSFORM_UTIL_H_

#include <tf/transform_datatypes.h>

#include <transform_util/local_xy_util.h>

namespace transform_util
{
  tf::Transform GetRelativeTransform(
      double latitude,
      double longitude,
      double yaw,
      double reference_latitude,
      double reference_longitude,
      double reference_yaw);
}

#endif  // TRANSFORM_UTIL_TRANSFORM_UTIL_H_
