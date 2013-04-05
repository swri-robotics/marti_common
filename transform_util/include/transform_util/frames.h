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

#include <string>

namespace transform_util
{
  /**
   * Special frame id for data defined in the WGS84 lat/lon coordinate system.
   */
  static const std::string _wgs84_frame = "/wgs84";

  /**
   * Special frame id for data defined in the UTM coordinate system.
   *
   * The zone is assumed to be the same as the LocalXY origin of the system.
   * Because of this zone transitions are not supported.
   */
  static const std::string _utm_frame = "/utm";

  /**
   * Special frame id for data defined a LocalXY coordinate system.
   *
   * Dependent on the LocalXY origin of the system.
   */
  static const std::string _local_xy_frame = "/local_xy";

  static const std::string _tf_frame = "/tf";
}
