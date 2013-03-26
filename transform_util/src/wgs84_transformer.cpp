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

#include <transform_util/wgs84_transformer.h>

#include <boost/make_shared.hpp>

#include <transform_util/frames.h>

namespace transform_util
{
  std::map<std::string, std::string> Wgs84Transformer::Supports() const
  {
    std::map<std::string, std::string> supports;

    supports[_wgs84_frame] = _tf_frame;
    supports[_tf_frame] = _wgs84_frame;

    return supports;
  }

  bool Wgs84Transformer::GetTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const ros::Time& time,
    Transform& transform)
  {
    if (target_frame == _wgs84_frame)
    {

    }
    else if (source_frame == _wgs84_frame)
    {

    }

    return false;
  }

  bool Wgs84Transformer::Initialize()
  {
    // Initialize LocalXY util with an origin.
    local_xy_util_ = ParseLocalXyOrigin();

    return local_xy_util_;
  }
}

