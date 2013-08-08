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

#include <transform_util/utm_transformer.h>

#include <boost/make_shared.hpp>

#include <transform_util/frames.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(
    transformers,
    utm,
    transform_util::UtmTransformer,
    transform_util::Transformer)

namespace transform_util
{
  UtmTransformer::UtmTransformer() :
    utm_util_(boost::make_shared<UtmUtil>())
  {
  }

  std::map<std::string, std::vector<std::string> > UtmTransformer::Supports() const
  {
    std::map<std::string, std::vector<std::string> >  supports;

    supports[_utm_frame].push_back(_wgs84_frame);
    supports[_wgs84_frame].push_back(_utm_frame);
    supports[_utm_frame].push_back(_tf_frame);
    supports[_tf_frame].push_back(_utm_frame);

    return supports;
  }

  bool UtmTransformer::GetTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const ros::Time& time,
    Transform& transform)
  {
    if (target_frame == _utm_frame)
    {
      if (source_frame == _wgs84_frame)
      {
        transform = boost::make_shared<Wgs84ToUtmTransform>(utm_util_);

        return true;
      }
      else
      {
        if (!initialized_)
        {
          Initialize();
        }

        if (initialized_)
        {
          tf::StampedTransform tf_transform;
          if (!Transformer::GetTransform(local_xy_util_->FrameId(), source_frame, time, tf_transform))
          {
            ROS_ERROR("Failed to get transform from %s to local_xy(%s)",
                source_frame.c_str(), local_xy_util_->FrameId().c_str());
            return false;
          }

          transform = boost::make_shared<TfToUtmTransform>(
                  tf_transform,
                  utm_util_,
                  local_xy_util_);

          return true;
        }
      }
    }
    else if (target_frame == _wgs84_frame && source_frame == _utm_frame)
    {
      if (!initialized_)
      {
        Initialize();
      }

      if (initialized_)
      {
        transform = boost::make_shared<UtmToWgs84Transform>(
                utm_util_,
                utm_zone_,
                utm_band_);

        return true;
      }
    }
    else if (source_frame == _utm_frame)
    {
      if (!initialized_)
      {
        Initialize();
      }

      if (initialized_)
      {
        tf::StampedTransform tf_transform;
        if (!Transformer::GetTransform(target_frame, local_xy_util_->FrameId(), time, tf_transform))
        {
          ROS_ERROR("Failed to get transform from local_xy(%s) to %s",
              local_xy_util_->FrameId().c_str(), target_frame.c_str());
          return false;
        }

        transform = boost::make_shared<UtmToTfTransform>(
                tf_transform,
                utm_util_,
                local_xy_util_,
                utm_zone_,
                utm_band_);

        return true;
      }
      else
      {
        ROS_WARN("Failed to initialize LocalXY origin");
      }
    }

    ROS_WARN("Failed to get UTM transform");
    return false;
  }

  bool UtmTransformer::Initialize()
  {
    // Initialize LocalXY util with an origin.
    local_xy_util_ = ParseLocalXyOrigin();

    if (local_xy_util_)
    {
      utm_zone_ = GetZone(local_xy_util_->ReferenceLongitude());
      utm_band_ = GetBand(local_xy_util_->ReferenceLatitude());
    }

    initialized_ = local_xy_util_;

    return initialized_;
  }

  UtmToTfTransform::UtmToTfTransform(
      const tf::Transform& transform,
      boost::shared_ptr<UtmUtil> utm_util,
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util,
      int32_t utm_zone,
      char utm_band) :
      transform_(transform),
      utm_util_(utm_util),
      local_xy_util_(local_xy_util),
      utm_zone_(utm_zone),
      utm_band_(utm_band)
   {
   }

  void UtmToTfTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    // Convert to WGS84 latitude and longitude
    double lat, lon;
    utm_util_->ToLatLon(utm_zone_, utm_band_, v_in.x(), v_in.y(), lat, lon);

    // Convert to the LocalXY coordinate system.
    double x, y;
    local_xy_util_->ToLocalXy(lat, lon, x, y);

    // Transform from the LocalXY coordinate frame using the TF transform
    v_out.setValue(x, y, v_in.z());
    v_out = transform_ * v_out;
  }

  TfToUtmTransform::TfToUtmTransform(
      const tf::Transform& transform,
      boost::shared_ptr<UtmUtil> utm_util,
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util) :
      transform_(transform),
      utm_util_(utm_util),
      local_xy_util_(local_xy_util)
   {
   }

  void TfToUtmTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    // Transform into the LocalXY coordinate frame using the TF transform
    tf::Vector3 local_xy = transform_ * v_in;

    // Convert to WGS84 latitude and longitude
    double latitude, longitude;
    local_xy_util_->ToWgs84(local_xy.x(), local_xy.y(), latitude, longitude);

    // Convert to UTM easting and northing
    double easting, northing;
    utm_util_->ToUtm(latitude, longitude, easting, northing);
    v_out.setValue(easting, northing, local_xy.z());
  }

  UtmToWgs84Transform::UtmToWgs84Transform(
    boost::shared_ptr<UtmUtil> utm_util,
    int32_t utm_zone,
    char utm_band) :
    utm_util_(utm_util),
    utm_zone_(utm_zone),
    utm_band_(utm_band)
  {
  }

  void UtmToWgs84Transform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    double lat, lon;
    utm_util_->ToLatLon(utm_zone_, utm_band_, v_in.x(), v_in.y(), lat, lon);
    v_out.setValue(lon, lat, v_in.z());
  }


  Wgs84ToUtmTransform::Wgs84ToUtmTransform(
    boost::shared_ptr<UtmUtil> utm_util) :
    utm_util_(utm_util)
  {
  }

  void Wgs84ToUtmTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    double easting, northing;
    utm_util_->ToUtm(v_in.y(), v_in.x(), easting, northing);
    v_out.setValue(easting, northing, v_in.z());
  }
}
