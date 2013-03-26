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

#ifndef TRANSFORM_UTIL_UTM_TRANSFORMER_H_
#define TRANSFORM_UTIL_UTM_TRANSFORMER_H_

#include <transform_util/utm_transformer.h>

#include <boost/make_shared.hpp>

#include <transform_util/frames.h>

namespace transform_util
{
  UtmTransformer::UtmTransformer() :
    utm_util_(boost::make_shared<UtmTransforms>())
  {
    
  }
  
  std::map<std::string, std::string> UtmTransformer::Supports() const
  {
    std::map<std::string, std::string> supports;
    
    supports[_utm_frame] = _wgs84_frame;
    supports[_wgs84_frame] = _utm_frame;
    supports[_utm_frame] = _tf_frame;
    supports[_tf_frame] = _utm_frame;
    
    return supports;
  }
  
  bool UtmTransformer::GetTransform(
    const std::string& target_frame,
    const std::string& source_frame,
    const ros::Time& time,
    Transform& transform)
  {
    if (target_frame_ == _utm_frame)
    {
      if (source_frame_ == _wgs84_frame)
      {
        transform = Transform(boost::make_shared<Wgs84ToUtmTransform>(utm_util_);
        return true;
      }
      else if (initialized_)
      {
        tf::StampedTransform tf_transform;
        //TODO(malban): Get tf transform;
        
        transform = Transform(boost::make_shared<TfToUtmTransform>(
          tf_transform,
          utm_util_,
          local_xy_util_);
        return true;
      }
    }
    else if target_frame_ == _wgs84_frame && source_frame_ == _utm_frame)
    {
      transform = Transform(boost::make_shared<UtmToWgs84Transform>(utm_util_);
      return true;
    }
    else if (initialized_ && source_frame_ == _utm_frame)
    {
      tf::StampedTransform tf_transform;
      //TODO(malban): Get tf transform;
        
      transform = Transform(boost::make_shared<UtmToTfTransform>(
        tf_transform,
        utm_util_,
        local_xy_util_);
      return true;
    }
    
    return false;
  }
 
  bool UtmTransformer::Initialize()
  {
    // TODO(malban): Initialize LocalXY util with origin.
    
    return false;
  }
  
  UtmToTfTransform::UtmToTfTransform(
      const tf::Transform& transform,
      boost::shared_ptr<UtmTransforms>& utm_util,
      boost::shared_ptr<LocalXyUtil>& local_xy_util) :
      transform_(transform),
      utm_util_(utm_util),
      local_xy_util_(local_xy_util)
   {
   
   }
      
  void UtmToTfTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    // TODO(malban)
  }
  
  TfToUtmTransform::TfToUtmTransform(
      const tf::Transform& transform,
      boost::shared_ptr<UtmTransforms>& utm_util,
      boost::shared_ptr<LocalXyUtil>& local_xy_util) :
      transform_(transform),
      utm_util_(utm_util),
      local_xy_util_(local_xy_util)
   {
   
   }
      
  void TfToUtmTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    // TODO(malban)
  }
  
  UtmToWgs84Transform::UtmToWgs84Transform(
    boost::shared_ptr<UtmTransforms>& utm_util) :
    utm_util_(utm_util)
  {
  
  }
      
  void UtmToWgs84Transform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    // TODO(malban)
  }
    
  
  Wgs84ToUtmTransform::Wgs84ToUtmTransform(
    boost::shared_ptr<UtmTransforms>& utm_util) :
    utm_util_(utm_util)
  {
  
  }
      
  void Wgs84ToUtmTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    // TODO(malban)
  }
}

