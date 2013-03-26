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

#ifndef TRANSFORM_UTIL_TRANSFORMER_H_
#define TRANSFORM_UTIL_TRANSFORMER_H_

#include <map>
#include <string>

#include <boost/shared_ptr.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <transform_util/transform.h>

namespace transform_util
{
  class Transformer
  {
    public:
      Transformer();
      virtual ~Transformer();
      
      void Initialize(const boost::shared_ptr<tf::TransformListener>& tf);
      
      virtual std::map<std::string, std::string> Supports() const = 0;
      
      virtual bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& time,
        Transform& transform) = 0;
        
    protected:
      bool initialized_;
      boost::shared_ptr<tf::TransformListener> tf_listener_;
      std::map<std::string, std::string> supports_;
      
      virtual bool Initialize();

      virtual bool GetTransform(
          const std::string& target_frame,
          const std::string& source_frame,
          const ros::Time& time,
          tf::StampedTransform& transform) const;
  };
}

#endif  // TRANSFORM_UTIL_TRANSFORMER_H_
