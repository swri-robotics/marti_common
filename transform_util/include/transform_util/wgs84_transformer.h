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

#ifndef TRANSFORM_UTIL_WGS84_TRANSFORMER_H_
#define TRANSFORM_UTIL_WGS84_TRANSFORMER_H_

#include <map>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <transform_util/local_xy_util.h>
#include <transform_util/transformer.h>

namespace transform_util
{
  class Wgs84Transformer : public Transformer
  {
    public:
      virtual std::map<std::string, std::vector<std::string> > Supports() const;

      virtual bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& time,
        Transform& transform);

    protected:
      virtual bool Initialize();

      boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;
  };

  class TfToWgs84Transform : public TransformImpl
  {
  public:
    TfToWgs84Transform(
      const tf::Transform& transform,
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

  protected:
    tf::Transform transform_;
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;
  };

  class Wgs84ToTfTransform : public TransformImpl
  {
  public:
    Wgs84ToTfTransform(
      const tf::Transform& transform,
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

  protected:
    tf::Transform transform_;
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;
  };
}

#endif  // TRANSFORM_UTIL_WGS84_TRANSFORMER_H_
