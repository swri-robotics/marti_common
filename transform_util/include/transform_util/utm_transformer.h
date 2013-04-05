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

#ifndef TRANSFORM_UTIL_UTM_TRANSFORMER_H_
#define TRANSFORM_UTIL_UTM_TRANSFORMER_H_

#include <map>
#include <string>

#include <boost/shared_ptr.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <transform_util/utm_util.h>
#include <transform_util/local_xy_util.h>
#include <transform_util/transformer.h>

namespace transform_util
{
  class UtmTransformer : public Transformer
  {
    public:
      UtmTransformer();

      virtual std::map<std::string, std::vector<std::string> > Supports() const;

      virtual bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& time,
        Transform& transform);

    protected:
      virtual bool Initialize();

      boost::shared_ptr<UtmUtil> utm_util_;
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;

      int32_t utm_zone_;
      char utm_band_;
  };

  class UtmToTfTransform : public TransformImpl
  {
  public:
    UtmToTfTransform(
      const tf::Transform& transform,
      boost::shared_ptr<UtmUtil> utm_util,
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util,
      int32_t utm_zone,
      char utm_band);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

  protected:
    tf::Transform transform_;
    boost::shared_ptr<UtmUtil> utm_util_;
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;
    int32_t utm_zone_;
    char utm_band_;
  };

  class TfToUtmTransform : public TransformImpl
  {
  public:
    TfToUtmTransform(
      const tf::Transform& transform,
      boost::shared_ptr<UtmUtil> utm_util,
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

  protected:
    tf::Transform transform_;
    boost::shared_ptr<UtmUtil> utm_util_;
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;
  };

  class UtmToWgs84Transform : public TransformImpl
  {
  public:
    UtmToWgs84Transform(
        boost::shared_ptr<UtmUtil> utm_util,
        int32_t utm_zone,
        char utm_band);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

  protected:
    boost::shared_ptr<UtmUtil> utm_util_;
    int32_t utm_zone_;
    char    utm_band_;
  };

  class Wgs84ToUtmTransform : public TransformImpl
  {
  public:
    explicit Wgs84ToUtmTransform(boost::shared_ptr<UtmUtil> utm_util);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

  protected:
    boost::shared_ptr<UtmUtil> utm_util_;
  };
}

#endif  // TRANSFORM_UTIL_UTM_TRANSFORMER_H_
