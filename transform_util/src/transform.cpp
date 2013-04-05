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

#include <transform_util/transform.h>

#include <boost/make_shared.hpp>

namespace transform_util
{
  Transform::Transform() :
    transform_(boost::make_shared<IdentityTransform>())
  {
  }

  Transform::Transform(const tf::Transform& transform) :
    transform_(boost::make_shared<TfTransform>(transform))
  {
  }

  Transform::Transform(boost::shared_ptr<TransformImpl> transform) :
    transform_(transform)
  {

  }

  Transform& Transform::operator=(const tf::Transform transform)
  {
    transform_ = boost::make_shared<TfTransform>(transform);

    return *this;
  }

  Transform& Transform::operator=(boost::shared_ptr<TransformImpl> transform)
  {
    transform_ = transform;

    return *this;
  }

  tf::Vector3 Transform::operator()(const tf::Vector3& v) const
  {
    tf::Vector3 transformed;

    transform_->Transform(v, transformed);

    return transformed;
  }

  tf::Vector3 Transform::operator*(const tf::Vector3& v) const
  {
    tf::Vector3 transformed;

    transform_->Transform(v, transformed);

    return transformed;
  }
  
  void IdentityTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    v_out = v_in;
  }
  
  TfTransform::TfTransform(const tf::Transform& transform) :
    transform_(transform)
  {
  }
  
  void TfTransform::Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const
  {
    v_out = transform_ * v_in;
  }
}
