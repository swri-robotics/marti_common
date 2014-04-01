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

#ifndef TRANSFORM_UTIL_TRANSFORM_H_
#define TRANSFORM_UTIL_TRANSFORM_H_

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

namespace transform_util
{
  class TransformImpl
  {
  public:
    TransformImpl() {}
    virtual ~TransformImpl() {}
    virtual void Transform(
      const tf::Vector3& v_in, tf::Vector3& v_out) const = 0;
    
    ros::Time stamp_;
  };

  /**
   * An abstraction of the tf::Transform class to support transforms in addition
   * to the rigid transforms supported by tf.
   *
   * Additional transforms are implemented through transformer plug-ins.
   *
   * It can be used in conjunction with the tf::Vector3 data type.
   */
  class Transform
  {
  public:
    /**
     * Constructor.
     *
     * Generates an identity transform.
     */
    Transform();

    /**
     * Constructor.
     *
     * Generates a standard rigid transform from a tf::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    explicit Transform(const tf::Transform& transform);
    
    /**
     * Constructor.
     *
     * Generates a standard rigid transform from a tf::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    explicit Transform(const tf::StampedTransform& transform);

    /**
     * Constructor.
     *
     * Defines the transform using an arbitrary transform implementation.
     *
     * @param[in]  transform  The input transform implementation.
     */
    explicit Transform(boost::shared_ptr<TransformImpl> transform);

    /**
     * Assignment operator for tf::Transform.
     *
     * Generates a standard rigid transform from a tf::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    Transform& operator=(const tf::Transform transform);

    /**
     * Assignment operator for TransformImpl.
     *
     * Note: The transform implementation is only a shallow copy.
     *
     * @param[in]  transform  The input transform.
     */
    Transform& operator=(boost::shared_ptr<TransformImpl> transform);

    /**
     * Return the transform of the vector.
     *
     * @param[in]  v  The vector.
     *
     * @returns The transformed vector.
     */
    tf::Vector3 operator()(const tf::Vector3& v) const;

    /**
     * Return the transform of the vector.
     *
     * @param[in]  v  The vector.
     *
     * @returns The transformed vector.
     */
    tf::Vector3 operator*(const tf::Vector3& v) const;

    /**
     * Return the inverse transform.
     *
     * @returns The inverse transform.
     */
    Transform Inverse() const;

    tf::Vector3 GetOrigin() const;

    tf::Quaternion GetOrientation() const;
    
    ros::Time GetStamp() { return transform_->stamp_; }

  private:
    boost::shared_ptr<TransformImpl> transform_;
  };

  class IdentityTransform : public TransformImpl
  {
  public:
    IdentityTransform() { stamp_ = ros::Time::now(); }
    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;
  };

  class TfTransform : public TransformImpl
  {
  public:
    explicit TfTransform(const tf::Transform& transform);
    explicit TfTransform(const tf::StampedTransform& transform);
    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

  protected:
    tf::Transform transform_;
  };
}

#endif  // TRANSFORM_UTIL_TRANSFORM_H_
