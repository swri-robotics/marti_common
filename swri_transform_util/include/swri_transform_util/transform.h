// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef TRANSFORM_UTIL_TRANSFORM_H_
#define TRANSFORM_UTIL_TRANSFORM_H_

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

namespace swri_transform_util
{
  /**
   * @brief Base class for Transform implementations.
   *
   * swri_transform_util::Transform uses a "pointer to implementation" design
   * pattern. Extend this class to create new implementations of Transform.
   * TransformImpl and its descendants should not be used bare, only as part
   * of a swri_transform_util::Transform.
   */
  class TransformImpl
  {
  public:
    TransformImpl() {}
    virtual ~TransformImpl() {}

    /**
     * @brief Apply this transform to a 3D vector
     *
     * @param[in]  v_in  Input vector
     * @param[out] v_out Transformed vector
     */
    virtual void Transform(
      const tf::Vector3& v_in, tf::Vector3& v_out) const = 0;
    
    /**
     * @brief Get the orientation of this transform
     *
     * Get the orientation of this transform by getting the vector between
     * the origin point and a point offset 1 on the x axis.
     * @return The orientation component of the transform
     */
    virtual tf::Quaternion GetOrientation() const
    {
      tf::Vector3 offset;
      Transform(tf::Vector3(1, 0, 0), offset);

      tf::Vector3 origin;
      Transform(tf::Vector3(0, 0, 0), origin);

      tf::Vector3 vector = offset - origin;

      // Use the "half-way quaternion method" of summing and normalizing a
      // quaternion with twice the rotation between the vector and the x-axis and
      // the zero rotation.

      tf::Vector3 cross = tf::Vector3(1, 0, 0).cross(vector);
      double w = vector.length() + tf::Vector3(1, 0, 0).dot(vector);
      return tf::Quaternion(cross.x(), cross.y(), cross.z(), w).normalized();
    }
    
    /// Time stamp for this transform
    ros::Time stamp_;
  };

  /**
   * @brief An abstraction of the tf::Transform class to support transforms in
   * addition to the rigid transforms supported by tf.
   *
   * Additional transforms are implemented through transformer plug-ins.
   *
   * It can be used in conjunction with the tf::Vector3 data type.
   */
  class Transform
  {
  public:
    /**
     * @brief Generates an identity transform.
     */
    Transform();

    /**
     * @brief Generates a standard rigid transform from a tf::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    explicit Transform(const tf::Transform& transform);
    
    /**
     * @brief Generates a standard rigid transform from a tf::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    explicit Transform(const tf::StampedTransform& transform);

    /**
     * @brief Defines the transform using an arbitrary transform implementation.
     *
     * @param[in]  transform  The input transform implementation.
     */
    explicit Transform(boost::shared_ptr<TransformImpl> transform);

    /**
     * @brief Assignment operator for tf::Transform.
     *
     * Generates a standard rigid transform from a tf::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    Transform& operator=(const tf::Transform transform);

    /**
     * @brief Assignment operator for TransformImpl.
     *
     * Note: The transform implementation is only a shallow copy.
     *
     * @param[in]  transform  The input transform.
     */
    Transform& operator=(boost::shared_ptr<TransformImpl> transform);

    /**
     * @brief Apply the transform to a vector and return the result.
     *
     * @param[in]  v  The vector.
     *
     * @returns The transformed vector.
     */
    tf::Vector3 operator()(const tf::Vector3& v) const;

    /**
     * @brief Apply the transform to a vector and return the result.
     *
     * @param[in]  v  The vector.
     *
     * @returns The transformed vector.
     */
    tf::Vector3 operator*(const tf::Vector3& v) const;

    /**
     * @brief Apply the transform to a quaternion and return the result.
     *
     * @param[in]  q  The quaternion.
     *
     * @returns The transformed quaternion.
     */
    tf::Quaternion operator*(const tf::Quaternion& q) const;

    /**
     * @brief Return a TF transform equivalent to this transform
     *
     * @return The equivalent tf::Transform
     */
    tf::Transform GetTF() const;

    /**
     * Return the inverse transform.
     *
     * @returns The inverse transform.
     */
    Transform Inverse() const;

    /**
     * @brief Get the origin (translation component) of the transform
     *
     * The result of this should always be equal to applying the transform to
     * the vector (0, 0, 0).
     *
     * @return The origin (translation component) of the transform
     */
    tf::Vector3 GetOrigin() const;

    /**
     * @brief Get the orientation (rotation component) of the transform
     *
     * @return The orientation (translation component) of the transform
     */
    tf::Quaternion GetOrientation() const;
    
    /**
     * @brief Get the time stamp of the transform
     * @return The time stamp of the transform
     */
    ros::Time GetStamp() { return transform_->stamp_; }

  private:
    /// Pointer to the implementation of the transform
    boost::shared_ptr<TransformImpl> transform_;
  };

  /**
   * @brief Specialization of swri_transform_util::TransformImpl that represents
   * the identity transform
   */
  class IdentityTransform : public TransformImpl
  {
  public:
    /**
     * @brief Construct an identity transform with stamp_=ros::Time::now()
     */
    IdentityTransform() { stamp_ = ros::Time::now(); }

    /**
     * @brief Apply the identity tranform to a 3D vector(sets v_out=v_in)
     *
     * @param[in] v_in  Input vector
     * @param[out] v_out Ouput vector
     */
    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;
  };

  /**
   * @brief Specialization of swri_transform_util::TransformImpl that performs
   * TF transformation
   */
  class TfTransform : public TransformImpl
  {
  public:
    /**
     * @brief Construct a TfTransform from a tf::Transform
     * @param transform The TF Transform that this TfTransform performs
     */
    explicit TfTransform(const tf::Transform& transform);

    /**
     * @brief Construct a TfTransform from a tf::StampedTransform
     * @param transform The TF StampedTransform that this TfTransform performs
     */
    explicit TfTransform(const tf::StampedTransform& transform);

    /**
     * @brief Apply this transform to a 3D vector using TF
     *
     * @param[in]  v_in  Input vector
     * @param[out] v_out Transformed vector
     */
    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

    /**
     * @brief Get the orientation component of this transform using TF
     * @return The orientation component of the transform
     */
    virtual tf::Quaternion GetOrientation() const;

  protected:
    tf::Transform transform_;
  };
}

#endif  // TRANSFORM_UTIL_TRANSFORM_H_
