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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/transform_datatypes.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>

namespace swri_transform_util
{
  class StampInterface
  {
  public:
    virtual tf2::TimePoint GetStamp() const = 0;

    virtual void SetStamp(const tf2::TimePoint&) = 0;
  };

  class StampedTransformStampInterface : virtual public StampInterface
  {
  public:
    tf2::TimePoint GetStamp() const final
    {
      return tf2_ros::fromMsg(transform_.header.stamp);
    }

    void SetStamp(const tf2::TimePoint& time) final
    {
      transform_.header.stamp = tf2_ros::toMsg(time);
    };

    tf2::Stamped<tf2::Transform> GetStampedTransform() const
    {
      tf2::Stamped<tf2::Transform> tf;
      tf2::fromMsg(transform_, tf);
      return tf;
    }

  protected:
    geometry_msgs::msg::TransformStamped transform_;
  };

  class Tf2StampStampInterface : virtual public StampInterface
  {
  public:
    tf2::TimePoint GetStamp() const final
    {
      return stamp_;
    }

    void SetStamp(const tf2::TimePoint& time) final
    {
      stamp_ = time;
    };

  protected:
    tf2::TimePoint stamp_;
  };

  /**
   * Base class for Transform implementations.
   *
   * swri_transform_util::Transform uses a "pointer to implementation" design
   * pattern. Extend this class to create new implementations of Transform.
   * TransformImpl and its descendants should not be used bare, only as part
   * of a swri_transform_util::Transform.
   */
  class TransformImpl : virtual public StampInterface
  {
  public:
    explicit TransformImpl(const rclcpp::Logger& logger = rclcpp::get_logger("swri_transform_util::TransformImpl")) :
      logger_(logger)
    {};
    virtual ~TransformImpl() = default;

    /**
     * Apply this transform to a 3D vector
     *
     * @param[in]  v_in  Input vector
     * @param[out] v_out Transformed vector
     */
    virtual void Transform(
      const tf2::Vector3& v_in, tf2::Vector3& v_out) const = 0;

    /**
     * Get the orientation of this transform
     *
     * Get the orientation of this transform by getting the vector between
     * the origin point and a point offset 1 on the x axis.
     * @return The orientation component of the transform
     */
    virtual tf2::Quaternion GetOrientation() const
    {
      tf2::Vector3 offset;
      Transform(tf2::Vector3(1, 0, 0), offset);

      tf2::Vector3 origin;
      Transform(tf2::Vector3(0, 0, 0), origin);

      tf2::Vector3 vector = offset - origin;

      // Use the "half-way quaternion method" of summing and normalizing a
      // quaternion with twice the rotation between the vector and the x-axis and
      // the zero rotation.

      tf2::Vector3 cross = tf2::Vector3(1, 0, 0).cross(vector);
      double w = vector.length() + tf2::Vector3(1, 0, 0).dot(vector);
      return tf2::Quaternion(cross.x(), cross.y(), cross.z(), w).normalized();
    }

    virtual std::shared_ptr<TransformImpl> Inverse() const = 0;

  protected:

    rclcpp::Logger logger_;
  };
  typedef std::shared_ptr<TransformImpl> TransformImplPtr;

  /**
   * An abstraction of the tf2::Transform class to support transforms in
   * addition to the rigid transforms supported by tf.
   *
   * Additional transforms are implemented through transformer plug-ins.
   *
   * It can be used in conjunction with the tf2::Vector3 data type.
   */
  class Transform
  {
  public:
    /**
     * Generates an identity transform.
     */
    Transform();

    /**
     * Generates a standard rigid transform from a tf2::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    explicit Transform(const tf2::Transform& transform);

    /**
     * Generates a standard rigid transform from a tf2::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    explicit Transform(const tf2::Stamped<tf2::Transform>& transform);

    /**
     * Defines the transform using an arbitrary transform implementation.
     *
     * @param[in]  transform  The input transform implementation.
     */
    explicit Transform(std::shared_ptr<TransformImpl> transform);

    /**
     * Assignment operator for tf2::Transform.
     *
     * Generates a standard rigid transform from a tf2::Transform.
     *
     * @param[in]  transform  The input transform.
     */
    Transform& operator=(const tf2::Transform transform);

    /**
     * Assignment operator for TransformImpl.
     *
     * Note: The transform implementation is only a shallow copy.
     *
     * @param[in]  transform  The input transform.
     */
    Transform& operator=(std::shared_ptr<TransformImpl> transform);

    /**
     * Apply the transform to a vector and return the result.
     *
     * @param[in]  v  The vector.
     *
     * @returns The transformed vector.
     */
    tf2::Vector3 operator()(const tf2::Vector3& v) const;

    /**
     * Apply the transform to a vector and return the result.
     *
     * @param[in]  v  The vector.
     *
     * @returns The transformed vector.
     */
    tf2::Vector3 operator*(const tf2::Vector3& v) const;

    /**
     * Apply the transform to a quaternion and return the result.
     *
     * @param[in]  q  The quaternion.
     *
     * @returns The transformed quaternion.
     */
    tf2::Quaternion operator*(const tf2::Quaternion& q) const;

    /**
     * Return a TF transform equivalent to this transform
     *
     * @return The equivalent tf2::Transform
     */
    tf2::Transform GetTF() const;

    /**
     * Return the inverse transform.
     *
     * @returns The inverse transform.
     */
    Transform Inverse() const;

    /**
     * Get the origin (translation component) of the transform
     *
     * The result of this should always be equal to applying the transform to
     * the vector (0, 0, 0).
     *
     * @return The origin (translation component) of the transform
     */
    tf2::Vector3 GetOrigin() const;

    /**
     * Get the orientation (rotation component) of the transform
     *
     * @return The orientation (translation component) of the transform
     */
    tf2::Quaternion GetOrientation() const;

    /**
     * Get the time stamp of the transform
     * @return The time stamp of the transform
     */
    tf2::TimePoint GetStamp() { return transform_->GetStamp(); }

  private:
    /// Pointer to the implementation of the transform
    std::shared_ptr<TransformImpl> transform_;
  };

  /**
   * Specialization of swri_transform_util::TransformImpl that represents
   * the identity transform
   */
  class IdentityTransform : public TransformImpl, public Tf2StampStampInterface
  {
  public:
    /**
     * Construct an identity transform.
     */
    IdentityTransform() { Tf2StampStampInterface::SetStamp(tf2::TimePointZero); }

    /**
     * Apply the identity tranform to a 3D vector(sets v_out=v_in)
     *
     * @param[in] v_in  Input vector
     * @param[out] v_out Ouput vector
     */
    void Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const override;
    TransformImplPtr Inverse() const override;
  };

  /**
   * Specialization of swri_transform_util::TransformImpl that performs
   * TF transformation
   */
  class TfTransform : public TransformImpl, public Tf2StampStampInterface
  {
  public:
    /**
     * Construct a TfTransform from a tf2::Transform
     * @param transform The TF Transform that this TfTransform performs
     */
    explicit TfTransform(const tf2::Transform& transform);

    /**
     * Construct a TfTransform from a tf2::StampedTransform
     * @param transform The TF StampedTransform that this TfTransform performs
     */
    explicit TfTransform(const tf2::Stamped<tf2::Transform>& transform);

    /**
     * Apply this transform to a 3D vector using TF
     *
     * @param[in]  v_in  Input vector
     * @param[out] v_out Transformed vector
     */
    void Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const override;

    /**
     * Get the orientation component of this transform using TF
     * @return The orientation component of the transform
     */
    tf2::Quaternion GetOrientation() const override;
    TransformImplPtr Inverse() const override;

  protected:
    tf2::Transform transform_;
  };
}

#endif  // TRANSFORM_UTIL_TRANSFORM_H_
