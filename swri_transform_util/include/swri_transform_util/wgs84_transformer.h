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

#ifndef TRANSFORM_UTIL_WGS84_TRANSFORMER_H_
#define TRANSFORM_UTIL_WGS84_TRANSFORMER_H_

#include <map>
#include <string>
#include <vector>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <swri_transform_util/local_xy_util.h>
#include <swri_transform_util/transformer.h>

namespace swri_transform_util
{
  /***
   * Specialization of Transformer to perform transforms to/from WGS84
   */
  class Wgs84Transformer : public Transformer
  {
    public:
      explicit Wgs84Transformer(LocalXyWgs84UtilPtr local_xy_util);

      /**
       * Get a map of the transforms supported by this Transformer
       * @return A map from source frame IDs to list of destination frame IDs.
       *   A source->destination entry does not imply that the inverse
       *   transform is supported as well.
       */
      std::map<std::string, std::vector<std::string> > Supports() const override;

      /**
       * Get a Transform from a non-UTM frame to UTM or vice-versa
       *
       * Gets the swri_transform_util::Transform that transforms coordinates
       * from the source_frame into the target_frame. If the transform is not
       * available (or not supported), return false.
       *
       * @param[in] target_frame Destination frame for transform
       * @param[in] source_frame Source frame for transform
       * @param[in] time Time that the transform is valid for. To get the most
       *    recent transform, use ros::Time(0)
       * @param[out] transform Output container for the transform
       * @return True if the transform was found, false if no transform between
       *    the specified frames is available for the specified time.
       */
      bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const tf2::TimePoint& time,
        Transform& transform) override;

    protected:
      bool Initialize() override;

      std::string local_xy_frame_;
  };

  /**
   * Specialization of TransformImpl for transforming from TF to WGS84
   *
   * This class should not be used directly. It is used internally by
   * swri_transform_util::Transform
   */
  class TfToWgs84Transform : public TransformImpl, public StampedTransformStampInterface
  {
  public:
    /**
     * Create a TfToWgs84Transform from a TF transform and local_xy_util
     *
     * @param[in] transform The TF transform to use as the source frame. This
     *    transform should be the transform from the source frame to the local
     *    XY origin frame
     * @param local_xy_util A local XY Utility object to transform from the
     *    local XY origin frame to WGS84 coordinates
     */
    TfToWgs84Transform(
      const geometry_msgs::msg::TransformStamped& transform,
      std::shared_ptr<LocalXyWgs84Util> local_xy_util);

    /**
     * Transform a 3D vector to latitude/longitude
     *
     * The vector is first transformed with the `transform` into the local
     * XY ortho-rectified frame. Then, the `local_xy_util` is used to convert
     * to latitude/longitude.
     *
     * @param[in]  v_in  Input vector in the 'transform' parent frame.
     * @param[out] v_out Output vector. x is the longitude in degrees, y is the
     *    latitude in degrees, and z is the altitude in meters.
     */
    void Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const override;

    /**
     * Get the orientation of the transform.
     *
     * This is calculated by transforming the reference angle of the
     * `local_xy_util` using the `transform`. The result is the composition
     * of those two rotations, which is the complete rotation from ENU frame
     * to this frame.
     *
     * @return The orientation of the transform
     */
    tf2::Quaternion GetOrientation() const override;

    TransformImplPtr Inverse() const override;

  protected:
    std::shared_ptr<LocalXyWgs84Util> local_xy_util_;
  };

  /**
   * Specialization of TransformImpl for transforming from WGS84 to TF
   *
   * This class should not be used directly. It is used internally by
   * swri_transform_util::Transform
   */
  class Wgs84ToTfTransform : public TransformImpl, public StampedTransformStampInterface
  {
  public:
      /**
       * Create a Wgs84ToTfTransform from a TF transform and local_xy_util
       *
       * @param[in] transform The TF transform to use as the destination frame.
       *    This transform should be the transform from the local XY origin
       *    frame to the destination frame.
       * @param local_xy_util A local XY Utility object to transform from WGS84
       *    coordinates to the local XY origin frame
       */
    Wgs84ToTfTransform(
      const geometry_msgs::msg::TransformStamped& transform,
      std::shared_ptr<LocalXyWgs84Util> local_xy_util);

    /**
     * Transform a WGS84 triple to a 3D vector
     *
     * The vector is first converted from latitude/longitude/altitude to the
     * local XY ortho-rectified frame using the `local_xy_util`. Then, the
     * `transform` is applied.
     *
     * @param[in]  v_in  Input vector. x is the longitude in degrees, y is the
     *    latitude in degrees, and z is the altitude in meters.
     * @param[out] v_out Output vector in the 'transform' child frame.
     */
    void Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const override;

    /**
     * Get the orientation of the transform.
     *
     * This is calculated by transforming the inverse of the reference angle of
     * the `local_xy_util` using the `transform`. The result is the composition
     * of those two rotations, which is the complete rotation from this frame
     * to the ENU frame.
     *
     * @return The orientation of the transform
     */
    tf2::Quaternion GetOrientation() const override;
    TransformImplPtr Inverse() const override;
  protected:
    std::shared_ptr<LocalXyWgs84Util> local_xy_util_;
  };
}

#endif  // TRANSFORM_UTIL_WGS84_TRANSFORMER_H_
