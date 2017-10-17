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

#ifndef TRANSFORM_UTIL_UTM_TRANSFORMER_H_
#define TRANSFORM_UTIL_UTM_TRANSFORMER_H_

#include <map>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <swri_transform_util/utm_util.h>
#include <swri_transform_util/local_xy_util.h>
#include <swri_transform_util/transformer.h>

namespace swri_transform_util
{
  /**
   * Instantiation of Transformer to handle transforms to/from UTM
   */
  class UtmTransformer : public Transformer
  {
    public:
      UtmTransformer();

      /**
       * Get a map of the transforms supported by this Transformer
       * @return A map from source frame IDs to list of destination frame IDs.
       *   A source->destination entry does not imply that the inverse
       *   transform is supported as well.
       */
      virtual std::map<std::string, std::vector<std::string> > Supports() const;


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
      std::string local_xy_frame_;
  };


  /**
   * Class to implement transformation from UTM to TF
   *
   * This class should not be used directly. It is used internally by
   * swri_transform_util::Transform
   */
  class UtmToTfTransform : public TransformImpl
  {
  public:
    UtmToTfTransform(
      const tf::StampedTransform& transform,
      boost::shared_ptr<UtmUtil> utm_util,
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util,
      int32_t utm_zone,
      char utm_band);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

    virtual tf::Quaternion GetOrientation() const;
    virtual TransformImplPtr Inverse() const;

  protected:
    tf::StampedTransform transform_;
    boost::shared_ptr<UtmUtil> utm_util_;
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;
    int32_t utm_zone_;
    char utm_band_;
  };


  /**
   * Class to implement transformation from TF to UTM
   *
   * This class should not be used directly. It is used internally by
   * swri_transform_util::Transform
   */
  class TfToUtmTransform : public TransformImpl
  {
  public:
    TfToUtmTransform(
      const tf::StampedTransform& transform,
      boost::shared_ptr<UtmUtil> utm_util,
      boost::shared_ptr<LocalXyWgs84Util> local_xy_util,
      int32_t utm_zone,
      char utm_band);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;

    virtual tf::Quaternion GetOrientation() const;
    virtual TransformImplPtr Inverse() const;

  protected:
    tf::StampedTransform transform_;
    boost::shared_ptr<UtmUtil> utm_util_;
    boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;
    int32_t utm_zone_;
    char    utm_band_;
  };


  /**
   * Class to implement transformation from UTM to WGS84
   *
   * This class should not be used directly. It is used internally by
   * swri_transform_util::Transform
   */
  class UtmToWgs84Transform : public TransformImpl
  {
  public:
    UtmToWgs84Transform(
        boost::shared_ptr<UtmUtil> utm_util,
        int32_t utm_zone,
        char utm_band);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;
    virtual TransformImplPtr Inverse() const;

  protected:
    boost::shared_ptr<UtmUtil> utm_util_;
    int32_t utm_zone_;
    char    utm_band_;
  };

  /**
   * Class to implement transformation from WGS84 to UTM
   *
   * This class should not be used directly. It is used internally by
   * swri_transform_util::Transform
   */
  class Wgs84ToUtmTransform : public TransformImpl
  {
  public:
    explicit Wgs84ToUtmTransform(
        boost::shared_ptr<UtmUtil> utm_util,
        int32_t utm_zone,
        char utm_band);

    virtual void Transform(const tf::Vector3& v_in, tf::Vector3& v_out) const;
    virtual TransformImplPtr Inverse() const;

  protected:
    boost::shared_ptr<UtmUtil> utm_util_;
    int32_t utm_zone_;
    char    utm_band_;
  };
}

#endif  // TRANSFORM_UTIL_UTM_TRANSFORMER_H_
